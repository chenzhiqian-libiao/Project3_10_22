#include "Feixiang_can.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include "semphr.h"

static SemaphoreHandle_t transmitMutex;
HAL_StatusTypeDef CAN_TransmitMutexInit(void)
{
    transmitMutex = xSemaphoreCreateMutex();
    return transmitMutex ? HAL_OK : HAL_ERROR;
}


/* CAN1 使用默认映射（不重映射）：PA11 = CAN_RX，PA12 = CAN_TX
   外部收发器 PCA82C251 */
#define CAN_GPIO_PORT               GPIOA
#define CAN_RX_PIN                  GPIO_PIN_11
#define CAN_TX_PIN                  GPIO_PIN_12

#define CAN_FILTER_BANK             0U
#define CAN_SLAVE_START_FILTER_BANK 14U

static CAN_HandleTypeDef hcan1;
static uint8_t canInitialized;
static uint8_t canBusOffActive;

/* ---- 调试诊断变量 ---- */
volatile uint32_t canTxOkCount = 0U;
volatile uint32_t canTxErrCount = 0U;
volatile uint32_t canTxMutexTimeoutCount = 0U;
volatile uint32_t canTxMailboxWaitCount = 0U;
volatile uint32_t canTxMailboxTimeoutCount = 0U;
volatile uint32_t canRxOkCount = 0U;
volatile uint32_t canRxErrCount = 0U;
volatile uint32_t canRxFifo0OverrunCount = 0U;
volatile uint32_t canRxFifo1OverrunCount = 0U;
volatile uint32_t canEsr = 0U;
volatile uint32_t canFreeMailboxes = 3U;
volatile uint8_t canRxData = 0U;
volatile uint8_t canRxReady = 0U;
volatile uint8_t canMode = 0U;
volatile uint32_t canBusOffCount = 0U;
volatile uint8_t canRxPinLevel = 0U;

/* 由 HAL_CAN_Init() 自动调用：打开时钟并配置 CAN 引脚 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (hcan->Instance == CAN1)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_AFIO_CLK_ENABLE();
		__HAL_RCC_CAN1_CLK_ENABLE();

		/* PA12: CAN1_TX，复用推挽，高速 */
		GPIO_InitStruct.Pin = CAN_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStruct);

		/* PA11: CAN1_RX，输入（不用复用模式，AF 由外设直接取用）
		   不加内部上拉：收发器 RXD 是推挽输出，总线空闲时它把引脚驱到高电平；
		   若收发器没供电，引脚悬空会被 canRxPinLevel 采到 0，正好能暴露问题 */
		GPIO_InitStruct.Pin = CAN_RX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStruct);
	}
}

HAL_StatusTypeDef CAN_Init(void)
{
	CAN_FilterTypeDef filterConfig = {0};

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = CAN_PRESCALER;
#if (CAN_LOOPBACK_TEST != 0U)
	/* 回环自检：帧在芯片内部回连，自己应答自己，不依赖外部节点和收发器供电 */
	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	canMode = 1U;
#else
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	canMode = 0U;
#endif
	hcan1.Init.SyncJumpWidth = CAN_SYNC_JUMP_WIDTH;
	hcan1.Init.TimeSeg1 = CAN_TIME_SEG1;
	hcan1.Init.TimeSeg2 = CAN_TIME_SEG2;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	/* Single-shot hardware TX; bounded task-level retry on arbitration loss only.
	 * Never leave an unbounded movement request queued on a disconnected bus. */
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* 相同模式/位宽时低编号滤波器优先：bank0精确匹配标准数据帧041A到FIFO1，
	 * bank1把其余帧送FIFO0，避免自动位置/力矩等反馈挤掉关键ACK。 */
	filterConfig.FilterBank = CAN_FILTER_BANK;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	filterConfig.FilterIdHigh = (0x041AU << 5);
	filterConfig.FilterIdLow = 0x0000U;
	filterConfig.FilterMaskIdHigh = 0xFFE0U;
	filterConfig.FilterMaskIdLow = 0x0006U; /* IDE=0, RTR=0 */
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.SlaveStartFilterBank = CAN_SLAVE_START_FILTER_BANK;

	if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK)
	{
		return HAL_ERROR;
	}

	filterConfig.FilterBank = CAN_FILTER_BANK + 1U;
	filterConfig.FilterIdHigh = 0U;
	filterConfig.FilterMaskIdHigh = 0U;
	filterConfig.FilterMaskIdLow = 0U;
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK) return HAL_ERROR;

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		return HAL_ERROR;
	}

	canInitialized = 1U;
	canBusOffActive = 0U;
	canFreeMailboxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

	return HAL_OK;
}

/* 发送一帧经典 CAN 标准数据帧（非 CAN FD、非扩展帧） */
HAL_StatusTypeDef CAN_SendFrame(uint16_t std_id, const uint8_t *data, uint8_t len)
{
	return CAN_SendFrameWait(std_id, data, len, 0U);
}

static HAL_StatusTypeDef CAN_SendFrameUnlocked(uint16_t std_id, const uint8_t *data, uint8_t len, uint32_t timeoutMs)
{
	CAN_TxHeaderTypeDef txHeader;
	/* HAL_CAN_AddTxMessage() 固定读取 8 个字节，缓冲区必须是 8 字节 */
	uint8_t txData[8] = {0};
	uint32_t txMailbox = 0U;
	HAL_StatusTypeDef status;
	uint8_t i;
	uint32_t waitStart;

	if ((canInitialized == 0U) || (std_id > 0x7FFU) || (data == NULL) ||
		(len == 0U) || (len > 8U))
	{
		canTxErrCount++;
		return HAL_ERROR;
	}

	for (i = 0U; i < len; i++)
	{
		txData[i] = data[i];
	}

	/* AutoBusOff 已启用，由硬件自行恢复；不在 TX 任务里重初始化外设，
	   避免与独立 RX 任务同时操作同一个 CAN 句柄。 */
	if ((hcan1.Instance->ESR & CAN_ESR_BOFF) != 0U)
	{
		if (canBusOffActive == 0U)
		{
			canBusOffCount++;
			canBusOffActive = 1U;
		}
	}
	else
	{
		canBusOffActive = 0U;
	}

	txHeader.StdId = (uint32_t)std_id;
	txHeader.ExtId = 0U;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = (uint32_t)len;
	txHeader.TransmitGlobalTime = DISABLE;

	if ((timeoutMs != 0U) && (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U))
	{
		canTxMailboxWaitCount++;
		waitStart = HAL_GetTick();
		while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
		{
			if ((uint32_t)(HAL_GetTick() - waitStart) >= timeoutMs)
			{
				canEsr = hcan1.Instance->ESR;
				canFreeMailboxes = 0U;
				canTxMailboxTimeoutCount++;
				canTxErrCount++;
				return HAL_TIMEOUT;
			}
		}
	}

	/* Original transport: success means submitted to a mailbox.
	 * Motor command acceptance is checked separately through 0x041A. */
	status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

	/* 更新诊断变量 */
	canEsr = hcan1.Instance->ESR;
	canFreeMailboxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	if (status == HAL_OK)
	{
		canTxOkCount++;
	}
	else
	{
		canTxErrCount++;
	}

	return status;
}

/* 测试接口：以标准 ID 0x123 发送 1 个字节 */
HAL_StatusTypeDef CAN_SendByte(uint8_t data)
{
	return CAN_SendFrame(CAN_TEST_STD_ID, &data, 1U);
}

/* 接收一帧：优先读取FIFO1中的ACK，然后读取FIFO0；仅处理标准数据帧。
   两个FIFO都空时返回HAL_TIMEOUT。 */
static HAL_StatusTypeDef receive_frame(uint16_t *std_id, uint8_t *data, uint8_t *len, int ackOnly)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint32_t rxFifo;
	uint8_t rxData[8] = {0};
	uint8_t i;

	if ((std_id == NULL) || (data == NULL) || (len == NULL))
	{
		return HAL_ERROR;
	}

	if (canInitialized == 0U)
	{
		canRxErrCount++;
		return HAL_ERROR;
	}
	*len = 0U;

	if (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV0)) {
		++canRxFifo0OverrunCount; __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
	}
	if (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV1)) {
		++canRxFifo1OverrunCount; __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV1);
	}
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1) != 0U) rxFifo = CAN_RX_FIFO1;
	else if (!ackOnly && HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0U) rxFifo = CAN_RX_FIFO0;
	else return HAL_TIMEOUT;

	if (HAL_CAN_GetRxMessage(&hcan1, rxFifo, &rxHeader, rxData) != HAL_OK)
	{
		canRxErrCount++;
		return HAL_ERROR;
	}

	/* 只处理标准数据帧 */
	if ((rxHeader.IDE != CAN_ID_STD) || (rxHeader.RTR != CAN_RTR_DATA))
	{
		canRxErrCount++;
		return HAL_ERROR;
	}

	if (rxHeader.DLC > 8U)
	{
		canRxErrCount++;
		return HAL_ERROR;
	}

	*std_id = (uint16_t)rxHeader.StdId;
	*len = (uint8_t)rxHeader.DLC;

	for (i = 0U; i < *len; i++)
	{
		data[i] = rxData[i];
	}

	canRxOkCount++;
	return HAL_OK;
}

HAL_StatusTypeDef CAN_ReceiveAckFrame(uint16_t *id, uint8_t *data, uint8_t *len)
{ return receive_frame(id, data, len, 1); }
HAL_StatusTypeDef CAN_ReceiveFrame(uint16_t *id, uint8_t *data, uint8_t *len)
{ return receive_frame(id, data, len, 0); }

/* 接收 1 个字节：无数据时返回 HAL_TIMEOUT */
HAL_StatusTypeDef CAN_ReceiveByte(uint8_t *data)
{
	uint16_t stdId = 0U;
	uint8_t payload[8] = {0};
	uint8_t len = 0U;
	HAL_StatusTypeDef status;

	if (data == NULL)
	{
		return HAL_ERROR;
	}

	/* 采样 CAN_RX 引脚电平（PA11）：总线空闲（隐性）时应为 1。
	   一直读到 0，说明总线被持续拉低，或收发器没有 +5V 供电（RXD 悬空）——
	   这是判断"收发器到底有没有在工作"最直接的一个数 */
	canRxPinLevel = (HAL_GPIO_ReadPin(CAN_GPIO_PORT, CAN_RX_PIN) == GPIO_PIN_SET) ? 1U : 0U;

	status = CAN_ReceiveFrame(&stdId, payload, &len);
	if (status != HAL_OK)
	{
		return status;
	}

	if (len != 1U)
	{
		canRxErrCount++;
		return HAL_ERROR;
	}

	*data = payload[0];
	return HAL_OK;
}

HAL_StatusTypeDef CAN_SendFrameWait(uint16_t id, const uint8_t *data, uint8_t len, uint32_t timeoutMs)
{
    HAL_StatusTypeDef result;
    if (transmitMutex && xSemaphoreTake(transmitMutex, pdMS_TO_TICKS(10U)) != pdTRUE) {
        ++canTxMutexTimeoutCount;
        return HAL_TIMEOUT;
    }
    result = CAN_SendFrameUnlocked(id, data, len, timeoutMs);
    if (transmitMutex) xSemaphoreGive(transmitMutex);
    return result;
}
