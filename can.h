#ifndef CAN_H
#define CAN_H

#include <stm32f1xx_hal.h>

/* CAN1, default mapping (no remap): PA11 = CAN_RX, PA12 = CAN_TX.
   External transceiver: PCA82C251.
   模块开关：是否初始化由 Project1_10_13.c 顶部的旧 CAN 测试开关或
             MODULE_TASK6_CAN_ENABLE 决定；关闭所有 CAN 功能时不调用 CAN_Init()
   系统时钟 72 MHz，APB1 分频 2 -> PCLK1 = 36 MHz
   36 MHz / [4 x (1 + 5 + 3)] = 1 Mbps, 采样点 (1+5)/9 = 66.7% */
#define CAN_PRESCALER               4U
#define CAN_SYNC_JUMP_WIDTH         CAN_SJW_1TQ
#define CAN_TIME_SEG1               CAN_BS1_5TQ
#define CAN_TIME_SEG2               CAN_BS2_3TQ

/* 测试用标准帧 ID：CAN_SendByte() 使用它发送 */
#define CAN_TEST_STD_ID             0x123U

/* 自检开关：置 1 时 CAN 工作在回环模式（CAN_MODE_LOOPBACK）——
   发出去的帧在芯片内部直接回连给接收端，不需要总线上有第二个节点，
   也不需要接 USB-CAN 工具，与板外的供电、接线、终端电阻全部无关。
   用途：把"板子和程序的问题"与"外部接线/工具设置的问题"一刀切开。
   【自检步骤】把下面改成 1U → 编译烧写 → Watch 窗口看：
       canMode = 1、canRxOkCount 每 0.5s 加 1、canRxData = 0xAA
     出现这些值说明 MCU + 驱动 + 位时序 + 滤波器都正常，问题在板外
     （收发器供电 / CANH CANL 接线 / USB-CAN 工具设置）。
   ★ 自检做完必须改回 0U 再重新编译烧写，恢复正常总线模式，
     否则 TX 引脚不往总线发数据，CANH/CANL 上量不到波形 */
#define CAN_LOOPBACK_TEST           0U

HAL_StatusTypeDef CAN_Init(void);
/* Call after sensor init, before scheduler start. */
HAL_StatusTypeDef CAN_TransmitMutexInit(void);
HAL_StatusTypeDef CAN_SendFrame(uint16_t std_id, const uint8_t *data, uint8_t len);
/* 有界等待空闲邮箱；HAL_OK仅表示提交成功，041A由控制层检查。
 * timeoutMs=0 不等待空闲邮箱。 */
HAL_StatusTypeDef CAN_SendFrameWait(uint16_t std_id, const uint8_t *data, uint8_t len,
								   uint32_t timeoutMs);
HAL_StatusTypeDef CAN_SendByte(uint8_t data);
HAL_StatusTypeDef CAN_ReceiveAckFrame(uint16_t *id, uint8_t *data, uint8_t *len);
HAL_StatusTypeDef CAN_ReceiveFrame(uint16_t *std_id, uint8_t *data, uint8_t *len);
HAL_StatusTypeDef CAN_ReceiveByte(uint8_t *data);

/* ---- 调试诊断变量（Watch 窗口直接观察）----
   canTxOkCount     : API返回成功的帧数；非阻塞调用只表示入邮箱
   canTxErrCount    : 发送失败次数（邮箱满 / 参数错误）
   canRxOkCount     : 成功收到的标准数据帧数
   canRxErrCount    : 接收失败次数（HAL 错误或非标准数据帧）
   canEsr           : CAN_ESR 寄存器；bit15-8 = TEC 发送错误计数，
                      TEC 持续增长说明总线上没有节点应答（波特率/接线问题）
   canFreeMailboxes : 空闲发送邮箱数（0 = 3 个邮箱都被占用）
   canRxData        : task_can_rx 最后收到的 1 个字节
   canRxReady       : 收到新数据时置 1（可在 task_can_rx 中设断点观察）
   canMode          : 当前模式：1 = 回环自检，0 = 正常总线模式（由 CAN_LOOPBACK_TEST 决定）
   canBusOffCount   : 进入总线关闭并自行恢复的次数；总线没人应答时会不断增长
   canRxPinLevel    : PA11(CAN_RX) 采样电平，总线空闲时应为 1（隐性）。
                      一直为 0 = 收发器没供电（RXD 悬空）或总线被持续拉低 */
extern volatile uint32_t canTxOkCount;
extern volatile uint32_t canTxErrCount;
/* Separate from hardware TX failures: the frame never reached a mailbox. */
extern volatile uint32_t canTxMutexTimeoutCount;
extern volatile uint32_t canTxMailboxWaitCount;
extern volatile uint32_t canTxMailboxTimeoutCount;
extern volatile uint32_t canRxOkCount;
extern volatile uint32_t canRxErrCount;
/* 硬件溢出标志观察次数（不是精确丢帧数）；FIFO1专收041A ACK。 */
extern volatile uint32_t canRxFifo0OverrunCount;
extern volatile uint32_t canRxFifo1OverrunCount;
extern volatile uint32_t canEsr;
extern volatile uint32_t canFreeMailboxes;
extern volatile uint8_t canRxData;
extern volatile uint8_t canRxReady;
extern volatile uint8_t canMode;
extern volatile uint32_t canBusOffCount;
extern volatile uint8_t canRxPinLevel;

#endif
