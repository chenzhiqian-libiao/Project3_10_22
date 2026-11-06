/* 电机/TOF调试及原推拉箱流程，新增独立三轴回零初始化。
 * Watch 写入 flyCommand：
 * 9=一键执行3 -> 10 -> 20 -> 7 -> 8，任一步失败/停止则中止。
 * 1=前轴初始化；2=底座初始化；3=按flyHomeAxisMask初始化三轴；4=钩子初始化。
 * 上电默认不动；先3初始化成功，再按原操作执行7拉箱、8推箱。
 * 6=回到IDLE（清故障/重置接收溢出基线）；
 * 5=停止全部电机；
 * 7=完整动作流程：底座正对箱子->前推箱->钩箱->前回原位->放箱->回收底座
 *       （进度flySeqStep 1~6；结果flySeqResult：2=完成 3=某步失败 5=停止打断）；
 * 10=一键绑定+配置+使能电机01~05；
 * 11~15=分别绑定+配置+使能电机01~05；
 * 20=一键配置TOF25+TOF26并开启持续测距；
 * 25/26=分别绑定+配置TOF25/TOF26并开启持续测距；
 * 31~35=分别测试电机01~05正向点动；
 * 41~45=分别测试电机01~05反向点动；
 * 34/44=底座（电机04）动作：44=底座正对箱子，34=回收底座（从初始位置起）；
 * 31/41=前（电机01）动作：41=向前推箱子（最大行程），31=回到原位（从初始位置起）；
 * 36/48=左右皮带（电机02/03）同步动作：36=履带向外送箱子，48=履带向里送箱子；
 * 46/47=抓钩（电机05）动作：46=抬起钩住箱子，47=放下箱子（从抓钩初始位置起）。
 */
#include "stm32f1xx_hal.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>
#include <string.h>

/* ===================== 调试参数区 ===================== */
static const uint16_t wheel001mm[6] = {0,5000,5000,5000,5000,5000};
static const int8_t pullSign[6] = {0,1,-1,1,1,1};
static const uint8_t motorParam[6] = {0xB4,0xBF,0xCF,0xAA,0x00,0x5F};
static const uint8_t deviceIds[7] = {1,2,3,4,5,25,26};
/* 电机设备号映射（dev01~05）：
 * 01=前，02=右后，03=左后，04=底座，05=夹爪。 */
static const uint8_t sn[7][7] = {
    {0x18,0x03,0x1C,0x07,0xDA,0xFD,0x00}, /* 电机01 前 */
    {0x17,0x0B,0x1B,0x07,0x85,0x15,0x00}, /* 电机02 右后 */
    {0x17,0x0B,0x1C,0x07,0xF3,0x17,0x00}, /* 电机03 左后 */
    {0x19,0x0C,0x03,0x07,0x68,0x2D,0x00}, /* 电机04 底座 */
    {0x19,0x0C,0x03,0x07,0x8D,0x2D,0x00}, /* 电机05 夹爪 */
    {0x1A,0x0A,0x1D,0x2A,0x7E,0x45,0x00}, /* TOF25 */
    {0x18,0x07,0x1B,0x07,0xC2,0x86,0x00}  /* TOF26 */
};

#define FB_ACK_TIMEOUT_MS      150U
#define FB_TOF_POLL_MS         50U
#define FB_TOF_INIT_TIMEOUT_MS 2500U
#define FB_TOF_STALE_MS        300U
#define FB_TEST_SPEED          200U  /* 0.1mm/s */
#define FB_TEST_DISTANCE       300   /* 0.1mm */
#define FB_TEST_TIMEOUT_MS     8000U
#define FB_TEST_TOLERANCE      5
#define FB_RX_OVERFLOW_STRICT  0U    /* 1=任意溢出立刻故障；0=调试容忍窗口 */
#define FB_RX_OVERFLOW_WINDOW_MS 500U
#define FB_RX_OVERFLOW_BURST_LIMIT 3U
#define FB_RX_OVERFLOW_FAULT_ENABLE 0U /* 0=仅统计不故障，1=超过阈值后故障 */
#define FB_IGNORE_ERR_FRAME_010X 1U   /* 1=忽略0x0100~0x010F，不置故障 */

#define FB_HOME_LIMIT_INIT     0x55U   /* 初始化阶段最大电流档：顺/逆时针各5档（0x33=各3档带不动钩爪05，现场实测） */
#define FB_HOME_LIMIT_RUN      0xAAU   /* 正常动作最大电流档：顺/逆时针各10档 */
#define FB_HOME_POLL_MS        20U     /* 回零时查询绝对位置的周期 */
#define FB_HOME_BOOT_DELAY_MS  1500U   /* 上电后等电源稳定/人员离开再自动回零 */
#define FB_HOME_SETTLE_MS      150U    /* 判定堵转后等电机停稳 */
#define FB_HOME_STALL_MS       300U    /* 位置反馈连续不动多久判堵转 */
#define FB_HOME_START_GRACE_MS 500U    /* 起步宽限：这段时间内不判位置停滞 */
#define FB_HOME_TORQUE_HITS    3U      /* 连续N帧力矩超阈值判堵转 */
#define FB_HOME_TORQUE_IGNORE_MS 150U  /* 起步阶段忽略力矩，避开启动冲击 */
#define FB_HOME_TIMEOUT_MS     30000U  /* 单轴回零最长时间 */
#define FB_HOME_START_TIMEOUT_MS 2500U /* 起步超时：这段时间还没有有效位移就判失败，不再等满整段超时 */
#define FB_HOME_REQUIRED_MASK  0x19U   /* 完整初始化需要的轴：01前 + 04底座 + 05钩子 */
/* flyHomeStopReason 取值：1=力矩 2=位置停滞 3=错误帧 4=预定行程走完仍未撞到死点 5=起步超时
 *   6=驱动器不回设置帧(0x0414/0x0415) 7=初始化电流档反向轻退不动 8=换0xAA也退不动(机械压死/方向不对) */
#define FB_HOME_STOP_NO_DEADPOINT 4U
#define FB_HOME_STOP_NO_START     5U

#define FB_CMD_HOME_FRONT 1U
#define FB_CMD_HOME_BASE 2U
#define FB_CMD_HOME 3U
#define FB_CMD_HOME_HOOK 4U

#define FB_CMD_STOP            5U
#define FB_CMD_IDLE            6U
#define FB_CMD_SEQUENCE        7U
#define FB_CMD_PUSH_SEQUENCE   8U
#define FB_CMD_COMBINED        9U
#define FB_CMD_ENABLE_ALL      10U
#define FB_CMD_ENABLE_FIRST    11U
#define FB_CMD_ENABLE_LAST     15U
#define FB_CMD_TOF_ALL         20U
#define FB_CMD_TOF25           25U
#define FB_CMD_TOF26           26U
#define FB_CMD_TEST_FWD_FIRST  31U
#define FB_CMD_TEST_FWD_LAST   35U
#define FB_CMD_TEST_REV_FIRST  41U
#define FB_CMD_TEST_REV_LAST   45U
#define FB_CMD_GRIP_FWD        46U
#define FB_CMD_GRIP_REV        47U
#define FB_CMD_BASE_FWD        34U
#define FB_CMD_BASE_REV        44U
#define FB_CMD_FRONT_FWD       31U
#define FB_CMD_FRONT_REV       41U
#define FB_CMD_BELT_FWD        36U
#define FB_CMD_BELT_REV        48U

#define FB_SEQ_TOF25_OFFSET    1000U
#define FB_SEQ_TOF25_FULL_SPAN 4300U
#define FB_SEQ_TOF26_OFFSET    800U
#define FB_SEQ_TOF26_FULL_SPAN 4800U

typedef enum {
    FB_STATE_OFF=0, FB_STATE_READY=1, FB_STATE_BUSY=2, FB_STATE_FAULT=90
} FlyState;
typedef enum {
    FB_OK=0, FB_ERR_BAD_COMMAND, FB_ERR_TX, FB_ERR_ACK, FB_ERR_TOF,
    FB_ERR_MOTOR, FB_ERR_RX_OVERFLOW, FB_ERR_ACK_OVERFLOW, FB_ERR_TIMEOUT,
    FB_ERR_ALLOCATION
} FlyError;
typedef struct { uint16_t id; uint8_t dev; } Ack;
typedef struct {
    int32_t remaining;
    uint32_t remSeq, remMs;
    int32_t pos;
    uint32_t posSeq, posMs;
    int16_t torque;
    uint32_t torqueSeq, torqueMs;
} MotorFeedback;
typedef struct {
    uint16_t raw;
    uint8_t valid, stable;
    uint32_t seq, ms, rxSeq, rxMs;
} TofFeedback;

volatile uint32_t flyCommand;
volatile uint32_t flyLastCmd;
volatile int32_t flyTestDist;
volatile int32_t flyRem[6];
volatile uint32_t flyRemSeq[6];
volatile uint32_t flySeqStep;
volatile uint32_t flySeqResult;
volatile uint32_t flyFlowStep; /* 1~5对应3/10/20/7/8，失败保留所在阶段 */
volatile uint32_t flyFlowCommand; /* 当前/最后执行的子命令 */
volatile uint32_t flyFlowResult; /* 0未执行 1运行 2成功 3失败 5停止 */
volatile int32_t flyCustomDist = -22000;
volatile int32_t flyCustomSpeed = 20000;
volatile int32_t flyCustomAccel = 100000;
volatile int32_t flyBaseDist = 14000;
volatile int32_t flyBaseSpeed = 20000;
volatile int32_t flyBaseAccel = 20000;
volatile int32_t flyFrontDist = 45000;
volatile int32_t flyFrontSpeed = 15000;
volatile int32_t flyFrontAccel = 20000;
volatile int32_t flyFrontPrimeDist = 10000;
volatile int32_t flyFrontGoDist;
volatile int32_t flyBeltDist = 300;
volatile int32_t flyBeltSpeed = 5000;
volatile int32_t flyBeltAccel = 20000;
volatile int32_t flyBeltPullDist = 10000;
volatile int32_t flyBeltMapDist = 10000;
volatile int32_t flyBeltTof26GoDist;
/* ===================== 上电初始化（回零）参数：Live Watch 可调 ===================== */
volatile uint8_t  flyAutoHome = 0U;          /* 0=上电不动，等命令；1=上电延时后自动初始化 */
volatile uint32_t flyHomeAxisMask = 0x19U;   /* bit0=01前 bit3=04底座 bit4=05钩子；默认0x19=三轴都初始化 */
volatile int32_t  flyHomeSpeed = 5000;       /* 回零速度 0.1mm/s（500mm/s） */
volatile int32_t  flyHomeAccel = 20000;      /* 回零加速度 0.1mm/s^2 */
volatile uint8_t  flyHomeCurLimit = FB_HOME_LIMIT_INIT; /* 初始化阶段电流档，默认0x55（顺/逆各5档） */
/* 回零方向的最大行程（0.1mm），必须大于该轴实际可走行程，否则走不到死点 */
volatile int32_t  flyHomeDist[6] = {0,50000,0,0,25000,35000};
/* 死点→你想要的初始位置的距离（0.1mm），负数按绝对值处理；0=死点就是初始位置。
 * 前轴直接以死点为初始位置，不再退出；底座2100mm、钩子1000mm。 */
volatile int32_t  flyHomeBackoff[6] = {0,0,0,0,21000,10000};
/* 回零前先反向轻退的验证距离（0.1mm）：确认初始化电流档真的能让轴动，
 * 避免"动不了"被误判成死点；0=关闭验证 */
volatile int32_t  flyHomeProbeDist = 50;
/* 力矩判据阈值（0x0440+设备号 的 s16 原始值）：0=关闭，只用位置停滞判据；
 * 先用回零时的 flyHomeTorquePeak 读峰值，再设成峰值的 70%~80% */
volatile int32_t  flyHomeTorqueThr = 0;
volatile uint32_t flyHomeTorqueHits = FB_HOME_TORQUE_HITS;
volatile uint32_t flyHomeStallMs = FB_HOME_STALL_MS;
volatile uint32_t flyHomeTimeoutMs = FB_HOME_TIMEOUT_MS;
volatile uint8_t  flyHomed;                  /* 1=三轴初始化成功；不改变原命令7/8的调度 */
volatile uint32_t flyHomedMask;              /* 已初始化成功的轴：bit0=01 bit3=04 bit4=05 */
volatile uint32_t flyHomeAxis;               /* 当前/最后回零轴 0=无 */
volatile uint32_t flyHomeStep;               /* 1使能 2降流 3找死点 4停稳 5退初始位 6完成 */
volatile uint32_t flyHomeResult;             /* 0未执行 1执行中 2成功 3失败 4停止 5未使能 6参数不支持 */
volatile uint32_t flyHomeStopReason;         /* 死点判据：1=力矩 2=位置停滞 3=错误帧 4=走完行程仍未到死点 5=起步超时 */
volatile int32_t  flyHomeDeadPos[6];         /* 死点位置 P_ref（0.1mm） */
volatile int32_t  flyHomeInitPos[6];         /* 初始化位置 P_init（0.1mm） */
volatile int32_t  flyHomeBackoffUsed[6];     /* 本次实际下发的后退距离：0=没退（检查是不是取值源头没生效） */
volatile int32_t  flyHomeInitSpan[6];        /* 实测后退距离 |P_init-P_ref|，用来和 BackoffUsed 对照 */
volatile int32_t  flyHomeSearchSpan[6];      /* 找死点阶段实际走了多远：≈0 却报 reason=2 就是"假死点"（电流带不动） */
volatile int32_t  flyHomeTorquePeak[6];      /* 回零过程力矩峰值，用于标定 flyHomeTorqueThr */
volatile uint32_t flyHomeRescueCount;        /* 初始化档退不动、改用 AA 自救的次数（>0 说明轴曾被压死在死点） */
volatile int32_t  flyTorque[6];              /* 最近一次力矩反馈（s16 原始值），Watch观察 */
volatile uint32_t flyTorqueRxCount;          /* 收到力矩帧的累计次数：0=驱动器不发，力矩判据不生效 */
volatile uint8_t  flyMotorCurLimit = FB_HOME_LIMIT_RUN; /* 当前下发给驱动器的最大电流档，Watch观察 */
volatile int32_t homeDir[6] = {0,1,0,0,-1,1};
volatile uint32_t flyLastMotorErrorMs;
static uint8_t homeActive;

volatile FlyState flyState = FB_STATE_OFF;
volatile FlyError flyError = FB_OK;
volatile FlyError flyRejected = FB_OK;
volatile uint32_t flyAckWaitId, flyAckWaitDevice;
volatile uint32_t flyLastAckId, flyLastAckDevice, flyMalformedFrames;
volatile uint32_t flyEnabledMask;
volatile uint32_t flyEnableDevice, flyEnableStep, flyEnableResult;
volatile uint32_t flyTofDevice, flyTofStep, flyTofResult;
volatile uint32_t flyTestDevice, flyTestStep, flyTestResult;
volatile uint32_t flyTof25Raw, flyTof26Raw;
volatile uint32_t flySnSeenCount;
volatile uint8_t flySnLast[7];
volatile uint32_t flyLastMotorError, flyLastMotorErrorCanId;
volatile uint32_t flyTaskRxLoops, flyTaskControlLoops;
volatile uint32_t flyRxOverflowTotal, flyRxOverflowBurst;

static QueueHandle_t ackQueue;
static volatile MotorFeedback motor[6];
static volatile TofFeedback tof[2];
static volatile int32_t axisPos[6];
static volatile uint32_t axisPosSeq[6];
static volatile uint32_t tofPollingMask;
static volatile FlyError rxFault;
static uint32_t fifo0Baseline, fifo1Baseline;
static uint32_t rxOverflowWindowStartMs;

static void SystemClock_Config(void);
static void Error_Handler(void);
extern void xPortSysTickHandler(void);

static uint32_t Now(void) { return HAL_GetTick(); }
static void Sleep(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1U)); }
static int32_t Abs32(int32_t n) { return n<0 ? -n : n; }
static void Put32(uint8_t *p, int32_t n)
{
    uint32_t v=(uint32_t)n;
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
static int32_t Get32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1]<<8) |
                     ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24));
}
static TofFeedback ReadTof(uint8_t i)
{
    TofFeedback f;
    taskENTER_CRITICAL(); f=tof[i]; taskEXIT_CRITICAL(); return f;
}

static void ParseFrame(uint16_t id, const uint8_t *d, uint8_t len)
{
    uint32_t now=Now();
    if (id==0x041AU) {
        if (len!=3U || d[2]<1U || d[2]>5U) { ++flyMalformedFrames; return; }
        Ack a={(uint16_t)(d[0]|((uint16_t)d[1]<<8)),d[2]};
        flyLastAckId=a.id; flyLastAckDevice=a.dev;
        if (xQueueSend(ackQueue,&a,0)!=pdTRUE) rxFault=FB_ERR_ACK_OVERFLOW;
        return;
    }
    if (id>=0x0421U && id<=0x0425U) {
        uint8_t dev=(uint8_t)(id-0x0420U);
        if (len!=8U) { ++flyMalformedFrames; return; }
        taskENTER_CRITICAL();
        motor[dev].remaining=Get32(d); motor[dev].remMs=now; ++motor[dev].remSeq;
        flyRem[dev]=motor[dev].remaining; flyRemSeq[dev]=motor[dev].remSeq;
        taskEXIT_CRITICAL();
        return;
    }
    if (id>=0x0441U && id<=0x0445U) {
        /* 力矩反馈：4字节，Data[2~3]=当前扭矩 s16，用于回零判死点 */
        uint8_t dev=(uint8_t)(id-0x0440U);
        int16_t t;
        if (len<4U) { ++flyMalformedFrames; return; }
        t=(int16_t)(uint16_t)(d[2]|((uint16_t)d[3]<<8));
        taskENTER_CRITICAL();
        motor[dev].torque=t; motor[dev].torqueMs=now; ++motor[dev].torqueSeq;
        flyTorque[dev]=(int32_t)t;
        taskEXIT_CRITICAL();
        ++flyTorqueRxCount;
        return;
    }
    if (id==0x0409U && len>=2U) {
        if (d[0]>=1U && d[0]<=5U && d[1]==8U && (len==6U || len==8U)) {
            taskENTER_CRITICAL();
            axisPos[d[0]]=Get32(d+2); ++axisPosSeq[d[0]];
            motor[d[0]].pos=axisPos[d[0]]; ++motor[d[0]].posSeq; motor[d[0]].posMs=now;
            taskEXIT_CRITICAL();
            return;
        }
        if ((d[0]==25U || d[0]==26U) && d[1]==12U && len>=5U) {
            uint8_t i=(uint8_t)(d[0]-25U);
            uint16_t raw=(uint16_t)(d[2]|((uint16_t)d[3]<<8));
            taskENTER_CRITICAL();
            tof[i].rxMs=now; ++tof[i].rxSeq;
            if (raw<=10U || raw==8888U || raw==9999U || (d[4]!=1U && d[4]!=0xFFU)) {
                tof[i].valid=0; tof[i].stable=0;
            } else {
                uint8_t stable=tof[i].stable;
                if ((uint32_t)(now-tof[i].ms)>FB_TOF_STALE_MS ||
                    Abs32((int32_t)raw-(int32_t)tof[i].raw)>50) stable=0;
                tof[i].raw=raw; tof[i].valid=1;
                tof[i].stable=(uint8_t)(stable<3U ? stable+1U : 3U);
                tof[i].ms=now; ++tof[i].seq;
                if (i==0U) flyTof25Raw=raw; else flyTof26Raw=raw;
            }
            taskEXIT_CRITICAL();
        }
        return;
    }
    if ((id==0x0793U || id==0x0794U) && len>=4U && (d[0]==25U || d[0]==26U)) {
        uint8_t i=(uint8_t)(d[0]-25U);
        uint16_t raw=(uint16_t)(d[2]|((uint16_t)d[3]<<8));
        taskENTER_CRITICAL();
        tof[i].rxMs=now; ++tof[i].rxSeq;
        if (raw<=10U || raw==8888U || raw==9999U) {
            tof[i].valid=0; tof[i].stable=0;
        } else {
            uint8_t stable=tof[i].stable;
            if ((uint32_t)(now-tof[i].ms)>FB_TOF_STALE_MS ||
                Abs32((int32_t)raw-(int32_t)tof[i].raw)>50) stable=0;
            tof[i].raw=raw; tof[i].valid=1;
            tof[i].stable=(uint8_t)(stable<3U ? stable+1U : 3U);
            tof[i].ms=now; ++tof[i].seq;
            if (i==0U) flyTof25Raw=raw; else flyTof26Raw=raw;
        }
        taskEXIT_CRITICAL();
        return;
    }
    if (id>=0x0100U && id<=0x017FU) {
        if (len!=2U) { ++flyMalformedFrames; return; }
        flyLastMotorErrorMs=now;
        flyLastMotorErrorCanId=id;
        flyLastMotorError=(uint32_t)d[0]|((uint32_t)d[1]<<8);
        if (FB_IGNORE_ERR_FRAME_010X && id<=0x010FU) return;
        rxFault=FB_ERR_MOTOR;
        return;
    }
    if (id==0x0312U && len==7U) { ++flySnSeenCount; memcpy((void *)flySnLast,d,7); }
}
static void ReceiveTask(void *arg)
{
    (void)arg;
    for (;;) {
        uint16_t id; uint8_t d[8],len; unsigned n=0, got=0;
        while (n++<256U && CAN_ReceiveFrame(&id,d,&len)==HAL_OK) {
            ParseFrame(id,d,len);
            got=1;
        }
        ++flyTaskRxLoops;
        if (!got) Sleep(1);
    }
}
static void TofTask(void *arg)
{
    (void)arg;
    for (;;) {
        if (tofPollingMask) {
            for (uint8_t dev=25U; dev<=26U; ++dev) {
                if (!(tofPollingMask&(1U<<(dev-25U)))) continue;
                uint8_t d[8]={dev,12,0,0,0,0,0,0};
                if (CAN_SendFrameWait(0x0408,d,8,5)!=HAL_OK) rxFault=FB_ERR_TX;
                Sleep(2);
            }
        }
        Sleep(FB_TOF_POLL_MS);
    }
}

static int Guard(void)
{
    if (flyCommand==FB_CMD_STOP) { if (!homeActive && flyFlowResult!=1U) flyCommand=0; return 0; }
    if (rxFault!=FB_OK) { flyError=rxFault; flyState=FB_STATE_FAULT; return 0; }
    if (canRxFifo0OverrunCount!=fifo0Baseline || canRxFifo1OverrunCount!=fifo1Baseline) {
        uint32_t now=Now();
        uint32_t delta=(canRxFifo0OverrunCount-fifo0Baseline)+(canRxFifo1OverrunCount-fifo1Baseline);
        flyRxOverflowTotal+=delta;
        fifo0Baseline=canRxFifo0OverrunCount;
        fifo1Baseline=canRxFifo1OverrunCount;
        if (FB_RX_OVERFLOW_STRICT) {
            flyError=FB_ERR_RX_OVERFLOW; flyState=FB_STATE_FAULT; return 0;
        }
        if ((uint32_t)(now-rxOverflowWindowStartMs)>FB_RX_OVERFLOW_WINDOW_MS) {
            rxOverflowWindowStartMs=now; flyRxOverflowBurst=0;
        }
        flyRxOverflowBurst+=delta;
        if (FB_RX_OVERFLOW_FAULT_ENABLE && flyRxOverflowBurst>FB_RX_OVERFLOW_BURST_LIMIT) {
            flyError=FB_ERR_RX_OVERFLOW; flyState=FB_STATE_FAULT; return 0;
        }
    }
    return flyState!=FB_STATE_FAULT;
}
static int SendRaw(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (CAN_SendFrameWait(id,data,len,5)!=HAL_OK) { flyError=FB_ERR_TX; flyState=FB_STATE_FAULT; return 0; }
    return 1;
}
static int SendAck(uint16_t id, uint8_t dev, const uint8_t *data, uint8_t len)
{
    Ack a;
    while (xQueueReceive(ackQueue,&a,0)==pdTRUE) { }
    flyAckWaitId=id; flyAckWaitDevice=dev;
    if (!SendRaw(id,data,len)) return 0;
    uint32_t start=Now();
    do {
        if (flyCommand==FB_CMD_STOP) return 0;
        if (!Guard()) return 0;
        if (xQueueReceive(ackQueue,&a,pdMS_TO_TICKS(2))==pdTRUE && a.id==id && a.dev==dev) return 1;
    } while ((uint32_t)(Now()-start)<FB_ACK_TIMEOUT_MS);
    flyError=FB_ERR_ACK; flyState=FB_STATE_FAULT; return 0;
}
static int SetValue(uint16_t frame, uint8_t dev, int32_t value)
{
    uint8_t d[8]={0}; Put32(d,value); d[4]=dev;
    return SendAck(frame,dev,d,5);
}
static int PositionRelative(uint8_t dev, int32_t rel)
{
    uint8_t d[8]={0};
    Put32(d,rel); d[4]=dev; d[5]=0x00;
    return SendAck(0x0416,dev,d,6);
}
static int PositionAbsolute(uint8_t dev, int32_t pos)
{
    uint8_t d[8]={0};
    Put32(d,pos); d[4]=dev; d[5]=0x05;
    return SendAck(0x0416,dev,d,6);
}

static void StopAll(void)
{
    flyState=FB_STATE_BUSY;
    for (uint8_t id=1; id<=5; ++id) {
        uint8_t d[8]={0x4A,0,0,0,id};
        if (CAN_SendFrameWait(0x0413,d,5,5)!=HAL_OK) flyError=FB_ERR_TX;
        Sleep(2);
    }
    flyState=FB_STATE_OFF;
    flyHomed=0U; flyHomedMask=0U;
}
static void EnterIdle(void)
{
    StopAll();
    rxFault=FB_OK;
    flyError=FB_OK;
    flyRejected=FB_OK;
    fifo0Baseline=canRxFifo0OverrunCount;
    fifo1Baseline=canRxFifo1OverrunCount;
    flyRxOverflowBurst=0;
    rxOverflowWindowStartMs=Now();
    flyState=FB_STATE_OFF;
}
static void EnableOne(uint8_t dev)
{
    uint8_t d[8]={0};
    flyEnableDevice=dev; flyEnableStep=0; flyEnableResult=1;
    if (dev<1U || dev>5U || wheel001mm[dev]==0U) {
        flyEnableResult=6; flyRejected=FB_ERR_BAD_COMMAND; return;
    }
    flyEnableStep=1; memcpy(d,sn[dev-1U],7); d[7]=dev;
    if (!SendRaw(0x0313,d,8)) { flyEnableResult=4; return; }
    Sleep(20);
    flyEnableStep=2; memcpy(d,motorParam,6); d[3]=flyMotorCurLimit; d[6]=dev; d[7]=dev;
    if (!SendRaw(0x0316,d,8)) { flyEnableResult=4; return; }
    Sleep(20);
    flyEnableStep=3; d[0]=dev; d[1]=0x24;
    d[2]=(uint8_t)wheel001mm[dev]; d[3]=(uint8_t)(wheel001mm[dev]>>8);
    for (uint8_t n=0; n<3U; ++n) {
        if (!SendRaw(0x0408,d,4)) { flyEnableResult=4; return; }
        Sleep(10);
    }
    flyEnableStep=4; if (!SetValue(0x0414,dev,1000)) { flyEnableResult=3; return; }
    flyEnableStep=5; if (!SetValue(0x0415,dev,(int32_t)FB_TEST_SPEED)) { flyEnableResult=3; return; }
    flyEnableStep=6; d[0]=0x5A; d[1]=d[2]=d[3]=0; d[4]=dev;
    if (!SendAck(0x0413,dev,d,5)) { flyEnableResult=3; return; }
    flyEnabledMask|=1U<<(dev-1U);
    flyEnableStep=7; flyEnableResult=2; flyState=FB_STATE_READY; flyError=FB_OK;
}
static void EnableAll(void)
{
    flyEnableResult=1U;
    for (uint8_t dev=1U; dev<=5U; ++dev) {
        EnableOne(dev);
        if (flyCommand==FB_CMD_STOP) { flyEnableResult=5U; return; }
        if (flyEnableResult!=2U) return;
    }
    flyState=FB_STATE_READY;
}
static void ConfigureTofOne(uint8_t dev)
{
    uint8_t i=(uint8_t)(dev-25U), d[8];
    uint32_t beforeSeq, start;
    flyTofDevice=dev; flyTofStep=0; flyTofResult=1;
    if (dev<25U || dev>26U) { flyTofResult=6; flyRejected=FB_ERR_BAD_COMMAND; return; }
    tofPollingMask&=~(1U<<i);
    Sleep(FB_TOF_POLL_MS+5U);
    taskENTER_CRITICAL(); memset((void *)&tof[i],0,sizeof tof[i]); taskEXIT_CRITICAL();
    flyTofStep=1; memcpy(d,sn[5U+i],7); d[7]=dev;
    if (!SendRaw(0x0313,d,8)) { flyTofResult=4; return; }
    Sleep(20);
    flyTofStep=2;
    {
        const uint8_t config[8]={0x11,0x22,0,0xFF,0,0x05,0,dev};
        if (!SendRaw(0x0316,config,8)) { flyTofResult=4; return; }
    }
    Sleep(20);
    flyTofStep=3;
    {
        const uint8_t startFrame[8]={dev,10,1,0x11,0x22,0,0,0};
        if (!SendRaw(0x0408,startFrame,8)) { flyTofResult=4; return; }
    }
    taskENTER_CRITICAL(); beforeSeq=tof[i].rxSeq; taskEXIT_CRITICAL();
    tofPollingMask|=1U<<i; flyTofStep=4; start=Now();
    while (1) {
        TofFeedback f=ReadTof(i);
        if (flyCommand==FB_CMD_STOP) { flyTofResult=5; return; }
        if (f.rxSeq!=beforeSeq && (uint32_t)(Now()-f.rxMs)<=FB_TOF_STALE_MS) break;
        if ((uint32_t)(Now()-start)>=FB_TOF_INIT_TIMEOUT_MS) { flyTofResult=3; return; }
        Sleep(5);
    }
    flyTofStep=5; flyTofResult=2; flyState=FB_STATE_READY; flyError=FB_OK;
}
static void ConfigureTofAll(void)
{
    flyTofResult=1U;
    ConfigureTofOne(25U);
    if (flyCommand==FB_CMD_STOP) { flyTofResult=5U; return; }
    if (flyTofResult!=2U) return;
    ConfigureTofOne(26U);
    if (flyCommand==FB_CMD_STOP) { flyTofResult=5U; return; }
    if (flyTofResult!=2U) return;
    flyState=FB_STATE_READY;
}
/* 查询电机绝对位置（0x0408 Data[1]=0x08 → 0x0409），单次100ms内无响应返回0。
 * 回零全程依赖这个位置，掉一帧就判整轴失败会让初始化"做一个动作就停住"，
 * 所以失败自动重发，最多3次。 */
static int QueryPos(uint8_t dev, int32_t *pos)
{
    uint8_t d[8]={0};
    uint32_t before, start;
    d[0]=dev; d[1]=0x08;
    for (uint8_t attempt=0U; attempt<3U; ++attempt) {
        taskENTER_CRITICAL(); before=motor[dev].posSeq; taskEXIT_CRITICAL();
        if (CAN_SendFrameWait(0x0408,d,2,5)!=HAL_OK) return 0;
        start=Now();
        while (1) {
            uint32_t s, ms; int32_t p;
            taskENTER_CRITICAL(); s=motor[dev].posSeq; p=motor[dev].pos; ms=motor[dev].posMs; taskEXIT_CRITICAL();
            /* 只认本次请求之后收到的应答：推箱时有轮询请求在飞，别拿上一帧位置当现位置 */
            if (s!=before && (int32_t)(ms-start)>=0) { *pos=p; return 1; }
            if ((uint32_t)(Now()-start)>=100U) break;
            Sleep(2);
        }
    }
    return 0;
}
/* 只改 0x0316 的 Byte3（顺/逆时针最大电流各4bit，0x33=各3档、0x55=各5档、0xAA=各10档），
 * 其余位保持 motorParam 原值。0x0316 无 ACK。 */
static void SetMotorCurrentLimit(uint8_t dev, uint8_t limit)
{
    uint8_t d[8];
    memcpy(d,motorParam,6);
    d[3]=limit; d[6]=dev; d[7]=dev;
    /* 0x0316无ACK，发两遍降低丢帧概率：这帧决定堵转电流，丢了就可能把电源拉掉 */
    (void)CAN_SendFrameWait(0x0316,d,8,5);
    Sleep(5);
    (void)CAN_SendFrameWait(0x0316,d,8,5);
}
/* 回零统一出口：失败时先停住本轴，再恢复正常电流档，避免带着初始化电流档或还在运动就退出。 */
static uint32_t HomeFinish(uint8_t dev, uint32_t result)
{
    if (result!=2U) {
        flyState=FB_STATE_BUSY; flyError=FB_OK; rxFault=FB_OK;
        (void)PositionRelative(dev,0);
    }
    flyMotorCurLimit=FB_HOME_LIMIT_RUN;
    SetMotorCurrentLimit(dev,FB_HOME_LIMIT_RUN);
    return result;
}
/* 单轴命令累积成功位；三轴齐了才置 flyHomed，不改变原推拉箱调度。 */
static void RefreshHomed(void)
{
    flyHomed=((flyHomedMask&FB_HOME_REQUIRED_MASK)==FB_HOME_REQUIRED_MASK) ? 1U : 0U;
}
/* 等一段相对位移走完（轻退验证 / 退初始位用）：1=到位 0=超时/被停止/没起步。
 * 不能只认"任意一帧 remaining≈0"：驱动器是周期上报 remaining 的，命令下发前它还在
 * 报上一段的 remaining=0，那条迟到帧会被当成"已到位"，于是这次动作刚发出去就被判
 * 完成、整轴初始化当场失败——现象就是做完一个动作不再做下一个。
 * 所以 seq 必须由调用方在命令被应答之后取，并且先确认真的动过，再等到位；
 * 驱动器不报 remaining 时用位置反馈停稳兜底，不会白等满超时。 */
static int HomeWaitDone(uint8_t dev, uint32_t seq, uint32_t timeoutMs)
{
    uint32_t start=Now(), posPollMs=start, posSeq0, lastChangeMs=start, stallMs;
    int32_t lastPos;
    uint8_t sawPos=0, sawMotion=0;
    stallMs=flyHomeStallMs ? flyHomeStallMs : FB_HOME_STALL_MS;
    taskENTER_CRITICAL();
    posSeq0=motor[dev].posSeq; lastPos=motor[dev].pos;
    taskEXIT_CRITICAL();
    while (1) {
        MotorFeedback m;
        uint32_t now=Now();
        taskENTER_CRITICAL(); m=motor[dev]; taskEXIT_CRITICAL();
        if (flyCommand==FB_CMD_STOP) return 0;
        if (!Guard()) return 0;
        if ((uint32_t)(now-posPollMs)>=FB_HOME_POLL_MS) {
            uint8_t d[8]={dev,0x08U,0,0,0,0,0,0};
            if (CAN_SendFrameWait(0x0408,d,2,5)!=HAL_OK) return 0;
            posPollMs=now;
        }
        if (m.posSeq!=posSeq0) {
            /* 直接和进入本函数时的位置比：动作很短时第一帧应答就可能已经是终点，
             * 不能把"第一帧"当成基线，否则会误判成没动过。 */
            if (Abs32(m.pos-lastPos)>2) {
                lastPos=m.pos; lastChangeMs=now; sawPos=1U; sawMotion=1U;
            }
        }
        if (m.remSeq!=seq) {
            if (Abs32(m.remaining)>FB_TEST_TOLERANCE) sawMotion=1U;
            else if (sawMotion) return 1; /* 这次命令真的动过，剩余位移又清零=到位 */
        }
        /* 位置反馈停稳也算到位：驱动器不报 remaining、或单位对不上时不至于等满超时。
         * 只认位置判据，避免一帧来路不明的 remaining 就把还没起步的动作判成完成。 */
        if (sawPos && (uint32_t)(now-lastChangeMs)>=stallMs) return 1;
        /* 起步只认位置反馈：驱动器报了 remaining 但轴根本没动，同样是故障，
         * 也要提前退出，不能等满整段超时。 */
        if (!sawPos && (uint32_t)(now-start)>=FB_HOME_START_TIMEOUT_MS) return 0;
        if ((uint32_t)(now-start)>=timeoutMs) return 0;
        Sleep(5);
    }
}
/* 回零过程中要反复重新使能（撞死点后驱动器可能自保护下电）：失败重试一次，
 * 掉一帧就判整轴失败会让初始化看起来"做完一个动作就停住"。 */
static void HomeEnable(uint8_t dev)
{
    for (uint8_t n=0U; n<2U; ++n) {
        EnableOne(dev);
        if ((flyEnabledMask&(1U<<(dev-1U))) || flyCommand==FB_CMD_STOP) return;
        if (n==0U) { flyState=FB_STATE_BUSY; flyError=FB_OK; rxFault=FB_OK; }
        Sleep(50);
    }
}
/* 切到指定电流档并重新使能：0x0316 要随使能一起下发才可靠（同 HomeOneAxis 开头）。 */
static void HomeSetLimitAndEnable(uint8_t dev, uint8_t limit)
{
    if (flyCommand==FB_CMD_STOP) return;
    flyState=FB_STATE_BUSY; flyError=FB_OK; rxFault=FB_OK;
    flyMotorCurLimit=limit;
    flyEnabledMask&=~(1U<<(dev-1U));
    HomeEnable(dev);
    if (flyEnabledMask&(1U<<(dev-1U))) SetMotorCurrentLimit(dev,limit);
    Sleep(20);
}
/* 反向轻退 probe 距离，确认这台在**当前电流档**下真的能动。
 * 返回 1=确实动了 0=没动/被停止/通信失败（调用方决定是否换电流档重试）。 */
static int HomeProbeTry(uint8_t dev, int32_t dir, int32_t probe)
{
    int32_t P0=0, P1=0;
    uint32_t seq;
    if (!SetValue(0x0414,dev,flyHomeAccel) || !SetValue(0x0415,dev,flyHomeSpeed)) return 0;
    if (!QueryPos(dev,&P0)) return 0;
    if (!PositionRelative(dev,(int32_t)(-dir)*probe*pullSign[dev])) return 0;
    taskENTER_CRITICAL(); seq=motor[dev].remSeq; taskEXIT_CRITICAL();
    if (!HomeWaitDone(dev,seq,3000U)) return 0;
    if (!QueryPos(dev,&P1)) return 0;
    return (Abs32(P1-P0)>=probe/2) ? 1 : 0;
}
/* 设置类帧（0x0414/0x0415，幂等）丢一个 ACK 就重发一次：
 * 单帧 ACK 超时不该把整轴回零判死（现场就是"step=2 直接失败"）。 */
static int SetValueRetry(uint16_t frame, uint8_t dev, int32_t value)
{
    for (uint8_t n=0U; n<3U; ++n) {
        if (SetValue(frame,dev,value)) return 1;
        if (flyCommand==FB_CMD_STOP) return 0;
        /* SendAck 超时会把状态置 FAULT，不清掉重发也会被 Guard 挡住 */
        flyState=FB_STATE_BUSY; flyError=FB_OK; rxFault=FB_OK;
        Sleep(20);
    }
    flyError=FB_ERR_ACK;
    return 0;
}
/* 单轴回零：初始化电流档(默认0x55)低速撞死点 → 停车 → 强制重新使能 → 读死点位置 P_ref
 * → 按 flyHomeBackoff 退到想要的初始位置 P_init。
 * 死点判据三选一：力矩超阈值 / 位置反馈停滞 / 错误帧0x0104或0x0102。
 * 行程走完仍没撞到死点、或 2.5s 内没起步都立即判失败，不等满 flyHomeTimeoutMs。
 * 返回 2=成功 3=失败 4=停止打断 5=未使能 6=参数不支持。 */
static uint32_t HomeOneAxis(uint8_t dev)
{
    int32_t dir;
    int32_t dist, P_ref=0, P_init=0, lastPos=0, P_start=0;
    uint32_t start, posPollMs, posSeq0, remSeq0, torSeq0, lastChangeMs, seq;
    uint32_t errMs0, hits=0, hitsThr, stallMs, timeoutMs;
    uint8_t gotPos=0, gotRem=0, sawMove=0, reason=0;
    if (dev<1U || dev>5U) return 6U;
    dir=homeDir[dev];
    if (dir==0 || flyHomeSpeed<=0 || flyHomeAccel<=0) return 6U;
    flyHomeAxis=dev; flyHomeStep=1U; flyHomeStopReason=0U;
    /* 强制重新使能：让初始化电流档随使能一起生效，不依赖驱动器是否即时接受0x0316 */
    flyMotorCurLimit=flyHomeCurLimit;
    flyEnabledMask&=~(1U<<(dev-1U));
    HomeEnable(dev);
    if (!(flyEnabledMask&(1U<<(dev-1U))))
        return HomeFinish(dev,(flyCommand==FB_CMD_STOP)?4U:5U);
    flyHomeStep=2U;
    SetMotorCurrentLimit(dev,flyHomeCurLimit); /* AA→55：堵转峰值不会把电源拉掉 */
    Sleep(20);
    if (!SetValueRetry(0x0414,dev,flyHomeAccel) || !SetValueRetry(0x0415,dev,flyHomeSpeed)) {
        flyHomeStopReason=6U; /* 6=驱动器不回设置帧(0x0414/0x0415)，连试3次都不应答 */
        return HomeFinish(dev,3U);
    }
    dist=Abs32(flyHomeDist[dev]);
    if (dist<=0) dist=50000;
    /* 搜索行程严格按 flyHomeDist[轴] 下发，不做任何自动放大：
     * 这个值是现场标定好的，固件擅自加长会让轴撞过死点。
     * 如果走完这段还没撞到死点，下面按 flyHomeStopReason=4 立即报失败。 */
    /* 先反向轻退一小步，确认驱动/初始化电流档真的能让轴动；否则"位置一直不动"
     * 会把"电流太小/驱动没使能"误判成死点，初始化到错误位置。
     * 初始化档退不动时（最常见：上一次停在死点被压住）换 AA 再退一次，退出来后再打回初始化档，
     * 避免"停在死点上就再也出不来"（现场：flyHomeStep=2 直接失败、轴一动不动）。 */
    if (flyHomeProbeDist>0) {
        int32_t probe=Abs32(flyHomeProbeDist);
        if (!HomeProbeTry(dev,dir,probe)) {
            if (flyCommand==FB_CMD_STOP) return HomeFinish(dev,4U);
            flyHomeStopReason=7U; /* 7=初始化电流档反向轻退不动 */
            ++flyHomeRescueCount;
            HomeSetLimitAndEnable(dev,FB_HOME_LIMIT_RUN); /* 先用全电流把自己从死点里退出来 */
            if (!(flyEnabledMask&(1U<<(dev-1U)))) return HomeFinish(dev,5U);
            if (!HomeProbeTry(dev,dir,probe)) {
                if (flyCommand==FB_CMD_STOP) return HomeFinish(dev,4U);
                flyHomeStopReason=8U; /* 8=换全电流也退不动：机械压死/方向不对 */
                return HomeFinish(dev,3U);
            }
            /* 退出来了：恢复初始化电流档继续找死点（找死点仍按低电流档跑，不改判据） */
            HomeSetLimitAndEnable(dev,flyHomeCurLimit);
            if (!(flyEnabledMask&(1U<<(dev-1U)))) return HomeFinish(dev,5U);
            if (!SetValueRetry(0x0414,dev,flyHomeAccel) || !SetValueRetry(0x0415,dev,flyHomeSpeed)) {
                flyHomeStopReason=6U;
                return HomeFinish(dev,3U);
            }
            flyHomeStopReason=0U;
        }
        dist+=probe;
    }
    hitsThr=flyHomeTorqueHits ? flyHomeTorqueHits : 1U;
    stallMs=flyHomeStallMs ? flyHomeStallMs : FB_HOME_STALL_MS;
    timeoutMs=flyHomeTimeoutMs ? flyHomeTimeoutMs : FB_HOME_TIMEOUT_MS;
    flyHomeTorquePeak[dev]=0;
    taskENTER_CRITICAL();
    posSeq0=motor[dev].posSeq; remSeq0=motor[dev].remSeq;
    torSeq0=motor[dev].torqueSeq; lastPos=motor[dev].pos;
    taskEXIT_CRITICAL();
    errMs0=flyLastMotorErrorMs;
    flyHomeStep=3U;
    flyHomeSearchSpan[dev]=0;
    P_start=lastPos;
    if (!PositionRelative(dev,(int32_t)dir*dist*pullSign[dev]))
        return HomeFinish(dev,(flyCommand==FB_CMD_STOP)?4U:3U);
    start=Now(); lastChangeMs=start; posPollMs=start;
    while (1) {
        MotorFeedback m;
        uint32_t now=Now();
        taskENTER_CRITICAL(); m=motor[dev]; taskEXIT_CRITICAL();
        if (flyCommand==FB_CMD_STOP) return HomeFinish(dev,4U);
        if (!Guard()) return HomeFinish(dev,3U);
        /* 位置轮询：0x0408 Data[1]=0x08 → 0x0409 返回绝对位置 */
        if ((uint32_t)(now-posPollMs)>=FB_HOME_POLL_MS) {
            uint8_t d[8]={dev,0x08U,0,0,0,0,0,0};
            if (CAN_SendFrameWait(0x0408,d,2,5)!=HAL_OK) return HomeFinish(dev,3U);
            posPollMs=now;
        }
        /* 位置停滞判据：必须有"动过"或"还有剩余位移"的证据，避免把命令没被接受误判成死点 */
        if (m.posSeq!=posSeq0) {
            if (!gotPos) { gotPos=1U; lastPos=m.pos; lastChangeMs=now; }
            else if (Abs32(m.pos-lastPos)>2) { lastPos=m.pos; lastChangeMs=now; sawMove=1U; }
        }
        if (m.remSeq!=remSeq0) gotRem=1U;
        /* 起步超时：一直没有有效位移，别再等满整段超时（现场看起来就是卡住） */
        if (!sawMove && !(gotRem && Abs32(m.remaining)>FB_TEST_TOLERANCE) &&
            (uint32_t)(now-start)>=FB_HOME_START_TIMEOUT_MS) {
            reason=FB_HOME_STOP_NO_START; break;
        }
        {
            uint8_t moved=(uint8_t)(sawMove || gotRem);
            uint8_t pending=gotRem ? (uint8_t)(Abs32(m.remaining)>FB_TEST_TOLERANCE) : 1U;
            if (gotPos && moved && pending &&
                (uint32_t)(now-start)>=FB_HOME_START_GRACE_MS &&
                (uint32_t)(now-lastChangeMs)>=stallMs) {
                reason=2U; break;
            }
            /* 相对位移被驱动器走完（remaining 清零）却已经停住=没撞到死点：
             * 立即判失败，不要白等到 30s 超时（现象：轴停住不动，固件发呆）。 */
            if (sawMove && gotRem && Abs32(m.remaining)<=FB_TEST_TOLERANCE &&
                (uint32_t)(now-start)>=FB_HOME_START_GRACE_MS &&
                (uint32_t)(now-lastChangeMs)>=stallMs) {
                reason=FB_HOME_STOP_NO_DEADPOINT; break;
            }
        }
        /* 力矩判据：连续 hitsThr 帧 |torque|>=flyHomeTorqueThr；阈值0=关闭 */
        if (m.torqueSeq!=torSeq0) {
            int32_t tq;
            torSeq0=m.torqueSeq;
            tq=Abs32((int32_t)m.torque);
            if (tq>flyHomeTorquePeak[dev]) flyHomeTorquePeak[dev]=tq;
            if (flyHomeTorqueThr>0 && (uint32_t)(now-start)>=FB_HOME_TORQUE_IGNORE_MS) {
                if (tq>=flyHomeTorqueThr) {
                    if (++hits>=hitsThr) { reason=1U; break; }
                } else hits=0U;
            }
        }
        /* 错误帧判据：只认本次回零开始后的新帧；0x0104堵转/0x0102未达目标=到死点 */
        if (flyLastMotorErrorMs!=errMs0) {
            errMs0=flyLastMotorErrorMs;
            if (flyLastMotorError==0x0104U || flyLastMotorError==0x0102U) { reason=3U; break; }
            return HomeFinish(dev,3U);
        }
        if ((uint32_t)(now-start)>=timeoutMs) return HomeFinish(dev,3U);
        Sleep(5);
    }
    flyHomeStopReason=reason;
    flyHomeSearchSpan[dev]=Abs32(lastPos-P_start); /* 找死点实际走了多远，判断 reason=2 是不是假死点 */
    /* 不是撞到死点（行程走完/没起步）就不算初始化成功：停住本轴直接返回 */
    if (reason==FB_HOME_STOP_NO_DEADPOINT || reason==FB_HOME_STOP_NO_START)
        return HomeFinish(dev,3U);
    /* 停住：目标同步当前位置，平滑减速并保持力矩；不用0x4A释放，避免掉负载 */
    flyHomeStep=4U;
    (void)PositionRelative(dev,0);
    Sleep(FB_HOME_SETTLE_MS);
    if (flyCommand==FB_CMD_STOP) return HomeFinish(dev,4U);
    /* 堵转时上一条停止命令可能没有ACK：先清本地故障态，否则重新使能会被Guard挡住 */
    flyState=FB_STATE_BUSY; flyError=FB_OK; rxFault=FB_OK;
    /* 顶到死点可能触发驱动器自保护下电：无条件重新使能后再读位置 */
    flyMotorCurLimit=flyHomeCurLimit;
    flyEnabledMask&=~(1U<<(dev-1U));
    HomeEnable(dev);
    if (!(flyEnabledMask&(1U<<(dev-1U))))
        return HomeFinish(dev,(flyCommand==FB_CMD_STOP)?4U:5U);
    SetMotorCurrentLimit(dev,flyHomeCurLimit);
    Sleep(20);
    if (!QueryPos(dev,&P_ref)) return HomeFinish(dev,3U);
    flyHomeDeadPos[dev]=P_ref;
    flyHomeStep=5U;
    P_init=P_ref;
    flyHomeBackoffUsed[dev]=0;
    flyHomeInitSpan[dev]=0;
    if (flyHomeBackoff[dev]!=0) {
        int32_t back=Abs32(flyHomeBackoff[dev]);
        flyHomeBackoffUsed[dev]=back;
        /* 目标位置（旧坐标系）算好备用：初始化档退不动时用它算"还差多少" */
        int32_t target=P_ref+(int32_t)(-dir)*back*pullSign[dev];
        if (!SetValue(0x0414,dev,flyHomeAccel) || !SetValue(0x0415,dev,flyHomeSpeed) ||
            !PositionRelative(dev,(int32_t)(-dir)*back*pullSign[dev]))
            return HomeFinish(dev,(flyCommand==FB_CMD_STOP)?4U:3U);
        taskENTER_CRITICAL(); seq=motor[dev].remSeq; taskEXIT_CRITICAL();
        if (!HomeWaitDone(dev,seq,timeoutMs)) {
            int32_t cur=0, remain;
            if (flyCommand==FB_CMD_STOP) return HomeFinish(dev,4U);
            /* 初始化档退不动（顶死在死点上）：换 AA 再退。剩余量按"目标-当前位置"算，
             * 且在重新使能前取，重新使能即使复位位置计数器也不影响这个相对量。 */
            if (!QueryPos(dev,&cur)) return HomeFinish(dev,3U);
            remain=target-cur;
            if (remain==0) remain=(int32_t)(-dir)*back*pullSign[dev];
            ++flyHomeRescueCount;
            HomeSetLimitAndEnable(dev,FB_HOME_LIMIT_RUN);
            if (!(flyEnabledMask&(1U<<(dev-1U)))) return HomeFinish(dev,5U);
            if (!SetValue(0x0414,dev,flyHomeAccel) || !SetValue(0x0415,dev,flyHomeSpeed) ||
                !PositionRelative(dev,remain))
                return HomeFinish(dev,(flyCommand==FB_CMD_STOP)?4U:3U);
            taskENTER_CRITICAL(); seq=motor[dev].remSeq; taskEXIT_CRITICAL();
            if (!HomeWaitDone(dev,seq,timeoutMs))
                return HomeFinish(dev,(flyCommand==FB_CMD_STOP)?4U:3U);
        }
        if (!QueryPos(dev,&P_init)) return HomeFinish(dev,3U);
        flyHomeInitSpan[dev]=Abs32(P_init-P_ref);
        /* 退不出去=电流太小/轴卡住，不能假装初始化成功（backoff=0时无法校验） */
        if (Abs32(P_init-P_ref)<back/2) return HomeFinish(dev,3U);
    }
    flyHomeInitPos[dev]=P_init;
    /* 恢复AA并重新使能：保证任务9用正常电流档。若驱动器在使能时复位位置计数器，
     * 使能后重读一次，保证 flyHomeInitPos 和后续位置查询同一坐标系。
     * 必须每轴做完就恢复：不能等到三轴全做完，否则已做完的轴长时间停在初始化电流档，
     * 带负载的轴（尤其钩爪05）会被自重压到行程端，轮到自己时就走不动了。 */
    flyMotorCurLimit=FB_HOME_LIMIT_RUN;
    flyEnabledMask&=~(1U<<(dev-1U));
    HomeEnable(dev);
    if (!(flyEnabledMask&(1U<<(dev-1U))))
        return HomeFinish(dev,(flyCommand==FB_CMD_STOP)?4U:5U);
    if (!QueryPos(dev,&P_init)) return HomeFinish(dev,3U);
    flyHomeInitPos[dev]=P_init;
    flyHomeDeadPos[dev]=P_init-(int32_t)(-dir)*Abs32(flyHomeBackoff[dev])*pullSign[dev];

    flyHomeStep=6U;
    return 2U;
}
/* 初始化 mask 里指定、且有死点的轴（homeDir≠0）。
 * 每个成功的轴置 flyHomedMask 对应位；01/04/05 三轴齐了才置 flyHomed=1。
 * 电流档按轴处理：轮到这个轴才降初始化电流档，这轴做完 HomeOneAxis 立刻恢复 0xAA，
 * 不会让还没做的轴长时间停在初始化档（带负载的轴会被自重压到行程端走不动）。 */
static void RunHomingMaskImpl(uint32_t mask)
{
    uint8_t dev, homedCount=0U;
    uint32_t r;
    flyHomedMask&=~mask;
    RefreshHomed();
    flyHomeResult=1U; flyHomeAxis=0U; flyHomeStep=0U;
    flyState=FB_STATE_BUSY; flyError=FB_OK; rxFault=FB_OK;
    for (dev=1U; dev<=5U; ++dev) {
        if (!(mask&(1U<<(dev-1U)))) continue;
        if (homeDir[dev]==0) continue;
        r=HomeOneAxis(dev);
        if (r==2U) {
            ++homedCount;
            flyHomedMask|=1U<<(dev-1U);
            RefreshHomed();
            continue;
        }
        flyHomeResult=(r==4U)?4U:((r==5U)?5U:3U);
        RefreshHomed();
        if (r==4U) StopAll();
        else { flyState=FB_STATE_FAULT; flyError=FB_ERR_TIMEOUT; }
        return;
    }
    if (homedCount==0U) { /* 掩码里没有可用轴：不假装初始化成功 */
        flyHomeResult=6U; flyHomeStep=0U;
        RefreshHomed();
        flyState=FB_STATE_FAULT; flyError=FB_ERR_BAD_COMMAND;
        return;
    }
    RefreshHomed();
    flyHomeResult=2U; flyHomeAxis=0U; flyHomeStep=6U;
    flyState=FB_STATE_READY; flyError=FB_OK;
}
/* 命令3：按 flyHomeAxisMask 做；命令1/2/4：只做单轴。 */
static void RunHomingMask(uint32_t mask)
{
    homeActive=1U;
    RunHomingMaskImpl(mask);
    homeActive=0U;
    if (flyCommand==FB_CMD_STOP) {
        flyCommand=0U;
        StopAll();
        flyHomeResult=4U;
    }
}
static void RunHoming(void) { RunHomingMask(flyHomeAxisMask); }

static void TestOneMotor(uint8_t dev, int8_t direction, int32_t distance, int32_t speed, int32_t accel)
{
    uint32_t start, seq;
    int32_t rel=(direction>=0) ? distance : -distance;
    flyTestDevice=dev; flyTestStep=0; flyTestResult=1;
    if (dev<1U || dev>5U) { flyTestResult=7; flyRejected=FB_ERR_BAD_COMMAND; return; }
    if (!(flyEnabledMask&(1U<<(dev-1U)))) {
        EnableOne(dev);
        if (!(flyEnabledMask&(1U<<(dev-1U)))) { flyTestResult=6; flyRejected=FB_ERR_BAD_COMMAND; return; }
    }
    flyState=FB_STATE_BUSY;
    flyTestStep=1;
    flyTestDist=rel*pullSign[dev];
    if (!SetValue(0x0414,dev,accel) || !SetValue(0x0415,dev,speed) ||
        !PositionRelative(dev,rel*pullSign[dev])) {
        flyTestResult=3; flyState=FB_STATE_FAULT; return;
    }
    taskENTER_CRITICAL(); seq=motor[dev].remSeq; taskEXIT_CRITICAL();
    flyTestStep=2; start=Now();
    while (1) {
        MotorFeedback m;
        taskENTER_CRITICAL(); m=motor[dev]; taskEXIT_CRITICAL();
        if (flyCommand==FB_CMD_STOP) { flyTestResult=5; flyState=FB_STATE_OFF; return; }
        if (!Guard()) { flyTestResult=3; return; }
        if (m.remSeq!=seq && Abs32(m.remaining)<=FB_TEST_TOLERANCE) break;
        if ((uint32_t)(Now()-start)>=FB_TEST_TIMEOUT_MS) {
            flyTestResult=4; flyError=FB_ERR_TIMEOUT; flyState=FB_STATE_FAULT; return;
        }
        Sleep(5);
    }
    flyTestStep=3; flyTestResult=2; flyState=FB_STATE_READY; flyError=FB_OK;
}
static void TestBeltPair(int8_t direction)
{
    uint32_t seq2, seq3, start;
    int32_t rel=(direction>=0) ? flyBeltDist : -flyBeltDist;
    flyTestDevice=23U; flyTestStep=0; flyTestResult=1;
    if (!(flyEnabledMask&(1U<<1))) EnableOne(2U);
    if (!(flyEnabledMask&(1U<<2))) EnableOne(3U);
    if (!(flyEnabledMask&((1U<<1)|(1U<<2)))) { flyTestResult=6; flyRejected=FB_ERR_BAD_COMMAND; return; }
    flyState=FB_STATE_BUSY;
    flyTestStep=1;
    flyTestDist=rel*pullSign[2];
    if (!SetValue(0x0414,2U,flyBeltAccel) || !SetValue(0x0415,2U,flyBeltSpeed) ||
        !SetValue(0x0414,3U,flyBeltAccel) || !SetValue(0x0415,3U,flyBeltSpeed) ||
        !PositionRelative(2U,rel*pullSign[2]) || !PositionRelative(3U,rel*pullSign[3])) {
        flyTestResult=3; flyState=FB_STATE_FAULT; return;
    }
    taskENTER_CRITICAL(); seq2=motor[2].remSeq; seq3=motor[3].remSeq; taskEXIT_CRITICAL();
    flyTestStep=2; start=Now();
    while (1) {
        MotorFeedback m2, m3;
        taskENTER_CRITICAL(); m2=motor[2]; m3=motor[3]; taskEXIT_CRITICAL();
        if (flyCommand==FB_CMD_STOP) { flyTestResult=5; flyState=FB_STATE_OFF; return; }
        if (!Guard()) { flyTestResult=3; return; }
        if (m2.remSeq!=seq2 && Abs32(m2.remaining)<=FB_TEST_TOLERANCE &&
            m3.remSeq!=seq3 && Abs32(m3.remaining)<=FB_TEST_TOLERANCE) break;
        if ((uint32_t)(Now()-start)>=FB_TEST_TIMEOUT_MS) {
            flyTestResult=4; flyError=FB_ERR_TIMEOUT; flyState=FB_STATE_FAULT; return;
        }
        Sleep(5);
    }
    flyTestStep=3; flyTestResult=2; flyState=FB_STATE_READY; flyError=FB_OK;
}
static int TestBeltPairDistance(int8_t direction, int32_t distance)
{
    uint32_t seq2, seq3, start;
    int32_t rel=(direction>=0) ? distance : -distance;
    flyTestDevice=23U; flyTestStep=0; flyTestResult=1;
    if (!(flyEnabledMask&(1U<<1))) EnableOne(2U);
    if (!(flyEnabledMask&(1U<<2))) EnableOne(3U);
    if (!(flyEnabledMask&((1U<<1)|(1U<<2)))) { flyTestResult=6; flyRejected=FB_ERR_BAD_COMMAND; return 0; }
    flyState=FB_STATE_BUSY;
    flyTestStep=1;
    flyTestDist=rel*pullSign[2];
    if (!SetValue(0x0414,2U,flyBeltAccel) || !SetValue(0x0415,2U,flyBeltSpeed) ||
        !SetValue(0x0414,3U,flyBeltAccel) || !SetValue(0x0415,3U,flyBeltSpeed) ||
        !PositionRelative(2U,rel*pullSign[2]) || !PositionRelative(3U,rel*pullSign[3])) {
        flyTestResult=3; flyState=FB_STATE_FAULT; return 0;
    }
    taskENTER_CRITICAL(); seq2=motor[2].remSeq; seq3=motor[3].remSeq; taskEXIT_CRITICAL();
    flyTestStep=2; start=Now();
    while (1) {
        MotorFeedback m2, m3;
        taskENTER_CRITICAL(); m2=motor[2]; m3=motor[3]; taskEXIT_CRITICAL();
        if (flyCommand==FB_CMD_STOP) { flyTestResult=5; flyState=FB_STATE_OFF; return 0; }
        if (!Guard()) { flyTestResult=3; return 0; }
        if (m2.remSeq!=seq2 && Abs32(m2.remaining)<=FB_TEST_TOLERANCE &&
            m3.remSeq!=seq3 && Abs32(m3.remaining)<=FB_TEST_TOLERANCE) break;
        if ((uint32_t)(Now()-start)>=FB_TEST_TIMEOUT_MS) {
            flyTestResult=4; flyError=FB_ERR_TIMEOUT; flyState=FB_STATE_FAULT; return 0;
        }
        Sleep(5);
    }
    flyTestStep=3; flyTestResult=2; flyState=FB_STATE_READY; flyError=FB_OK;
    return 1;
}
static int PullBackWithBelt(void)
{
    uint32_t seq1, seq2, seq3, start;
    int32_t frontRel=flyFrontGoDist;
    int32_t beltRel=-flyBeltPullDist;
    if (frontRel<=FB_TEST_TOLERANCE) {
        flyTestResult=7; flyRejected=FB_ERR_BAD_COMMAND; return 0;
    }
    if (flyBeltPullDist<=FB_TEST_TOLERANCE) {
        flyTestResult=7; flyRejected=FB_ERR_BAD_COMMAND; return 0;
    }
    if (!(flyEnabledMask&(1U<<0))) EnableOne(1U);
    if (!(flyEnabledMask&(1U<<1))) EnableOne(2U);
    if (!(flyEnabledMask&(1U<<2))) EnableOne(3U);
    if (!(flyEnabledMask&((1U<<0)|(1U<<1)|(1U<<2)))) {
        flyTestResult=6; flyRejected=FB_ERR_BAD_COMMAND; return 0;
    }
    flyTestDevice=123U; flyTestStep=0; flyTestResult=1;
    flyState=FB_STATE_BUSY;
    flyTestStep=1;
    if (!SetValue(0x0414,1U,flyFrontAccel) || !SetValue(0x0415,1U,flyFrontSpeed) ||
        !SetValue(0x0414,2U,flyBeltAccel) || !SetValue(0x0415,2U,flyBeltSpeed) ||
        !SetValue(0x0414,3U,flyBeltAccel) || !SetValue(0x0415,3U,flyBeltSpeed) ||
        !PositionRelative(1U,frontRel*pullSign[1]) ||
        !PositionRelative(2U,beltRel*pullSign[2]) ||
        !PositionRelative(3U,beltRel*pullSign[3])) {
        flyTestResult=3; flyState=FB_STATE_FAULT; return 0;
    }
    taskENTER_CRITICAL(); seq1=motor[1].remSeq; seq2=motor[2].remSeq; seq3=motor[3].remSeq; taskEXIT_CRITICAL();
    flyTestStep=2; start=Now();
    while (1) {
        MotorFeedback m1, m2, m3;
        taskENTER_CRITICAL(); m1=motor[1]; m2=motor[2]; m3=motor[3]; taskEXIT_CRITICAL();
        if (flyCommand==FB_CMD_STOP) { flyTestResult=5; flyState=FB_STATE_OFF; return 0; }
        if (!Guard()) { flyTestResult=3; return 0; }
        if (m1.remSeq!=seq1 && Abs32(m1.remaining)<=FB_TEST_TOLERANCE &&
            m2.remSeq!=seq2 && Abs32(m2.remaining)<=FB_TEST_TOLERANCE &&
            m3.remSeq!=seq3 && Abs32(m3.remaining)<=FB_TEST_TOLERANCE) break;
        if ((uint32_t)(Now()-start)>=FB_TEST_TIMEOUT_MS) {
            flyTestResult=4; flyError=FB_ERR_TIMEOUT; flyState=FB_STATE_FAULT; return 0;
        }
        Sleep(5);
    }
    flyTestStep=3; flyTestResult=2; flyState=FB_STATE_READY; flyError=FB_OK;
    return 1;
}
static int PushForwardWithBelt(void)
{
    uint32_t seq1, seq2, seq3, start;
    int32_t frontRel=-flyFrontGoDist;
    int32_t beltRel=flyBeltPullDist;
    if (flyFrontGoDist<=FB_TEST_TOLERANCE || flyBeltPullDist<=FB_TEST_TOLERANCE) {
        flyTestResult=7; flyRejected=FB_ERR_BAD_COMMAND; return 0;
    }
    if (!(flyEnabledMask&(1U<<0))) EnableOne(1U);
    if (!(flyEnabledMask&(1U<<1))) EnableOne(2U);
    if (!(flyEnabledMask&(1U<<2))) EnableOne(3U);
    if (!(flyEnabledMask&((1U<<0)|(1U<<1)|(1U<<2)))) {
        flyTestResult=6; flyRejected=FB_ERR_BAD_COMMAND; return 0;
    }
    flyTestDevice=132U; flyTestStep=0; flyTestResult=1;
    flyState=FB_STATE_BUSY;
    flyTestStep=1;
    if (!SetValue(0x0414,1U,flyFrontAccel) || !SetValue(0x0415,1U,flyFrontSpeed) ||
        !SetValue(0x0414,2U,flyBeltAccel) || !SetValue(0x0415,2U,flyBeltSpeed) ||
        !SetValue(0x0414,3U,flyBeltAccel) || !SetValue(0x0415,3U,flyBeltSpeed) ||
        !PositionRelative(1U,frontRel*pullSign[1]) ||
        !PositionRelative(2U,beltRel*pullSign[2]) ||
        !PositionRelative(3U,beltRel*pullSign[3])) {
        flyTestResult=3; flyState=FB_STATE_FAULT; return 0;
    }
    taskENTER_CRITICAL(); seq1=motor[1].remSeq; seq2=motor[2].remSeq; seq3=motor[3].remSeq; taskEXIT_CRITICAL();
    flyTestStep=2; start=Now();
    while (1) {
        MotorFeedback m1, m2, m3;
        taskENTER_CRITICAL(); m1=motor[1]; m2=motor[2]; m3=motor[3]; taskEXIT_CRITICAL();
        if (flyCommand==FB_CMD_STOP) { flyTestResult=5; flyState=FB_STATE_OFF; return 0; }
        if (!Guard()) { flyTestResult=3; return 0; }
        if (m1.remSeq!=seq1 && Abs32(m1.remaining)<=FB_TEST_TOLERANCE &&
            m2.remSeq!=seq2 && Abs32(m2.remaining)<=FB_TEST_TOLERANCE &&
            m3.remSeq!=seq3 && Abs32(m3.remaining)<=FB_TEST_TOLERANCE) break;
        if ((uint32_t)(Now()-start)>=FB_TEST_TIMEOUT_MS) {
            flyTestResult=4; flyError=FB_ERR_TIMEOUT; flyState=FB_STATE_FAULT; return 0;
        }
        Sleep(5);
    }
    flyTestStep=3; flyTestResult=2; flyState=FB_STATE_READY; flyError=FB_OK;
    return 1;
}
static int BeltMoveByTof26Delta(void)
{
    uint32_t start=Now();
    while (1) {
        TofFeedback t=ReadTof(1U);
        if (flyCommand==FB_CMD_STOP) { flyTestResult=5; flyState=FB_STATE_OFF; return 0; }
        if (!Guard()) { flyTestResult=3; return 0; }
        if (t.valid && t.stable>=2U && (uint32_t)(Now()-t.ms)<=FB_TOF_STALE_MS) {
            int32_t delta=(int32_t)t.raw-(int32_t)FB_SEQ_TOF26_OFFSET;
            int32_t targetDist;
            if (delta<=0) {
                targetDist=0;
            } else {
                targetDist=(int32_t)(((int64_t)delta*(int64_t)flyBeltMapDist)/(int64_t)FB_SEQ_TOF26_FULL_SPAN);
                if (targetDist>flyBeltMapDist) targetDist=flyBeltMapDist;
            }
            if (targetDist<=FB_TEST_TOLERANCE) {
                flyTestDist=targetDist;
                flyTestResult=4; flyError=FB_ERR_TOF; flyState=FB_STATE_FAULT;
                return 0;
            }
            if (!TestBeltPairDistance(-1,targetDist)) return 0;
            flyBeltTof26GoDist=targetDist;
            return 1;
        }
        if ((uint32_t)(Now()-start)>=FB_TEST_TIMEOUT_MS) {
            flyTestResult=4; flyError=FB_ERR_TOF; flyState=FB_STATE_FAULT; return 0;
        }
        Sleep(5);
    }
}
static int PushBeltBySavedTof26Dist(void)
{
    if (flyBeltTof26GoDist<=FB_TEST_TOLERANCE) {
        flyTestResult=7; flyRejected=FB_ERR_BAD_COMMAND; return 0;
    }
    return TestBeltPairDistance(1,flyBeltTof26GoDist);
}

/* 命令7第2步：臂正对箱子后，按 TOF25(raw-1000) 计算01前移目标距离。 */
static int FrontMoveByTof25Delta(void)
{
    uint32_t start;
    uint8_t dev=1U;
    int32_t primeDist=flyFrontPrimeDist;
    flyTestDevice=dev; flyTestStep=0; flyTestResult=1;
    flyFrontGoDist=0;
    if (!(flyEnabledMask&(1U<<(dev-1U)))) {
        EnableOne(dev);
        if (!(flyEnabledMask&(1U<<(dev-1U)))) { flyTestResult=6; flyRejected=FB_ERR_BAD_COMMAND; return 0; }
    }

    if (primeDist<=FB_TEST_TOLERANCE || primeDist>=flyFrontDist) {
        flyTestResult=7; flyRejected=FB_ERR_BAD_COMMAND; return 0;
    }

    /* 先短距前移，确保TOF25进入有效测距区。 */
    flyTestStep=1;
    TestOneMotor(dev,-1,primeDist,flyFrontSpeed,flyFrontAccel);
    if (flyTestResult!=2) return 0;

    flyTestStep=1;
    start=Now();
    while (1) {
        TofFeedback t;
        t=ReadTof(0U);
        if (flyCommand==FB_CMD_STOP) { flyTestResult=5; flyState=FB_STATE_OFF; return 0; }
        if (!Guard()) { flyTestResult=3; return 0; }

        if (t.valid && t.stable>=2U && (uint32_t)(Now()-t.ms)<=FB_TOF_STALE_MS) {
            int32_t delta=(int32_t)t.raw-(int32_t)FB_SEQ_TOF25_OFFSET;
            int32_t targetDist;
            int32_t remainMax=flyFrontDist-primeDist;
            if (remainMax<0) remainMax=0;
            if (delta<=0) {
                targetDist=0;
            } else {
                targetDist=(int32_t)(((int64_t)delta*(int64_t)remainMax)/(int64_t)FB_SEQ_TOF25_FULL_SPAN);
                if (targetDist>remainMax) targetDist=remainMax;
            }
            if (targetDist<=FB_TEST_TOLERANCE) {
                flyTestDist=targetDist;
                flyTestResult=4;
                flyError=FB_ERR_TOF;
                flyState=FB_STATE_FAULT;
                return 0;
            }
            flyTestStep=2;
            TestOneMotor(dev,-1,targetDist,flyFrontSpeed,flyFrontAccel);
            if (flyTestResult==2) flyFrontGoDist=primeDist+targetDist;
            return flyTestResult==2;
        }
        if ((uint32_t)(Now()-start)>=FB_TEST_TIMEOUT_MS) {
            flyTestResult=4; flyError=FB_ERR_TOF; flyState=FB_STATE_FAULT; return 0;
        }
        Sleep(5);
    }
}
static int SeqStepFailed(void)
{
    if (flyCommand==FB_CMD_STOP) { flySeqResult=5U; return 1; }
    if (flyTestResult!=2U) { flySeqResult=3U; return 1; }
    return 0;
}
static void RunSequence(void)
{
    flySeqResult=1U;
    flyFrontGoDist=0;
    flyBeltTof26GoDist=0;
    flySeqStep=1U; TestOneMotor(4U,-1,flyBaseDist,flyBaseSpeed,flyBaseAccel);
    if (SeqStepFailed()) return;
    flySeqStep=2U; if (!FrontMoveByTof25Delta()) { if (flySeqResult==1U) flySeqResult=3U; return; }
    if (SeqStepFailed()) return;
    flySeqStep=3U; TestOneMotor(5U,1,flyCustomDist,flyCustomSpeed,flyCustomAccel);
    if (SeqStepFailed()) return;
    flySeqStep=4U;
    if (!PullBackWithBelt()) { if (flySeqResult==1U) flySeqResult=3U; return; }
    if (SeqStepFailed()) return;
    flySeqStep=5U; TestOneMotor(5U,-1,flyCustomDist,flyCustomSpeed,flyCustomAccel);
    if (SeqStepFailed()) return;
    flySeqStep=6U; TestOneMotor(4U,1,flyBaseDist,flyBaseSpeed,flyBaseAccel);
    if (SeqStepFailed()) return;
    flySeqStep=7U;
    if (!BeltMoveByTof26Delta()) { if (flySeqResult==1U) flySeqResult=3U; return; }
    if (SeqStepFailed()) return;
    flySeqStep=8U; flySeqResult=2U;
}
static void RunPushSequence(void)
{
    flySeqResult=1U;
    if (flyFrontGoDist<=FB_TEST_TOLERANCE || flyBeltTof26GoDist<=FB_TEST_TOLERANCE) {
        flyTestResult=7; flyRejected=FB_ERR_BAD_COMMAND; flySeqResult=3U; return;
    }
    /* 推箱流程：履带二段反向 -> 底座正对 -> 抓钩抬起 -> 01+履带推箱 -> 抓钩下放 -> 01回位 -> 04底座回收 */
    flySeqStep=1U;
    if (!PushBeltBySavedTof26Dist()) { if (flySeqResult==1U) flySeqResult=3U; return; }
    if (SeqStepFailed()) return;
    flySeqStep=2U; TestOneMotor(4U,-1,flyBaseDist,flyBaseSpeed,flyBaseAccel);
    if (SeqStepFailed()) return;
    flySeqStep=3U; TestOneMotor(5U,1,flyCustomDist,flyCustomSpeed,flyCustomAccel);
    if (SeqStepFailed()) return;
    flySeqStep=4U;
    if (!PushForwardWithBelt()) { if (flySeqResult==1U) flySeqResult=3U; return; }
    if (SeqStepFailed()) return;
    flySeqStep=5U; TestOneMotor(5U,-1,flyCustomDist,flyCustomSpeed,flyCustomAccel);
    if (SeqStepFailed()) return;
    flySeqStep=6U; TestOneMotor(1U,1,flyFrontGoDist,flyFrontSpeed,flyFrontAccel);
    if (SeqStepFailed()) return;
    flySeqStep=7U; TestOneMotor(4U,1,flyBaseDist,flyBaseSpeed,flyBaseAccel);
    if (SeqStepFailed()) return;
    flySeqStep=8U;
    /* 流程完成保持使能，不发送0x4A下电指令。 */
    flyState=FB_STATE_READY; flyError=FB_OK;
    flySeqResult=2U;
}

/* command9：依次执行原命令3、10、20、7、8，不改变各动作函数。 */
static void RunCombinedFlow(void)
{
    static const uint32_t commands[5]={FB_CMD_HOME,FB_CMD_ENABLE_ALL,FB_CMD_TOF_ALL,
                                       FB_CMD_SEQUENCE,FB_CMD_PUSH_SEQUENCE};
    flyFlowResult=1U; flyFlowStep=0U; flyFlowCommand=0U;
    for (uint8_t i=0U; i<5U; ++i) {
        uint32_t result;
        if (flyCommand==FB_CMD_STOP) goto stopped;
        flyFlowStep=(uint32_t)i+1U; flyFlowCommand=commands[i];
        switch (commands[i]) {
        case FB_CMD_HOME:
            RunHoming(); result=flyHomeResult;
            if (result==4U) goto stopped;
            if (result==2U && !flyHomed) { flyRejected=FB_ERR_BAD_COMMAND; result=3U; }
            break;
        case FB_CMD_ENABLE_ALL:
            EnableAll(); result=flyEnableResult; break;
        case FB_CMD_TOF_ALL:
            ConfigureTofAll(); result=flyTofResult; break;
        case FB_CMD_SEQUENCE:
            RunSequence(); result=flySeqResult; break;
        default:
            RunPushSequence(); result=flySeqResult; break;
        }
        if (flyCommand==FB_CMD_STOP || (i!=0U && result==5U)) goto stopped;
        if (result!=2U || flyState==FB_STATE_FAULT) {
            /* 部分动作已发出时也尝试停止并保持；不释放抓钩负载。 */
            for (uint8_t dev=1U; dev<=5U; ++dev) {
                uint8_t d[8]={0}; d[4]=dev;
                (void)CAN_SendFrameWait(0x0416,d,6,5);
            }
            flyFlowResult=3U; flyState=FB_STATE_FAULT;
            return;
        }
    }
    flyFlowResult=2U; flyState=FB_STATE_READY;
    return; /* 保持command8原有的完成后使能状态，不下电。 */
stopped:
    flyCommand=0U;
    StopAll();
    flyFlowResult=5U;
}


int FlyBox_Request(uint32_t cmd)
{
    int accepted=0;
    if (!((cmd==FB_CMD_STOP || cmd==FB_CMD_IDLE || cmd==FB_CMD_SEQUENCE || cmd==FB_CMD_PUSH_SEQUENCE ||
            cmd==FB_CMD_COMBINED || cmd==FB_CMD_ENABLE_ALL || cmd==FB_CMD_TOF_ALL ||
            cmd==FB_CMD_HOME_FRONT || cmd==FB_CMD_HOME_BASE || cmd==FB_CMD_HOME || cmd==FB_CMD_HOME_HOOK ||
           cmd==FB_CMD_BELT_FWD || cmd==FB_CMD_BELT_REV) ||
          (cmd>=FB_CMD_ENABLE_FIRST && cmd<=FB_CMD_ENABLE_LAST) ||
          cmd==FB_CMD_TOF25 || cmd==FB_CMD_TOF26 ||
          (cmd>=FB_CMD_TEST_FWD_FIRST && cmd<=FB_CMD_TEST_FWD_LAST) ||
          (cmd>=FB_CMD_TEST_REV_FIRST && cmd<=FB_CMD_TEST_REV_LAST) ||
          cmd==FB_CMD_GRIP_FWD || cmd==FB_CMD_GRIP_REV)) return 0;
    taskENTER_CRITICAL();
    if ((flyCommand==0 && flyFlowResult!=1U) || cmd==FB_CMD_STOP) { flyCommand=cmd; accepted=1; }
    taskEXIT_CRITICAL();
    return accepted;
}
static void ControlTask(void *arg)
{
    (void)arg;
    /* 上电自动初始化：先延时等电源稳定/人员离开；写 STOP 跳过；
     * 已经写了单轴命令1/2/4就交给主循环，不再重复做全部三轴。 */
    if (flyAutoHome) {
        uint32_t t0=Now();
        while ((uint32_t)(Now()-t0)<FB_HOME_BOOT_DELAY_MS && flyCommand!=FB_CMD_STOP) Sleep(10);
        if (flyCommand==FB_CMD_HOME) flyCommand=0U;
        if (flyCommand!=FB_CMD_STOP &&
            flyCommand!=FB_CMD_HOME_FRONT &&
            flyCommand!=FB_CMD_HOME_BASE &&
            flyCommand!=FB_CMD_HOME_HOOK && flyCommand!=FB_CMD_COMBINED)
            RunHoming();
    }
    for (;;) {
        uint32_t cmd;
        taskENTER_CRITICAL(); cmd=flyCommand; flyCommand=0; taskEXIT_CRITICAL();
        if (cmd) { flyLastCmd=cmd; flyRejected=FB_OK; }
        if (cmd==FB_CMD_STOP) StopAll();
        else if (cmd==FB_CMD_IDLE) EnterIdle();
        else if (flyState==FB_STATE_BUSY) flyRejected=FB_ERR_BAD_COMMAND;
        else if (cmd==FB_CMD_COMBINED) RunCombinedFlow();
        else if (cmd==FB_CMD_HOME) RunHoming();
        else if (cmd==FB_CMD_HOME_FRONT) RunHomingMask(1U<<0);
        else if (cmd==FB_CMD_HOME_BASE) RunHomingMask(1U<<3);
        else if (cmd==FB_CMD_HOME_HOOK) RunHomingMask(1U<<4);
        else if (cmd==FB_CMD_SEQUENCE) RunSequence();
        else if (cmd==FB_CMD_PUSH_SEQUENCE) RunPushSequence();
        else if (cmd==FB_CMD_ENABLE_ALL) EnableAll();
        else if (cmd==FB_CMD_TOF_ALL) ConfigureTofAll();
        else if (cmd>=FB_CMD_ENABLE_FIRST && cmd<=FB_CMD_ENABLE_LAST) EnableOne((uint8_t)(cmd-10U));
        else if (cmd==FB_CMD_TOF25 || cmd==FB_CMD_TOF26) ConfigureTofOne((uint8_t)cmd);
        else if (cmd==FB_CMD_BASE_FWD) TestOneMotor(4U,1,flyBaseDist,flyBaseSpeed,flyBaseAccel);
        else if (cmd==FB_CMD_BASE_REV) TestOneMotor(4U,-1,flyBaseDist,flyBaseSpeed,flyBaseAccel);
        else if (cmd==FB_CMD_FRONT_FWD) TestOneMotor(1U,1,flyFrontDist,flyFrontSpeed,flyFrontAccel);
        else if (cmd==FB_CMD_FRONT_REV) TestOneMotor(1U,-1,flyFrontDist,flyFrontSpeed,flyFrontAccel);
        else if (cmd==FB_CMD_BELT_FWD) TestBeltPair(1);
        else if (cmd==FB_CMD_BELT_REV) TestBeltPair(-1);
        else if (cmd>=FB_CMD_TEST_FWD_FIRST && cmd<=FB_CMD_TEST_FWD_LAST) TestOneMotor((uint8_t)(cmd-30U),1,FB_TEST_DISTANCE,FB_TEST_SPEED,1000);
        else if (cmd>=FB_CMD_TEST_REV_FIRST && cmd<=FB_CMD_TEST_REV_LAST) TestOneMotor((uint8_t)(cmd-40U),-1,FB_TEST_DISTANCE,FB_TEST_SPEED,1000);
        else if (cmd==FB_CMD_GRIP_FWD) TestOneMotor(5U,1,flyCustomDist,flyCustomSpeed,flyCustomAccel);
        else if (cmd==FB_CMD_GRIP_REV) TestOneMotor(5U,-1,flyCustomDist,flyCustomSpeed,flyCustomAccel);
        else if (cmd) flyRejected=FB_ERR_BAD_COMMAND;
        ++flyTaskControlLoops;
        Sleep(5);
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED) xPortSysTickHandler();
}
int main(void)
{
    HAL_Init(); SystemClock_Config();
    if (CAN_Init()!=HAL_OK || CAN_TransmitMutexInit()!=HAL_OK) Error_Handler();
    ackQueue=xQueueCreate(16,sizeof(Ack));
    if (ackQueue==NULL ||
        xTaskCreate(ReceiveTask,"CAN_RX",256,NULL,4,NULL)!=pdPASS ||
        xTaskCreate(TofTask,"TOF",256,NULL,2,NULL)!=pdPASS ||
        xTaskCreate(ControlTask,"CTRL",512,NULL,3,NULL)!=pdPASS) {
        flyError=FB_ERR_ALLOCATION; Error_Handler();
    }
    fifo0Baseline=canRxFifo0OverrunCount; fifo1Baseline=canRxFifo1OverrunCount;
    rxOverflowWindowStartMs=Now();
    vTaskStartScheduler();
    flyError=FB_ERR_ALLOCATION; Error_Handler();
    return 0;
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc={0}; RCC_ClkInitTypeDef clk={0};
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSE; osc.HSEState=RCC_HSE_ON;
    osc.HSEPredivValue=RCC_HSE_PREDIV_DIV1; osc.PLL.PLLState=RCC_PLL_ON;
    osc.PLL.PLLSource=RCC_PLLSOURCE_HSE; osc.PLL.PLLMUL=RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc)!=HAL_OK) Error_Handler();
    clk.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider=RCC_HCLK_DIV2; clk.APB2CLKDivider=RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_2)!=HAL_OK) Error_Handler();
}
static void Error_Handler(void)
{
    __disable_irq();
    for (;;) { }
}
