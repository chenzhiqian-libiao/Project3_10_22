/* 电机/TOF独立调试版（不含课题流程状态机）。
 * Watch 写入 flyCommand：
 * 6=回到IDLE（清故障/重置接收溢出基线）；
 * 5=停止全部电机；
 * 7=完整动作流程：底座正对箱子→前推箱→钩箱→前回原位→放箱→回收底座
 *       （进度flySeqStep 1~6；结果flySeqResult：2=完成 3=某步失败 5=停止打断）；
 * 10=一键绑定+配置+使能电机01~05；
 * 11~15=分别绑定+配置+使能电机01~05；
 * 20=一键配置TOF25+TOF26并开启持续测距；
 * 25/26=分别绑定+配置TOF25/TOF26并开启持续测距；
 * 31~35=分别测试电机01~05正向点动；
 * 41~45=分别测试电机01~05反向点动；
 * 34/44=底座（电机04）动作：44=底座正对箱子，34=回收底座（从初始位置起），
 *       参数已按实际校准写死：flyBaseDist=14000、flyBaseSpeed=20000、
 *       flyBaseAccel=20000（单位0.1mm / 0.1mm/s / 0.1mm/s^2，可Watch改）。
 * 31/41=前（电机01）动作：41=向前推箱子（最大行程），31=回到原位（从初始位置起），
 *       参数已按实际校准写死：flyFrontDist=45000、flyFrontSpeed=15000、
 *       flyFrontAccel=20000（单位0.1mm / 0.1mm/s / 0.1mm/s^2，可Watch改）。
 * 36/48=左右皮带（电机02/03）同步动作：36=履带向外送箱子，48=履带向里送箱子，
 *       参数已按实际校准：flyBeltSpeed=5000、flyBeltAccel=20000
 *       （flyBeltDist=300，单位0.1mm / 0.1mm/s / 0.1mm/s^2，可Watch改）；
 * 46/47=抓钩（电机05）动作：46=抬起钩住箱子，47=放下箱子（从抓钩初始位置起），
 *       参数已按实际校准写死：flyCustomDist=-22000、flyCustomSpeed=20000、
 *       flyCustomAccel=100000（单位0.1mm / 0.1mm/s / 0.1mm/s^2，可Watch改）。
 *       后续课题流程需要这个钩箱/放箱动作。
 * 31~47点动命令在该电机未使能时会自动先使能（复位/重烧/下电后可直接测试）；
 * 6会停转并下电全部电机，之后直接发点动命令即可自动重新使能。
 */
#include "stm32f1xx_hal.h"
#include "Feixiang_can.h"
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

#define FB_CMD_STOP            5U
#define FB_CMD_IDLE            6U
#define FB_CMD_SEQUENCE        7U
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
} MotorFeedback;
typedef struct {
    uint16_t raw;
    uint8_t valid, stable;
    uint32_t seq, ms, rxSeq, rxMs;
} TofFeedback;

volatile uint32_t flyCommand;
volatile uint32_t flyLastCmd;          /* ControlTask最近消费的命令号；确认Watch写入是否到达 */
volatile int32_t flyTestDist;          /* 点动测试实际下发的相对距离（含方向符号） */
volatile int32_t flyRem[6];            /* 电机反馈remaining镜像（flyRem[5]=抓钩），Watch观察 */
volatile uint32_t flyRemSeq[6];        /* 对应反馈序号，变化说明收到新反馈 */
volatile uint32_t flySeqStep;          /* 完整流程(7)当前步骤：1~6，7=全部完成 */
volatile uint32_t flySeqResult;        /* 完整流程(7)结果：0未执行 1执行中 2完成 3某步失败 5停止打断 */
volatile int32_t flyCustomDist = -22000; /* 抓钩动作距离（0.1mm）：初始位置下46=抬起钩住箱子，47=放下箱子 */
volatile int32_t flyCustomSpeed = 20000; /* 抓钩动作速度（0.1mm/s） */
volatile int32_t flyCustomAccel = 100000;/* 抓钩动作加速度（0.1mm/s^2） */
volatile int32_t flyBaseDist = 14000;   /* 底座动作距离（0.1mm）：初始位置下44=正对箱子，34=回收底座 */
volatile int32_t flyBaseSpeed = 20000;  /* 底座动作速度（0.1mm/s） */
volatile int32_t flyBaseAccel = 20000;  /* 底座动作加速度（0.1mm/s^2） */
volatile int32_t flyFrontDist = 45000;  /* 前（电机01）动作距离（0.1mm）：初始位置下41=向前推箱子（最大行程），31=回到原位 */
volatile int32_t flyFrontSpeed = 15000; /* 前动作速度（0.1mm/s） */
volatile int32_t flyFrontAccel = 20000; /* 前动作加速度（0.1mm/s^2） */
volatile int32_t flyBeltDist = 300;     /* 左右皮带（电机02/03）同步动作距离（0.1mm），36/48使用 */
volatile int32_t flyBeltSpeed = 5000;   /* 皮带同步速度（0.1mm/s） */
volatile int32_t flyBeltAccel = 20000;  /* 皮带同步加速度（0.1mm/s^2） */
volatile FlyState flyState = FB_STATE_OFF;
volatile FlyError flyError = FB_OK;
volatile FlyError flyRejected = FB_OK;
volatile uint32_t flyAckWaitId, flyAckWaitDevice;
volatile uint32_t flyLastAckId, flyLastAckDevice, flyMalformedFrames;
volatile uint32_t flyEnabledMask; /* bit0~4 for motor01~05 */
volatile uint32_t flyEnableDevice, flyEnableStep, flyEnableResult;
/* 0未测试，1执行中，2成功，3ACK超时，4发送失败，5停止打断，6参数不允许 */
volatile uint32_t flyTofDevice, flyTofStep, flyTofResult;
/* 0未测试，1执行中，2成功，3测距超时，4发送失败，5停止打断，6参数不允许 */
volatile uint32_t flyTestDevice, flyTestStep, flyTestResult;
/* 0未测试，1执行中，2完成，3发送/ACK/反馈失败，4超时，5停止打断，6未使能，7参数或状态不允许 */
volatile uint32_t flyTof25Raw, flyTof26Raw;
volatile uint32_t flySnSeenCount;
volatile uint8_t flySnLast[7];          /* 最近一次0x0312应答的设备SN，用于核对绑定对象 */
volatile uint32_t flyLastMotorError, flyLastMotorErrorCanId;
volatile uint32_t flyTaskRxLoops, flyTaskControlLoops;
volatile uint32_t flyRxOverflowTotal, flyRxOverflowBurst;

static QueueHandle_t ackQueue;
static volatile MotorFeedback motor[6];
static volatile TofFeedback tof[2];
static volatile uint32_t tofPollingMask; /* bit0 TOF25, bit1 TOF26 */
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
    if (id==0x0409U && len>=2U) {
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
        flyLastMotorError=(uint32_t)d[0]|((uint32_t)d[1]<<8);
        if (FB_IGNORE_ERR_FRAME_010X && id<=0x010FU) return;
        flyLastMotorErrorCanId=id; rxFault=FB_ERR_MOTOR;
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
    if (flyCommand==FB_CMD_STOP) { flyCommand=0; return 0; }
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

static void StopAll(void)
{
    flyState=FB_STATE_BUSY;
    for (uint8_t id=1; id<=5; ++id) {
        uint8_t d[8]={0x4A,0,0,0,id};
        if (CAN_SendFrameWait(0x0413,d,5,5)!=HAL_OK) flyError=FB_ERR_TX;
        Sleep(2);
    }
    flyState=FB_STATE_OFF;
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
    flyEnableStep=2; memcpy(d,motorParam,6); d[6]=dev; d[7]=dev;
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
        if (flyEnableResult!=2U) return; /* 保留失败码，flyEnableDevice 指示停在哪台 */
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
    const uint8_t config[8]={0x11,0x22,0,0xFF,0,0x05,0,dev};
    if (!SendRaw(0x0316,config,8)) { flyTofResult=4; return; }
    Sleep(20);
    flyTofStep=3;
    const uint8_t startFrame[8]={dev,10,1,0x11,0x22,0,0,0};
    if (!SendRaw(0x0408,startFrame,8)) { flyTofResult=4; return; }
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
static void TestOneMotor(uint8_t dev, int8_t direction, int32_t distance, int32_t speed, int32_t accel)
{
    uint32_t start, seq;
    int32_t rel=(direction>=0) ? distance : -distance;
    flyTestDevice=dev; flyTestStep=0; flyTestResult=1;
    if (dev<1U || dev>5U) { flyTestResult=7; flyRejected=FB_ERR_BAD_COMMAND; return; }
    if (!(flyEnabledMask&(1U<<(dev-1U)))) {
        EnableOne(dev); /* 未使能（复位/重烧/下电后）自动先使能 */
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
/* 36/48：左右皮带（电机02/03）同步点动。两台位置命令连续下发，同时等待两台到位。 */
static void TestBeltPair(int8_t direction)
{
    uint32_t seq2, seq3, start;
    int32_t rel=(direction>=0) ? flyBeltDist : -flyBeltDist;
    flyTestDevice=23U; flyTestStep=0; flyTestResult=1;
    if (!(flyEnabledMask&(1U<<1))) EnableOne(2U); /* 未使能自动先使能 */
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
/* 命令7：完整推拉箱流程。某步失败立即中止；STOP随时可打断。 */
static int SeqStepFailed(void)
{
    if (flyCommand==FB_CMD_STOP) { flySeqResult=5U; return 1; }
    if (flyTestResult!=2U) { flySeqResult=3U; return 1; }
    return 0;
}
static void RunSequence(void)
{
    flySeqResult=1U;
    flySeqStep=1U; TestOneMotor(4U,-1,flyBaseDist,flyBaseSpeed,flyBaseAccel);      /* 44 底座正对箱子 */
    if (SeqStepFailed()) return;
    flySeqStep=2U; TestOneMotor(1U,-1,flyFrontDist,flyFrontSpeed,flyFrontAccel);  /* 41 前推箱到最大行程 */
    if (SeqStepFailed()) return;
    flySeqStep=3U; TestOneMotor(5U,1,flyCustomDist,flyCustomSpeed,flyCustomAccel);/* 46 抓钩钩住箱子 */
    if (SeqStepFailed()) return;
    flySeqStep=4U; TestOneMotor(1U,1,flyFrontDist,flyFrontSpeed,flyFrontAccel);   /* 31 前回到原位 */
    if (SeqStepFailed()) return;
    flySeqStep=5U; TestOneMotor(5U,-1,flyCustomDist,flyCustomSpeed,flyCustomAccel);/* 47 抓钩放下箱子 */
    if (SeqStepFailed()) return;
    flySeqStep=6U; TestOneMotor(4U,1,flyBaseDist,flyBaseSpeed,flyBaseAccel);      /* 34 回收底座 */
    if (SeqStepFailed()) return;
    flySeqStep=7U; flySeqResult=2U;
}

/* 任务上下文入口。STOP优先。 */
int FlyBox_Request(uint32_t cmd)
{
    int accepted=0;
    if (!((cmd==FB_CMD_STOP || cmd==FB_CMD_IDLE || cmd==FB_CMD_SEQUENCE ||
           cmd==FB_CMD_ENABLE_ALL || cmd==FB_CMD_TOF_ALL ||
           cmd==FB_CMD_BELT_FWD || cmd==FB_CMD_BELT_REV) ||
          (cmd>=FB_CMD_ENABLE_FIRST && cmd<=FB_CMD_ENABLE_LAST) ||
          cmd==FB_CMD_TOF25 || cmd==FB_CMD_TOF26 ||
          (cmd>=FB_CMD_TEST_FWD_FIRST && cmd<=FB_CMD_TEST_FWD_LAST) ||
          (cmd>=FB_CMD_TEST_REV_FIRST && cmd<=FB_CMD_TEST_REV_LAST) ||
      cmd==FB_CMD_GRIP_FWD || cmd==FB_CMD_GRIP_REV)) return 0;
    taskENTER_CRITICAL();
    if (flyCommand==0 || cmd==FB_CMD_STOP) { flyCommand=cmd; accepted=1; }
    taskEXIT_CRITICAL();
    return accepted;
}
static void ControlTask(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t cmd;
        taskENTER_CRITICAL(); cmd=flyCommand; flyCommand=0; taskEXIT_CRITICAL();
        if (cmd) { flyLastCmd=cmd; flyRejected=FB_OK; }
        if (cmd==FB_CMD_STOP) StopAll();
        else if (cmd==FB_CMD_IDLE) EnterIdle();
        else if (flyState==FB_STATE_BUSY) flyRejected=FB_ERR_BAD_COMMAND;
        else if (cmd==FB_CMD_SEQUENCE) RunSequence();
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

/* HAL与FreeRTOS共享1kHz SysTick，调度器启动前只维护HAL时基。 */
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
