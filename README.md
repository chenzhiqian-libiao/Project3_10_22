# Project3_10_22

这是一个仅保留 CAN 驱动的 STM32/VisualGDB 工程。

## 工程目标
- 保留并验证 `can.c` / `can.h` 的收发与诊断能力
- 移除课题一/二、陀螺仪、小车控制等历史业务代码

## 关键文件
- `Project3_10_22.sln`
- `Project3_10_22.vcxproj`
- `Project3_10_22.c`（CAN-only 入口）
- `can.c`
- `can.h`

## 运行行为
程序启动后会初始化时钟与 CAN 外设，并每 500ms 发送一个测试字节。
