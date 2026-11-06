# DJ-Omni_Infantry 代码阅读框架（面向 AI）

> 本文档目的：让 AI 在**不逐行读完 5 万行代码**的前提下，快速定位「某个功能在哪一层、由哪个函数实现、被谁调用」。
> 阅读顺序建议：**§1 → §2 → §4 → §5 → §6 → §7（按需查表）→ §11 陷阱必读**。

---

## 0. 给 AI 的阅读协议（先读这一段）

1. **先读 `Src/freertos.c`，但不要相信它。** 它里面的 9 个任务函数全部是 `__weak` 空壳（`osDelay(1)` 死循环）。真正的实现分散在 `Src/Application/app_*.c`，靠链接期强符号覆盖。**看到任务名 → 直接去 `Src/Application/` 找同名文件。**
2. **`Drivers/` 与 `Middlewares/` 是 ST 与 FreeRTOS 官方代码，整体跳过。** 只有 `Inc/stm32f4xx_hal_conf.h`（HAL 模块开关）值得瞄一眼。
3. **`MDK-ARM/gc_chass/` 是另一套废弃工程**（MPU9250 + 自制 EKF 的旧底盘工程），不在 `gimbal_task.uvprojx` 的编译列表里。**完全忽略。**
4. **源文件编码混杂**：`Src/`+`Inc/` 共 122 个文件，其中 **32 个是 GBK**、90 个是 UTF-8。用直读工具打开 GBK 文件会报 `invalid UTF-8`。需要先转码（脚本见 **附录 A**）。
5. **注释不可信**。本工程有大量"注释写着 A、代码做 B"的情况（尤其是达妙电机 ID、电机名、死字段）。**一律以代码为准**，§11 列出了已确认的全部坑。
6. 判断"这段代码是否真的在跑"，用 §12 的死代码清单交叉验证 —— 本工程约有 **4 个完整模块从未被调用**。
7. **基线声明**：本文档描述的是**当前工作区（working tree）**的状态，不是 `git HEAD`。截至撰写时工作区有 1 处未提交的源码改动 —— `Src/Modules/module_shoot.c` 中把 `Shooter_FeederMotorLockedJudge()` 注释掉了（见 §11.5-25）。若你阅读的是已提交版本，这一处会不同。

---

## 1. 项目一句话定位

**RoboMaster 全向轮步兵机器人主控固件**。单块 STM32F407IGHx（UFBGA176 封装）跑 FreeRTOS，通过 CAN 驱动 8 个电机（4× 底盘 RM3508 + 1× 云台 Yaw RM6020 + 1× 拨弹 RM2006 + 2× 摩擦轮 RM3508）、1 个达妙 MIT 直驱电机（Pitch 轴）、1 块超级电容功率板，通过 3 路串口接遥控器 DBUS、裁判系统、视觉（自瞄）上位机。

**框架血统**：代码骨架来自 *Polaris Robot* 框架（`app_` / `module_` / `periph_` / `alg_` / `util_` 五段命名），后被 RoboMaster 队伍改造。因此**存在大量从"平衡双腿轮式底盘"遗留的字段和协议**（`leg_len`、`bal_mode`、`Chassis_StatusTypeDef` 里的 `WheelLeg_*`），这些多数已是死字段。

| 项 | 值 |
|---|---|
| MCU | STM32F407IGHx（Cortex-M4F，UFBGA176） |
| 主频 | 168 MHz（HSE 12 MHz → PLLM=6, PLLN=168, PLLP=2） |
| RTOS | FreeRTOS + CMSIS-RTOS v1（heap_4，静态 Idle/Timer 内存） |
| CAN | CAN1 / CAN2，均 1 Mbps（Prescaler=3, BS1=10TQ, BS2=3TQ） |
| 构建 | Keil MDK（`MDK-ARM/gimbal_task.uvprojx`），工程名 `gimbal_task` |

---

## 2. 分层架构

依赖方向**严格单向向下**，上层可调下层，下层不得反向 include 上层（少数例外：`module_gimbal.c` 会读 `app_ins.h` 和 `protocol_common.h`、`app_remote.h`，属于历史耦合，是重构的技术债点）。

```
┌────────────────────────────────────────────────────────────────┐
│  L5  CubeMX 生成层   Src/{main,freertos,can,tim,usart,spi,      │
│                      dma,gpio,stm32f4xx_it,stm32f4xx_hal_msp}.c │
│                      → 只做硬件初始化 + 中断向量 + 任务骨架      │
├────────────────────────────────────────────────────────────────┤
│  L4  Application 层   Src/Application/app_*.c                   │
│      任务入口 + 模式状态机 + 输入解算（把拨杆/键鼠翻译成控制量） │
│      app_init / app_remote / app_ins / app_autoaim / app_gimbal │
│      app_chassis / app_shoot / app_referee / app_communicate    │
│      app_watchDog / app_Debug                                   │
├────────────────────────────────────────────────────────────────┤
│  L3  Modules 层       Src/Modules/module_*.c     ★业务核心★     │
│      每个被控对象一套 控制器 + 状态机 + 输出函数                 │
│      module_gimbal / module_chassis / module_shoot /            │
│      module_dm4310 / module_power / module_referee(空)          │
├────────────────────────────────────────────────────────────────┤
│  L3' Protocol 层      Src/Protocol/protocol_*.c                 │
│      CAN 总线表驱动收发（→ 另一块板的遗留协议）                  │
├────────────────────────────────────────────────────────────────┤
│  L2  Peripheral 层    Src/Periphal/periph_*.c    ★硬件抽象★     │
│      电机对象/电机组、达妙协议、BMI088、遥控器、裁判系统、       │
│      视觉串口、裁判UI绘制、舵机                                │
├────────────────────────────────────────────────────────────────┤
│  L1  Algorithm 层     Src/Algorithm/alg_*.c                     │
│      PID / 卡尔曼 / 四元数EKF / 滤波 / 数学工具 / CRC16         │
├────────────────────────────────────────────────────────────────┤
│  L0  System+Utility   Src/System/sys_*.c  Src/Utility/util_*.c  │
│                      Src/Library/lib_*.c                        │
│      DWT 计时 / 软定时器 / 全局常量表 / CAN&UART&SPI&PWM 封装   │
└────────────────────────────────────────────────────────────────┘
```

**三层调用范式**（读懂这 3 句就懂整个工程）：

```
periph_  : 我只负责"把物理量变成结构体字段"（解码）和"把结构体字段变成 CAN 帧"（编码）
module_  : 我持有 PID，读 periph_ 的反馈、算出 output 写回 periph_ 的 output 缓存，但我不发帧
app_     : 我按拨杆状态选模式，调 module_ 的 Set*Ref()，然后调 module_ 的 Output() 真正发帧
```

---

## 3. 启动流程

```
Reset
 └─ Src/main.c : main()
     ├─ HAL_Init()                     ← 时基是 TIM6（Src/stm32f4xx_hal_timebase_tim.c）
     ├─ SystemClock_Config()           ← 168 MHz
     ├─ MX_{GPIO,DMA,CAN1,CAN2,SPI1,TIM1,TIM8,TIM12,TIM3,USART3,USART6,USART1}_Init()
     ├─ Init_InitAll()                 ← ★全部业务初始化，见 app_init.c
     │   ├─ DWT_Init(168)                     DWT 周期计数（所有 dt 来源）
     │   ├─ BMI088_Init(0)                    IMU（SPI1，不做零偏标定）
     │   ├─ Remote_InitRemote()               USART3 DMA 接收启动
     │   ├─ INS_Init()                        QuaternionEKF_Init(10, 0.01, 1e7, 1, 0.0085)
     │   ├─ Motor_InitAllMotors()             ★注册 3 个电机组 + 9 个电机对象
     │   ├─ Can_InitFilterAndStart(hcan1/2)   全通滤波 → FIFO0 → 开中断
     │   ├─ Referee_InitReferee() / Referee_Setup()   裁判系统 + UI 初始画面
     │   ├─ GimbalPitch/Yaw_Init*()           云台 PID 参数表装载
     │   ├─ Shooter_InitShooter()             发射 PID 参数表装载
     │   ├─ Chassis_InitChassis()             4 个底盘速度环 PID
     │   ├─ Remote_RemotrControlInit()        斜坡发生器参数
     │   ├─ Protocol_InitProtocol()           总线协议（遗留）
     │   ├─ dm_motor_init()                   达妙 MIT 映射范围 PMAX/VMAX/TMAX
     │   ├─ dm_motor_enable(&hcan1, &motor[Motor1])   ★发一次使能帧
     │   └─ SuperCap_InitHeader/SetPower(70W)  超级电容设 70 W
     ├─ usart1_tx_dma_init() / usart1_rx_dma_init()  视觉串口双缓冲 DMA
     ├─ HAL_TIM_Base_Start_IT(&htim3)      20 ms 周期中断（置 g_uart6_print_flag）
     ├─ MX_FREERTOS_Init()                 创建 9 个任务 + 1 个软定时器
     └─ osKernelStart()
```

**注意 `Init_Task`（app_init.c）不是在 main 里跑的业务**：它只做 `SoftTimer_StartAll()` 然后 `vTaskSuspend(自己)` 永久挂起。业务初始化全部在 `main()` 里的 `Init_InitAll()` 同步完成（**在调度器启动前**）。

---

## 4. FreeRTOS 运行时全景

所有任务函数都在 `Src/freertos.c` 中声明为 `__weak`，由 `Src/Application/app_*.c` 强符号覆盖。

| 任务 | 定义位置 | 优先级 | 栈 | 循环周期 | 每轮做什么 |
|---|---|---|---|---|---|
| `Init_Task` | `app_init.c` | Realtime | 128 | **永不循环** | 起软定时器后自挂起 |
| `Comm_Task` | `app_communicate.c` | Realtime | 128 | `osDelay(1)` | `Protocol_SendProtocolData()`（CAN2 发 4 帧遗留协议） |
| `Ins_Task` | `app_ins.c` | Realtime | 128 | `osDelay(1)` | BMI088 解码 → `QuaternionEKF_Update` → 更新 `INS` 全局 |
| `Remote_Task` | `app_remote.c` | Realtime | **256** | `osDelay(1)` | `Remote_ControlCom()` ★**全部手动输入 → 各模块 SetRef** |
| `Gimbal_Task` | `app_gimbal.c` | Realtime | 128 | `osDelay(2)` | 视觉收发 + 云台 P/Y 控制 + 发 CAN |
| `Shoot_Task` | `app_shoot.c` | Realtime | 128 | `osDelay(2)` | `Shooter_Control()` + 摩擦轮发帧 |
| `Chassis_Task` | `app_chassis.c` | Realtime | 128 | `osDelay(2)` | 电容功率自检 + 4 轮速度环 + 发帧 |
| `Referee_Task` | `app_referee.c` | Realtime | 128 | `osDelay(200)` | `Referee_Update()`（裁判 UI 刷新） |
| `WatchDog_Task` | `app_watchDog.c` | **High** | 128 | `osDelay(50)` | `WatchDog_Feed()` 汇总离线位图到 `Dog_Bowl` |
| `SoftTimer` | `sys_softTimer.c` | — | — | 250 tick | 空回调 |

**读代码时必须注意的三点：**

- **几乎所有任务都是 `osPriorityRealtime`（最高）**，调度靠时间片轮转。"实时"在这里不等于"更高优先级"，`WatchDog_Task` 反而是 `osPriorityHigh`（**低于**其它所有任务）。
- 任务之间**没有队列、没有信号量、没有互斥锁**，全部通过全局结构体裸共享。`static` 修饰的 `pending` 标志是唯一的伪保护手段，实际不生效。
- 真正的"节拍"不是 `osDelay`，而是 **CAN 帧到达中断**（反馈更新 `last_update_time`）与 **DWT**（所有 `dt`）。

---

## 5. 中断与数据入口（所有反馈都从这里进来）

| 中断 | 处理链 | 说明 |
|---|---|---|
| `CAN1_RX0_IRQHandler` | `HAL_CAN_IRQHandler(&hcan1)` → `HAL_CAN_RxFifo0MsgPendingCallback` → **`Can_RxMessageCallback`** | CAN1：`StdId==0x00` → 达妙反馈；其它 → `Motor_EncoderDecodeCallback`（含 `0x211` 超级电容） |
| `CAN2_RX0_IRQHandler` | 同上 → `Can_RxMessageCallback` | CAN2：只走 `Motor_EncoderDecodeCallback`（摩擦轮 0x201/0x202） |
| `USART3_IRQHandler` | `Uart_ReceiveHandler(&huart3)` → IDLE → `Uart_RxIdleCallback` → **`Remote_RXCallback`** → `Remote_DecodeRemoteData` | 遥控器 DBUS，18 字节定长帧 |
| `USART6_IRQHandler` | `Uart_ReceiveHandler(&huart6)` → **`Referee_RXCallback`** → `Referee_DecodeRefereeData` → `Referee_ParseRefereeCmd` | 裁判系统。**注意：`Debug_RXCallback` 在此处被注释掉了** |
| `USART1_IRQHandler` | **用户自定义代码内联在 `stm32f4xx_it.c` 里**（非 HAL 回调）：IDLE 中断 + DMA 双缓冲 → **`GetVisionData(&visionDataGet, buf)`** | 视觉上位机，30 字节双缓冲 / 15 字节有效帧 |
| `TIM3_IRQHandler` | `HAL_TIM_IRQHandler` → `HAL_TIM_PeriodElapsedCallback`（在 `main.c`）→ `g_uart6_print_flag = 1` | 839/1999 分频 → **约 50 Hz ≈ 20 ms**（注释写"每 10ms"，不准） |
| `TIM6_DAC_IRQHandler` | `HAL_TIM_IRQHandler` → `HAL_IncTick()` | HAL 时基 |
| `EXTI4` / `EXTI9_5` | `HAL_GPIO_EXTI_IRQHandler(INT1_ACCEL / INT1_GYRO)` | **BMI088 数据就绪中断当前未接入任何回调**（`BMI088_BMI088DecodeData` 是 `Ins_Task` 里轮询的） |
| `DMA2_Stream5/6/7`, `DMA2_Stream1` | 各自 `HAL_DMA_IRQHandler` | 串口 / SPI 搬运 |

**反馈 → 全局变量的映射（一张表看懂"数据从哪来"）**

| 全局变量 | 生产者 | 消费者 |
|---|---|---|
| `Remote_RemoteData` | `Remote_DecodeRemoteData`（USART3 中断） | `Remote_ControlCom` 及全部 `Remote_*Process` |
| `BMI088_BMI088Data` | `BMI088_BMI088DecodeData`（`Ins_Task`，SPI 轮询） | `Ins_Task` |
| `INS` | `Ins_Task`（EKF 后） | 云台/底盘/视觉上报/自动瞄准 |
| `QEKF_INS` | `QuaternionEKF_Update` | `Ins_Task` |
| `Motor_*.encoder` | 各 `*_encoder_callback`（CAN 中断） | 各 `module_*.c` 的 PID 反馈 |
| `motor[Motor1].para` | `dm_motor_fbdata`（CAN1 中断） | `dm_motor_detect` |
| `Referee_RefereeData` | `Referee_ParseRefereeCmd`（USART6 中断） | `Shooter_*`、`Tidy_send_vision`、`Referee_Update` |
| `visionDataGet` | `GetVisionData`（USART1 中断） | `Remote_RemoteShooterModeSet` / `Remote_KeyMouseProcess` |
| `SuperCap` | `Parse_SuperCap_Feedback`（CAN1 中断，0x211） | `SuperCap_CheckPower` |
| `Protocol_Data` | 无（`Protocol_SetChaMode` 等无调用者） | `buscomm->yaw_ref` 被大量读写 |

---

## 6. 三条主控制链路（最重要的一节，务必记住）

### 链 1：遥控 → 底盘

```
USART3 DMA/IDLE ──▶ Remote_RXCallback ──▶ Remote_DecodeRemoteData ──▶ Remote_RemoteData
                                                                            │
                        Remote_Task (1ms) ──▶ Remote_ControlCom()           │
                                                  │                         │
                          按右拨杆 remote.s[0] 分派（★总开关）              │
          ┌───────────────────────┼───────────────────────┐                 │
     s[0]==UP               s[0]==MIDDLE            s[0]==DOWN             │
  Remote_RemoteProcess  Remote_RemoteShooterModeSet  Remote_KeyMouseProcess  │
  （纯遥控，无键鼠）     （键鼠+视觉混合）           + Remote_MouseShooterModeSet
          │                     │                    （全键鼠 + 自瞄）      │
          └─────────────────────┴────────────────────┘                     │
                                │                                          │
                   按左拨杆 remote.s[1] 选底盘模式                          │
        ┌───────────────────────┼───────────────────────┐                  │
   s[1]==UP                s[1]==MIDDLE           s[1]==DOWN               │
 Chassis_FOLLOW          Chassis_SEP            Chassis_XTL                │
 （跟随，Wz=编码器角）   （分离，Wz=拨杆）      （小陀螺，Wz 恒定 ±180）    │
        └───────────────────────┴───────────────────────┘                  │
                                │                                          │
        Chassis_SetChassisMode(mode) + Chassis_SetChassisRef(vx, vy, wz)   │
                                │                                          ▼
                                ▼                              Chassis_StatusData（四轮目标速度解算）
                        Chassis_Task (2ms)
                                │
                    ┌───────────┴───────────┐
             Chassis_Control()        Chassis_Output()
             （4× 速度环 PID）        （发 CAN1 0x200）
                    │
        Motor_SetMotorOutputChassic(&Motor_Chassis{4}Motor, pid_out)
```

**四轮解算公式**（`module_chassis.c:112 Chassis_SetChassisRef`，全向轮 / mecanum 逆解）：
```c
FrontRight = -Vx - Vy - Wz
FrontLeft  = +Vx - Vy - Wz
BackLeft   = +Vx + Vy - Wz
BackRight  = -Vx + Vy - Wz
```
`Chassis_XTL`（小陀螺）模式**不用这套公式**，而是用 `-Yaw_Rad` 做旋转矩阵投影，见 `module_chassis.c:159-178`。

### 链 2：遥控/视觉 → 云台（P 轴走达妙 MIT，Yaw 走 DJI 双环）

**Yaw 轴（RM6020，CAN1 0x1FF 组第 4 通道）**
```
buscomm->yaw_ref                ← Protocol_Data.yaw_ref，是唯一的"绝对角累加器"
   ├─ Remote_RemoteProcess:      += Gimbal_LimitYaw(ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN)
   ├─ Remote_RemoteShooterModeSet: += PID_GimbalYawVisionPID_Calc(visionDataGet.yaw_angle.yaw_predict)
   ├─ Remote_KeyMouseProcess:    += mouse.x * -MOUSE_YAW_ANGLE_TO_FACT  + autoaim_yaw
   └─ GimbalYaw_SetYawRef(buscomm->yaw_ref)
                        │
                 Gimbal_Task (2ms)
                        │
            GimbalYaw_Control()
              angPID: ref = yaw_ref,  fdb = INS.YawTotalAngle     → 输出作为速度环 ref
              spdPID: ref = angPID.out, fdb = INS.Gyro[Z_INS]     → 输出
              Motor_SetMotorOutput(&Motor_YawMotor, spdPID.out)
                        │
            GimbalYaw_Output()
              要求 P 轴与 Yaw 的 output_state 都为 1
              → Motor_SendMotorGroupOutput(&Motor_GimbalMotors)  → CAN1 0x1FF
                （D4/D5 = 拨弹 Motor_FeedMotor，D6/D7 = Yaw）
```

**Pitch 轴（达妙 DM4310/4340，MIT 模式，CAN1 0x001 发 / 0x000 收）**
```
GimbalPitch_SetPitchRef(inc)        ← ★这是【增量】接口，内部 += 后钳位到 [DMpitchrefMin, DMpitchrefMax]
                        │
                 Gimbal_Task (2ms)
                        │
            GimbalPitch_Control()
              ① dm_motor_init_test()      ← 每轮把 DM_KP/DM_KD 写回 ctrl
              ② imu_error = INS.Roll * π/180 + Const_PITCH_MOTOR_INIT_OFFSETf
              ③ pitch_ref_smooth += 0.1 * (pitch_ref - pitch_ref_smooth)
              ④ target_pos = pitch_ref_smooth - imu_error * IMUfdb
                 → ★★ IMUfdb 全局初值 0，所以 IMU 补偿项默认【完全关闭】★★
              ⑤ motor[Motor1].ctrl.pos_set = Gimbal_DMLimitPitch(target_pos)
                        │
            dm_motor_ctrl_send(&hcan1, &motor[Motor1])
              → mit_ctrl() → float_to_uint() 打包 8 字节 → CAN1 0x001
                        │
            dm_motor_detect(&motor[Motor1])   ← para.state != 1 就重发使能帧
```

### 链 3：视觉 → 自瞄（只在右拨杆 DOWN 时生效）

```
视觉上位机 ──USART1 115200──▶ DMA 双缓冲 ChariqotRecognition_data[2][30]
                                    │ IDLE 中断（代码内联在 stm32f4xx_it.c）
                              GetVisionData(&visionDataGet, buf)   要求恰好 15 字节
                                    │
                     visionDataGet.yaw_angle.yaw_predict   （int16，增量）
                     visionDataGet.pitch_angle.pitch_predict（int16，需 ×0.01×增益）
                                    │
                        Remote_KeyMouseProcess()  （mouse.r == 1 时）
                          autoaim_yaw   = PID_GimbalYawVisionPID_Calc(&Gimbal_YawVisionPID, yaw_predict)
                          autoaim_pitch = pitch_predict * 0.01f * 0.002f
                          buscomm->yaw_ref = 平滑后 yaw_ref + autoaim_yaw
                          pitch_ref       += autoaim_pitch
                                    │
                        Gimbal_Task 每轮
                          Tidy_send_vision(&visionDataSend)   ← 回填 INS.Yaw/Roll/Accel、robot_id、弹速
                          SendVisionData(&visionDataSend)     → USART1 TX（15 字节帧）
```

**视觉协议帧格式**

| 方向 | 长度 | 字段 |
|---|---|---|
| 下位机 → 视觉（发送） | 15 | `head(0x53) color mode robotID yawH yawL pitchH pitchL yawAccH yawAccL pitchAccH pitchAccL shoot_speed CRC end(0x45)` |
| 视觉 → 下位机（接收） | 15 | `head detect_signal shoot_msg yawH yawL pitchH pitchL xH xL yH yL depthH depthL CRC end` |

### 链 4（附）：发射

```
拨杆/鼠标 → Shooter_ChangeShooterMode(Shoot_NULL/FAST/SLOW/REFEREE)
          → Shooter_ChangeFeederMode(Feeder_NULL/SINGLE/FAST_CONTINUE/LOW_CONTINUE/
                                     VERY_FAST_CONTINUE/LOCKED_ROTOR/REFEREE/FINISH)
                        │
                 Shoot_Task (2ms)
                        │
        Shooter_Control() = Shooter_UpdataControlData()   ← 从裁判读 heat/limit
                          + Shooter_ShootControl()        ← 摩擦轮左右各一个 PID（速度环）
                          + Shooter_FeederControl()       ← 拨弹，按 mode 二选一：
                                 current_pid_num==1 → 只用速度环（连续拨弹）
                                 current_pid_num==2 → 角度环串速度环（单发/定位）
                        │
        Shooter_ShooterMotorOutput() → Motor_SendMotorGroupOutput(&Motor_ShootMotors) → CAN2 0x200
```

**摩擦轮与拨弹是同一帧还是不同帧？** 不同：摩擦轮在 **CAN2 0x200**（`Motor_ShootMotors`），拨弹在 **CAN1 0x1FF**（`Motor_GimbalMotors` 的第 3 通道）。

---

## 7. 逐层文件索引

### 7.1 L4 Application（`Src/Application/` + `Inc/Application/`）

| 文件 | 职责 | 关键入口 | 关键全局量 |
|---|---|---|---|
| `app_init.c` | 全部业务初始化的唯一入口 | `Init_InitAll()`、`Init_Task()` | — |
| **`app_remote.c`** | ★**全工程输入中枢**：拨杆/键鼠/视觉 → 各模块 SetRef | `Remote_ControlCom()`、`Remote_RemoteProcess()`、`Remote_RemoteShooterModeSet()`、`Remote_KeyMouseProcess()`、`Remote_MouseShooterModeSet()`、`Is_Key_Triggered()` | `Remote_remoteControlData`、`Gimbal_YawVisionPID`、`q_mode` |
| `app_ins.c` | 姿态解算任务；BMI088 → EKF → `INS`；四元数/欧拉角互转、机体系↔导航系变换 | `Ins_Task()`、`INS_Init()`、`BodyFrameToEarthFrame()`、`EarthFrameToBodyFrame()` | `INS`、`Param` |
| `app_autoaim.c` | 视觉串口协议收发、CRC16；历史遗留的自瞄解析代码 | `GetVisionData()`、`SendVisionData()`、`Tidy_send_vision()`、`Get_CRC16_CCITT()` | `visionDataGet`、`visionDataSend` |
| `app_gimbal.c` | 云台任务体（32 行） | `Gimbal_Task()` | — |
| `app_chassis.c` | 底盘任务体（18 行） | `Chassis_Task()` | — |
| `app_shoot.c` | 发射任务体（27 行） | `Shoot_Task()` | — |
| `app_referee.c` | 裁判 UI 刷新任务（16 行） | `Referee_Task()` | — |
| `app_communicate.c` | 遗留总线协议发送任务（21 行） | `Comm_Task()` | — |
| `app_watchDog.c` | 离线检测位图 | `WatchDog_Task()`、`WatchDog_Feed()` | `Dog_Bowl`（bit0=IMU, bit1=电机, bit2=遥控, bit3=协议） |
| `app_Debug.c` | VOFA+ JustFloat 上位机波形 + 在线改 PID | `Debug_Task()`、`PID_ChangePID()` | `PAKp/PAKd/PAKi`、`YSKp/YSKd/YSKi`… |

> **⚠ `app_Debug.c` 被编进工程但 `Debug_Task` 从未被创建**（`freertos.c` 里没有它），所以 VOFA 调参通道**当前不工作**。

### 7.2 L3 Modules（`Src/Modules/` + `Inc/Modules/`）★业务核心

| 文件 | 职责 | 关键函数 |
|---|---|---|
| `module_gimbal.c` | 云台 P/Y 控制器。P 轴只生成 MIT 位置目标；Yaw 轴角度+速度双环 PID | `GimbalPitch_Control()`、`GimbalYaw_Control()`、`GimbalYaw_Output()`、`GimbalPitch_SetPitchRef()`（**增量**）、`GimbalYaw_SetYawRef()`（**绝对**）、`Gimbal_DMLimitPitch()`、`Gimbal_LimitYaw()` |
| `module_chassis.c` | 底盘模式解算（5 种模式）+ 4 轮速度环 | `Chassis_SetChassisRef()`（核心解算）、`Chassis_SetChassisYawAngle()`、`Chassis_Control()`、`Chassis_Output()` |
| `module_shoot.c` | 发射状态机：摩擦轮速度环 + 拨弹速度环/位置环切换 + 热量控制 | `Shooter_Control()`、`Shooter_ShootControl()`、`Shooter_FeederControl()`、`Shooter_HeatCtrl()`、`Shooter_ChangeFeederMode()` |
| `module_dm4310.c` | 达妙驱动参数寄存器读取状态机（44 个寄存器轮询） | `read_all_motor_data()`、`receive_motor_data()`、`can1_rx_callback()`（后两者**已废弃**，见 §12） |
| `module_power.c` | 超级电容 CAN 协议 + 70 W 强制回拉看门狗 | `Parse_SuperCap_Feedback()`、`SuperCap_SetPower()`、`SuperCap_CheckPower()` |
| `module_referee.c` | **空文件（0 字节）** | — |

### 7.3 L2 Peripheral（`Src/Periphal/` + `Inc/Periphal/`）★硬件抽象

> 目录名拼写就是 `Periphal`（少一个 e），全局一致，不要以为是笔误去搜 `Peripheral`。

| 文件 | 职责 | 关键数据结构 / 函数 |
|---|---|---|
| **`periph_motor.c`** | ★DJI 电机对象框架 + **3 个电机组（4 通道一帧）** | `Motor_MotorTypeDef`、`Motor_MotorGroupTypeDef`；`Motor_InitAllMotors()`、`Motor_EncoderDecodeCallback()`（CAN 反馈分发）、`Motor_SendMotorGroupOutput()`、`{rm3508,rm2006,gm6020}_encoder_callback()` |
| **`periph_DMmotor.c`** | ★达妙 MIT 协议（**独立于上面那套电机框架**） | `DMmotor_t motor[num]`；`dm_motor_init()`、`dm_motor_enable/disable()`、`mit_ctrl()`、`dm_motor_fbdata()`、`dm_motor_ctrl_send()`、`dm_motor_detect()`、`float_to_uint()/uint_to_float()` |
| `periph_bmi088.c` | BMI088 六轴（SPI1，两片 CS 分开） | `BMI088_BMI088DataTypeDef`；`BMI088_Init()`、`BMI088_BMI088DecodeData()`、`BMI088_IsBMI088Offline()` |
| `periph_remote.c` | DBUS 18 字节帧解码（4 通道 + 2 拨杆 + 鼠标 + 键盘位图 + 滚轮） | `Remote_RemoteDataTypeDef`；`Remote_DecodeRemoteData()`、`Remote_DecodeKeyboardData()`、`Remote_RXCallback()` |
| `periph_referee.c` | 裁判系统 0xA5 帧解析（**表驱动**：`Const_Referee_CMD_NUM=20` 槽位，实际填充 9 条 cmd；另有 0x0301 交互数据通道） | `Const_Referee_CMD_LIST[]`；`Referee_ParseRefereeCmd()`、`Referee_SendRobotCustomData()`、`RobotClientTable[]`（机器人ID→clientID） |
| `periph_draw.c` | 裁判系统客户端 UI 绘制（图形缓冲、瞄准线、模式显示） | `Referee_Setup()`、`Referee_Update()`、`Draw_AddLine()`、`Draw_ClearAll()`、`Referee_DrawingBufferFlush()` |
| `periph_autoaim.c` | USART1 的 DMA 裸寄存器级初始化/启停（双缓冲 + IDLE） | `usart1_rx_dma_init()`、`usart1_tx_dma_init()`、`usart1_*_dma_enable()` |
| `periph_servo.c` | PWM 舵机抽象 | `Servo_ServoTypeDef`；`Servo_InitServo()`…（**从未初始化，见 §12**） |

### 7.4 L3' Protocol（`Src/Protocol/`）

表驱动的 CAN 双板通信协议，**是"平衡双腿轮式"时期的遗留**，当前只有 `yaw_ref` 累加器字段被真正使用。

| 文件 | 内容 |
|---|---|
| `protocol_common.c` | `Protocol_DataTypeDef Protocol_Data`；`Protocol_DecodeData()` 按 `cmd_id` 查 `ProtocolCmd_Receive[]`；`Protocol_SendProtocolData()` 用 `static i,j` 做两阶段轮转发送 |
| `protocol_transmit.c` | 发送表 `ProtocolCmd_Send[4]`：`0x205`(模式) `0x101`(IMU Yaw) `0x103`(底盘 ref) `0x105`(腿长 ref)。**全部发在 CAN2** |
| `protocol_receive.c` | 接收表 `ProtocolCmd_Receive[2]`：`0x203`(设状态) `0x204`(电容数据)。`0x204` 处理函数是空壳 |

`WheelLeg_BalanceModeEnum` / `WheelLeg_ChassisModeEnum` / `leg_len` / `dx` / `bal_mode` / `cha_mode` 都是遗留概念。

### 7.5 L1 Algorithm（`Src/Algorithm/` + `Inc/Algorithm/`）

| 文件 | 内容 | 关键点 |
|---|---|---|
| `alg_pid.c` | 位置式/增量式 PID + **误差限幅消积分饱和** + 微分低通 + 前馈 | `PID_PIDTypeDef` / `PID_PIDParamTypeDef`；`PID_InitPIDParam()` 的 13 个参数见 §10 |
| | | `PID_GimbalYawVisionPID_*`：专用于视觉增量的三阶误差预测 PID |
| `alg_kalman.c` | 通用线性卡尔曼（矩阵用 CMSIS-DSP `arm_matrix_instance_f32`） | `Kalman_KalmanTypeDef`；`Kalman_FilterUpdate()` |
| `alg_quaternionEKF.c` | **四元数 EKF**（陀螺零偏估计 + 卡方检验抗扰） | `QEKF_INS_t QEKF_INS`；`QuaternionEKF_Init(10, 0.01f, 1e7, 1, 0.0085f)`、`QuaternionEKF_Update()` |
| `alg_filter.c` | 一阶低通 / 滑动平均 / 二阶贝塞尔 | `Filter_LowPass()` 被 PID 内部使用 |
| `alg_math.c` | 角度↔弧度、斜率发生器、微分、`1/sqrt`、`fal/fsg/sign` | `LimitMax` / `LimitMaxMin` 是**宏**，直接改实参 |
| `alg_crc.c` | CRC8 / CRC16 查表 | 裁判系统帧校验用 `crc16_verify()` |
| `alg_power_limting.h` | **0 字节空文件** | `alg_power_limiting.c` 未加入 Keil 工程 → 整块死代码 |
| `alg_ahrs.h` | 只有声明，无对应 `.c` | 未使用 |

### 7.6 L0 System / Utility / Library

| 文件 | 职责 |
|---|---|
| `System/sys_const.c` | ★**全工程唯一的参数表**：19 组 `Const_*[4][5]` PID 参数表 + 遥控/鼠标灵敏度 + 限位 + 达妙增益（见 §10） |
| `System/sys_dwt.c` | DWT 周期计数器；`DWT_GetDeltaT()` 是所有 `dt` 的来源 |
| `System/sys_softTimer.c` | 一个 250 tick 的 `osTimer`，回调**为空** |
| `System/lqr.c` | LQR 控制器（348 字节头 + 672 字节源），**无任何调用者** |
| `Utility/util_can.c` | ★CAN 收发封装 + **`Can_RxMessageCallback()` 全局分发中枢** |
| `Utility/util_uart.c` | ★`Uart_RxIdleCallback()` 按句柄分派到遥控/裁判；DMA 模式启动 |
| `Utility/util_spi.c` | SPI1 单字节/多寄存器收发（BMI088 用） |
| `Utility/util_debug.c` | `usart_printf()`、`vofa_justfloat_send()`、`vofa_set_data()` |
| `Utility/util_flash.c` | STM32 内部 Flash 扇区擦写（BMI088 零偏持久化用） |
| `Utility/util_gpio.c` / `util_pwm.c` | GPIO 读写封装 / PWM 启动与占空比 |
| `Library/lib_buff.c` | `float2buff()` / `buff2float()` 等字节序转换（协议打包用） |
| `Library/lib_delay.c` | `Delay_us/ms`（DWT 忙等），**无调用者** |
| `etc/reg_BMI088.h` | BMI088 寄存器地址定义 |
| `etc/Code_specification.h` | 原框架的命名规范文档（**注释乱码，可略读**） |
| `struct_typedef.h` | `fp32`/`bool_t` 等自定义类型（**与 `stdint.h` 重复定义**） |

---

## 8. 硬件资源映射表

### 8.1 CAN（1 Mbps）

**CAN1**

| 方向 | StdId | 内容 | 成员 / 通道 |
|---|---|---|---|
| TX | `0x1FF` | `Motor_GimbalMotors`（云台+拨弹共享组） | D0-D1: 空(NULL)<br>D2-D3: 空(NULL)<br>D4-D5: `Motor_FeedMotor`（RM2006）<br>D6-D7: `Motor_YawMotor`（RM6020） |
| TX | `0x200` | `Motor_ChassisMotors`（底盘组） | D0-D1: FrontRight<br>D2-D3: FrontLeft<br>D4-D5: BackLeft<br>D6-D7: BackRight（均 RM3508） |
| TX | `0x001` | 达妙 MIT 控制帧（`id + MIT_MODE`） | 8 字节：pos16/vel12/kp12/kd12/tor12 |
| TX | `0x7FF` | 达妙参数寄存器读写 | `[idL, idH, 0x33/0x55/0xCC/0xAA, rid, ...]` |
| TX | `0x210` | 超级电容目标功率 | `[pwrH, pwrL, 0...]`，大端，单位 0.01 W |
| RX | `0x000` | 达妙 MIT 状态反馈 | `dm_motor_fbdata()`：D0 高 4 位状态（0失能/1使能/0xD超时）+ 低 4 位 ID |
| RX | `0x201`~`0x204` | 底盘 4 轮反馈 | 角度/转速/电流/温度 |
| RX | `0x207` | 拨弹反馈 | `rm2006_encoder_callback`（4096 线，速度 /36） |
| RX | `0x208` | Yaw 反馈 | `gm6020_encoder_callback`（8192 线） |
| RX | `0x211` | 超级电容反馈 | `Parse_SuperCap_Feedback`（4× uint16 /100） |

**CAN2**

| 方向 | StdId | 内容 | 成员 |
|---|---|---|---|
| TX | `0x200` | `Motor_ShootMotors`（摩擦轮组） | D0-D1: ShootRight(RX 0x201)<br>D2-D3: ShootLeft(RX 0x202)<br>D4-D7: 空 |
| TX | `0x205` `0x101` `0x103` `0x105` | 遗留总线协议（`Comm_Task` 1ms 发） | — |
| RX | `0x201` `0x202` | 摩擦轮反馈 | `rm3508_encoder_callback`（速度 /19） |

> **易混淆点**：同一 StdId `0x200` 在 CAN1 上是底盘组、在 CAN2 上是摩擦轮组。分发靠 `phcan` 句柄区分。

### 8.2 串口

| 外设 | 波特率 | 对端 | 接收机制 | 发送机制 |
|---|---|---|---|---|
| `USART1` | 115200 | **视觉 / 自瞄上位机** | DMA2_Stream5 双缓冲（2×30 B）+ IDLE 中断，代码内联在 `stm32f4xx_it.c` | `HAL_UART_Transmit(&huart1, ..., 15, 0xff)` 阻塞 |
| `USART3` | 100000 | **遥控器 DBUS** | DMA + IDLE → `Uart_RxIdleCallback` → `Remote_RXCallback` | 不发送 |
| `USART6` | 115200 | **裁判系统（RX）** / **VOFA 上位机（TX）** | DMA + IDLE → `Referee_RXCallback` | `usart_printf` / `vofa_justfloat_send` |

### 8.3 定时器 / SPI / GPIO

| 资源 | 用途 |
|---|---|
| `TIM6` | HAL 时基（`HAL_IncTick`），取代 SysTick |
| `TIM3` | 20 ms 周期中断 → `g_uart6_print_flag = 1` |
| `TIM1` / `TIM8` / `TIM12` | PWM 输出（舵机预留，**未接入**） |
| `SPI1` | BMI088（DMA2_Stream2 RX / Stream3 TX） |
| `PA4` / `PB0` | BMI088 CS：`CS1_ACCEL` / `CS1_GYRO`（宏名 `CS_ACCEL`/`CS_GYRO` 见 `periph_bmi088.h`） |
| `PC4` / `PC5` | `INT1_ACCEL`(EXTI4) / `INT1_GYRO`(EXTI9_5) |
| `PG3` / `PG6` | `DRDY_IST8310`(EXTI3) / `RSTN_IST8310` |
| `PC8` | `LASER` |

---

## 9. 关键数据结构速查

```
Remote_RemoteDataTypeDef  Remote_RemoteData        // 遥控原始量：ch[5] 拨杆 s[2] 鼠标 l/r 键盘位图
INS_INSTypeDef            INS                      // 姿态：q[4] Gyro[3] Accel[3] Roll/Pitch/Yaw/YawTotalAngle
Protocol_DataTypeDef      Protocol_Data            // ★ buscomm->yaw_ref 是云台全局角累加器
Chassis_StatusTypeDef     Chassis_StatusData       // 底盘目标速度 + 4 轮目标转速
Chassis_ChassisTypeDef    Chassis_ControlData[4]  // 4 个轮子的 PID 对象
GimbalPitch_...TypeDef    GimbalPitch_gimbalPitchControlData   // pitch_ref / pitch_ref_smooth / output_state
GimbalYaw_...TypeDef      GimbalYaw_gimbalYawControlData       // yaw_ref / angPID / spdPID
Shoot_StatusTypeDef       Shooter_ShooterControl  // 摩擦轮 + 拨弹 + 热量控制状态机
Motor_MotorTypeDef        Motor_YawMotor / Motor_FeedMotor / Motor_Shoot{Left,Right}Motor
                          / Motor_Chassis{FontRight,FontLeft,BackLeft,BackRight}Motor
                          // 共 9 个对象，其中 8 个在用；Motor_PitchMotor 已废弃(未初始化)
Motor_MotorGroupTypeDef   Motor_GimbalMotors(CAN1 0x1FF) / Motor_ChassisMotors(CAN1 0x200)
                          / Motor_ShootMotors(CAN2 0x200)       // ★只有 3 个组
DMmotor_t                 motor[num]  ==  motor[Motor1]        // ★num==1，只有 index 0
Referee_RefereeDataTypeDef Referee_RefereeData    // 裁判：status.shooter_limit / shoot_heat / robot_id
VisionDataGet_Typedef     visionDataGet           // 视觉下行：yaw_predict / pitch_predict / shoot_msg
VisionDataSend_Typedef    visionDataSend          // 视觉上行：IMU 角 + 弹速 + robotID
SuperCap_Data_t           SuperCap                // cap_voltage / target_power
BMI088_BMI088DataTypeDef  BMI088_BMI088Data       // accel / gyro / gyro_offset
QEKF_INS_t                QEKF_INS                // EKF 输出四元数与欧拉角
```

**`Motor_MotorTypeDef.encoder` 字段含义（反馈解码结果，最常被读错）**

| 字段 | 含义 |
|---|---|
| `angle` | 单圈原始编码值（RM3508/RM2006: 0~8191；RM6020: 0~8191） |
| `last_angle` | 上一帧单圈值（用于跨圈判断） |
| `round_count` | 跨圈计数 |
| `consequent_angle` | **连续累计角**（度）。RM3508: `round*360 + angle/8192*360`；RM2006: `round*10 + angle/8192*10`（**注意是 10 不是 360**） |
| `limited_angle` | 单圈限位角（仅 RM6020 更新，0~360°），用于跟随模式 |
| `speed` | 转速（RM3508: **/19**；RM2006: **/36**；RM6020: 原始值） |
| `current` | 实际转矩电流原始值 |
| `init_offset` | 回中偏置（当前**无任何代码设置它**，恒为 0） |

---

## 10. 调参入口地图（改行为先看这里）

| 想改什么 | 去哪 |
|---|---|
| **任何 PID 增益** | `Src/System/sys_const.c` 里的 `Const_*[4][5]` 表 |
| 云台 Yaw 增益 | `Const_GimbalYawAngParam` / `Const_GimbalYawSpdParam` |
| 云台 Pitch 增益 | `Const_GimbalPitchAngParam` / `Const_GimbalPitchSpdParam`（**注意：P 轴当前不用这两个 PID**） |
| 底盘 4 轮增益 | `Const_Chassis{FontRight,FontLeft,BackLeft,BackRight}{Spd,Ang}Param` |
| 摩擦轮 / 拨弹增益 | `Const_Shoot{Left,Right}Param` / `Const_Feed{Spd,Ang}Param` |
| **视觉增量环** | `Const_GimbalYawVision[5]` = `{Kp, Ki, Kd, inc_max, bias_deadband}` |
| 达妙 MIT 刚度/阻尼 | `DM_KP`(=130) / `DM_KD`(=1.5)。<br>⚠ 改 `motor[Motor1].ctrl.kp_set` 会被 `dm_motor_init_test()` 每轮覆盖 |
| 达妙映射范围 | `dm_motor_init()` 里的 `PMAX=12.5 / VMAX=30 / TMAX=10`（**必须与驱动器内一致**） |
| Pitch 软限位 | `Const_DMPITCH_UMAXANGLE`(0.8) / `Const_DMPITCH_DMAXANGLE`(-0.2) / `DMpitchrefMax`(1.08) / `DMpitchrefMin`(-0.4) |
| Pitch 装配零偏 | `Const_PITCH_MOTOR_INIT_OFFSETf`(=0.16) |
| **Pitch IMU 反馈开关** | `module_gimbal.c` 的 `float IMUfdb = 0.0;` ← **0 表示关闭 IMU 补偿**，改成 1 左右才启用 |
| 遥控杆量灵敏度 | `REMOTE_CHASSIS_VX_GAIN` / `VY_GAIN` / `SEP_WZ_GAIN` / `FOLLOW_WZ_GAIN` / `XTL_WZ_GAIN` |
| 鼠标灵敏度 | `MOUSE_PITCH_ANGLE_TO_FACT` / `MOUSE_YAW_ANGLE_TO_FACT` |
| 跟随模式零位 | `CHASSIS_YAW_ANGLE_OFFSET`(=170.62) |
| 小陀螺转速 | `CHASSIS_XTL_WZ`(=180) |
| Yaw 相对限幅 | `Const_YAW_MAXANGLE`(=40°) |
| 发射速度 | `Const_Shooter22mpers`(200) / `Const_Feeder{Slow,Fast,VeryFast,Wait}Speed` |
| 热量控制阈值 | `Const_HeatCtrl{Fast,Slow,Wait,Stop}Limit` |
| 视觉下行增益（遥控模式） | `Remote_RemoteProcess()` 里的 `0.01f*0.002f`；`Remote_RemoteShooterModeSet()` 里用 `VisionPitch`(=0.005) |
| 超级电容功率 | `app_init.c` 的 `SuperCap_SetPower(...,70)` + `module_power.c:SuperCap_CheckPower` 的 `70.0f` |
| 电机离线判定 | `periph_motor.c` 的 `Const_Motor_MOTOR_OFFLINE_TIME`(=200 ms) |
| 遥控离线判定 | `periph_remote.c` 的 `Const_Remote_REMOTE_OFFLINE_TIME`(=1000 ms) |
| 协议离线判定 | `protocol_common.c` 的 `Const_Protocol_OFFLINE_TIME`(=500 ms) |

**PID 参数表 `Const_XxxParam[4][5]` 的格式**（由 `PID_InitPIDParam` 的 13 个实参决定）：

```c
{ { kp,      ki,      kd,      sum_max, output_max },   // [0][0..4]
  { kd_fil,  delta_fil },                               // [1][0..1]  微分低通 / 参考低通
  { kf_1,    kf_2      },                               // [2][0..1]  前馈系数
  { kf1_fil, kf2_fil   } }                              // [3][0..1]  前馈低通
```
`-1` 表示该低通滤波系数**不启用**（`PID_InitPIDParam` 内部判断）。

---

## 11. 陷阱清单（★★ AI 最容易读错的地方，务必逐条核对 ★★）

### 11.1 框架级陷阱

1. **`freertos.c` 里的任务全是空壳。** 9 个任务函数均为 `__weak void Xxx_Task(void const *argument) { for(;;){ osDelay(1); } }`。真实实现在 `Src/Application/`。**永远不要在 `freertos.c` 里找业务逻辑。**
2. **`Motor_GimbalMotors` 这个名字是历史包袱。** 它是 CAN1 `0x1FF` 的 **Yaw + 拨弹共享组**，**不包含**达妙 Pitch。`Motor_PitchMotor` 对象存在但 `Motor_InitMotor` 那行被注释掉了 —— 是废弃对象。
3. **两套电机框架并存，数据结构完全不同：**
   - DJI 电机（RM3508/2006/6020）→ `Motor_MotorTypeDef` + `Motor_MotorGroupTypeDef`（`periph_motor.c`）
   - 达妙电机 → `DMmotor_t`（`periph_DMmotor.c`），**不参与任何 `Motor_*` 组**
   `periph_motor.c` 里的 `dm4340_*` 函数是给旧框架写的适配层，**当前不走这条路**。
4. **`motor[num]` 的 `num == 1`**（枚举 `{ Motor1, num }`），所以 `motor[Motor1]` 就是 `motor[0]`，全工程**只有一路达妙电机**。
5. **目录名是 `Periphal` 不是 `Peripheral`。** 搜索时别写错。

### 11.2 注释与代码不符（**一律以代码为准**）

6. **达妙 ID 三处说法互相矛盾：**
   - `periph_DMmotor.h` 注释：`"当前实物为 DM-J4340P-2EC，控制ID=6，反馈 Master ID=6"`
   - `util_can.c` 注释：`"CAN1/标准ID0x06分给达妙P轴"`
   - **代码事实**：`dm_motor_init()` 设 `id = 0x01`、`mst_id = 0x00`；`Can_RxMessageCallback` 判断 `StdId == 0x00`；`mit_ctrl` 发 `id + MIT_MODE(0x000)` = **`0x001`**。
   → **实际收发是 TX 0x001 / RX 0x000。**
7. `module_gimbal.h` 注释说 `spdPID`/`angPID` "P轴当前保留但未用于 MIT 目标生成" —— 正确，但**Yaw 轴确实在用**，同一个结构体字段名在两处含义不同。
8. `Chassis_StatusTypeDef` 里的 `Chassis_Yaw_Angle` / `Chassis_Yaw_Rad` 只有 `Chassis_XTL` 模式才会被 `Chassis_SetChassisYawAngle()` 更新，其它模式下是**上一模式的残留值**。

### 11.3 语义反直觉

9. **`Motor_IsMotorOffline()` 返回 1 表示"在线"（未超时）。** 与函数名完全相反。
10. **`Motor_IsAnyMotorOffline()` 也反着**：遍历时遇到一个"在线"就 `return 0`，只有**全部**在线才 `return 1`。调用方 `WatchDog_Feed()` 用它置 `Dog_Bowl` bit1，逻辑是反的。
11. **`Chassis_Control()` / `Chassis_Output()` 的 `for` 循环里用 `return` 而不是 `break`**：
    ```c
    for (int i = 0; i < 4; i++) { if (chassis[i].control_state != 1) return; }
    ```
    只要任一轮 `control_state != 1` 就**整体退出函数**（不是跳过该轮）。
12. **`SetPitchRef` 是增量、`SetYawRef` 是绝对。** 两个接口语义不一致：
    - `GimbalPitch_SetPitchRef(inc)` → `pitch_ref += inc` 后钳位
    - `GimbalYaw_SetYawRef(abs)` → 直接赋值
13. **`GimbalYaw_Output()` 需要 P 和 Yaw 的 `output_state` 同时为 1 才发帧**。而 `Remote_RemoteProcess` 在三个拨杆档位下都把两者置 1 → 实际总是发。

### 11.4 数值 / 单位陷阱

14. **`IMUfdb` 全局初值为 0**（`module_gimbal.c:173`）。`target_pos = pitch_ref_smooth - imu_error * IMUfdb` **= `pitch_ref_smooth`**。→ **Pitch 轴的 IMU 姿态补偿默认完全关闭**，P 轴是纯开环位置目标 + 达妙内环。这是理解"为什么 P 轴能稳"的关键。
15. **`float_to_uint()` 不做上下界钳位**（`periph_DMmotor.c:287`）。超范围值会打包出错值，不是饱和。调用方必须自己保证范围 —— `Gimbal_DMLimitPitch()` 只钳制位置，**速度/KP/KD/转矩都不钳**。
16. **`INS.Gyro` 的轴映射是交换的**：`Gyro[X] = gyro.pitch`、`Gyro[Y] = gyro.row`、`Gyro[Z] = gyro.yaw`。
17. **Pitch 控制用的反馈是 `INS.Roll`，不是 `INS.Pitch`**（`imu_error = ins->Roll * PI/180`）。因为机械上 Pitch 轴对应 IMU 的 Roll 角。
18. **`Remote_RemoteProcess` 里变量名叫 `cospitch` 但没做 `cos()`**，只是 `pitch_ref * PI/180`（度→弧度）。命名误导。
19. **三处视觉 pitch 增益不一致**：`Remote_RemoteShooterModeSet` 用 `*0.01f*VisionPitch`(0.005)；`Remote_RemoteProcess` UP 档用 `*0.01f*0.002f`；`Remote_KeyMouseProcess` 用 `*0.01f*0.002f`。
20. **`Const_FeederSlow/Fast/VeryFastSpeed` 三个常量当前全部等于 `-70.0f`**，所以三档拨弹速度实际相同。

### 11.5 已确认的缺陷（改代码时会踩到）

21. **`Remote_KeyMouseProcess` / `Remote_RemoteShooterModeSet` 每轮都重新 `PID_GimbalYawVisionPID_Init()`** → 视觉增量 PID 的历史误差每 1 ms 被清零，微分/积分项永远从 0 开始（调这三个增益时行为会与常规 PID 直觉不符）。
22. **`SendVisionData()` 数组越界**：`uint8_t data_buffer[14]`（下标 0~13）却调用 `HAL_UART_Transmit(&huart1, data_buffer, 15, 0xff)` → 多读 1 字节栈内存。
    （顺带澄清：`delay_num == 10` 的门限是**对的**——命中后函数内部 `delay_num = 0`，所以确实是每 10 次调用发一帧 ≈ 50 Hz，不是"1/255"。）
23. **`USART1_IRQHandler` 双缓冲两个分支都传 `ChariqotRecognition_data[0]`** → Memory 1 分支拿到的是旧缓冲，双缓冲形同虚设。
24. **`Shooter_ChangeShooterMode()` 里的裁判功率门槛被注释掉了** → 无裁判数据也能直接开摩擦轮。
25. **`Shooter_FeederMotorLockedJudge()`（卡弹检测）的调用被注释掉了**（`module_shoot.c:187`）→ `Feeder_LOCKED_ROTOR` 分支永远不会被自动触发。
26. **`Const_ShooterLockedReverseSpeed = 0.0f`** → 即使进入 `Feeder_LOCKED_ROTOR`，反转速度也是 0，卡弹处理等于停机而非退弹。
27. **`Protocol_SendProtocolData()` 的 `static i, j` 状态机**：每次只发 2 帧然后 `break`，靠下次调用续发；`i` 靠 `if (i >= 3) { i = 0; j = 0; }` 复位。可运行但可读性极差，且占满 CAN2 发送邮箱（与摩擦轮组 `0x200` 竞争）。
28. **`Can_ErrorHandler()` 是空操作**：`while (1) { return; }` —— 立即返回，不打印不重试。**CAN 发送失败会被静默吞掉**，现场表现为"电机不动但不报错"。
29. **`Referee_ParseRefereeCmd()` 里的数据长度校验被注释掉了**（`periph_referee.c:329`）→ 只看 `cmd_id` 不看 `data_length`，格式错误的帧也会被强行解析。
30. **`Protocol_SetChaMode()` / `Protocol_SetBalMode()` / `Protocol_SetYawState()` 全部无调用者** → `buscomm->cha_mode` 恒为 `Cha_NULL` → `Gimbal_LimitYaw()` 里的 `Cha_Gyro` 快速返回分支**永远进不去** → **Yaw 限幅（`Const_YAW_MAXANGLE`）始终生效**。
31. **`Gimbal_LimitYaw()` 只被 `Remote_RemoteProcess` 的 UP/MID 两档调用**，DOWN 档与全键鼠模式直接 `buscomm->yaw_ref += ...`，**绕过限幅**。

### 11.6 死字段（写入了，但控制路径从不读）

| 字段 | 实际情况 |
|---|---|
| `Chassis_ChassisTypeDef.chassis_ref` | 仅 `Chassis_InitChassis` 置 0，控制用 `chassis_status->Chassis_*_Speed` |
| `Chassis_ChassisTypeDef.angPID` / `AccelPID` | 参数有初始化，`Chassis_Control()` 里计算全被注释 → **只有 `spdPID` 在跑** |
| `GimbalYaw_*.yaw_position_fdb` / `yaw_speed_fdb` | 有 setter（`GimbalYaw_SetIMUYawPositionFdb/SpeedFdb`），但 `GimbalYaw_Control()` 直接读 `INS` |
| `Protocol_Data.leg_len` / `dx` / `bal_mode` / `bal_real_mode` / `cha_mode` | 遗留平衡底盘字段，仅有 `yaw_ref` 被真正使用 |
| `Motor_MotorTypeDef.fdb_fil` / `fdb_fil_param` | 初始化了低通，但**没有任何回调实际调用 `Filter_LowPass`** |
| `Motor_*.encoder.init_offset` | 恒为 0（无人赋值），但 `limited_angle` 的跨 180° 修正逻辑依赖它 |
| `SoftTimerCallback` | 空函数 |
| `Shooter_SingleShootCtrl()` 里的 `if (fabs(...) > 1.0f)` | 判断体被注释，只剩空 `if` |

### 11.7 编码问题

32. **32 个 `.c/.h` 是 GBK 编码**，`read` 工具直读报 `invalid UTF-8`。涉及：`alg_pid`、`app_autoaim/app_Debug/app_gimbal/app_init/app_ins/app_remote/app_shoot`、`module_dm4310/gimbal/power/shoot`、`periph_autoaim/DMmotor/draw/motor/referee`、`lqr`、`sys_const`、`util_can/util_debug` 及对应 `.h`。Keil 中需设 `Chinese GB2312`。
33. 部分 UTF-8 文件内本身含 `锟斤拷` 乱码（历史二次转码损坏），如 `Code_specification.h`、`sys_const.c` 的部分中文注释。**不影响逻辑**。

---

## 12. 死代码 / 未接入代码清单

> 用这份清单可以避免"读到一堆看起来很关键但其实不执行的代码"。

| 代码 | 状态 | 证据 |
|---|---|---|
| `alg_power_limiting.c`（`Power_limiting_Init`、`Power_Calculate`、`Chassic_Power_Control`、`Tyres_Power_Cal`） | **未加入 Keil 工程** | `gimbal_task.uvprojx` 文件列表中没有它；且 `alg_power_limting.h` 是 0 字节，无声明 |
| `lqr.c`（`mat_init`、`lqr_set_desire`） | **无调用者** | 全仓仅定义处命中 |
| `periph_servo.c` 全部 | **从未初始化** | `Init_InitAll()` 不调用 `Servo_InitAllServos()` |
| `lib_delay.c`（`Delay_us`/`Delay_ms`） | **无调用者** | DWT 已提供 `DWT_Delay` |
| `app_Debug.c` 的 `Debug_Task()` | **任务未创建** | `freertos.c` 里没有对应 `osThreadDef`；VOFA 调参通道不工作 |
| `module_dm4310.c` 的 `can1_rx_callback()` | **无调用者** | `Can_RxMessageCallback` 直接调 `dm_motor_fbdata()` + `receive_motor_data()` |
| `module_dm4310.c` 的 `read_all_motor_data()` | **无调用者** | 只被自身注释引用 |
| `alg_ahrs.h` | 只有声明，无 `.c` | — |
| `Motor_SendMotorGroupsOutput()` | **无调用者** | 各 module 各自调单组发送 |
| `Motor_PitchMotor` | 废弃对象 | `Motor_InitMotor(&Motor_PitchMotor, ...)` 被注释 |
| `dm4340_motor_send_output()` / `dm4340_motor_init()` | 旧适配层，无调用者 | 地址写死 0x01，与当前 P 轴无关 |
| `Shooter_FeederMotorLockedJudge()` | **调用被注释** | `module_shoot.c:187` |
| `app_autoaim.c` 的 `shoot_speed_sending_culculate()`、`get_shoot_msg()`、`CRringBuffer` 相关 | **无调用者** | 历史自瞄预测代码残留 |
| `Protocol_Cmd_Receive[0x204]` 的 `_send_cap_data()` | 空函数体 | 只 `DWT_GetDeltaT` |
| `freertos.c` 的 `SoftTimerCallback` | 空 | — |
| `MDK-ARM/gc_chass/`（整个目录） | **另一套废弃工程** | 不在 uvprojx 编译列表；用 MPU9250 + `my_ekf.c` |

---

## 13. 场景索引：想改 X 该看哪里

| 目标 | 必读文件（按顺序） |
|---|---|
| **搞懂整体怎么跑起来** | `main.c` → `app_init.c:Init_InitAll` → `freertos.c:MX_FREERTOS_Init` → §4 任务表 |
| **改遥控手感 / 拨杆功能** | `periph_remote.c`（解码）→ `app_remote.c:Remote_ControlCom`（分派）→ `sys_const.c`（增益） |
| **改底盘运动 / 加轮子 / 换全向方案** | `module_chassis.c:Chassis_SetChassisRef`（解算）→ `Chassis_Control`（PID）→ `periph_motor.c:Motor_InitAllMotors`（组映射） |
| **改云台 Yaw 响应** | `module_gimbal.c:GimbalYaw_Control` → `sys_const.c:Const_GimbalYaw{Ang,Spd}Param` |
| **改云台 Pitch（达妙）** | `module_gimbal.c:GimbalPitch_Control` → `periph_DMmotor.c:mit_ctrl` → `DM_KP/DM_KD`、`PMAX/VMAX/TMAX` |
| **调自瞄跟枪** | `app_autoaim.c:GetVisionData`（协议）→ `app_remote.c:Remote_KeyMouseProcess`（叠加）→ `alg_pid.c:PID_GimbalYawVisionPID_Calc` → `Const_GimbalYawVision` |
| **加/改视觉协议** | `Inc/Application/app_autoaim.h`（结构体）→ `app_autoaim.c:GetVisionData`/`Tidy_send_vision`/`SendVisionData` → `stm32f4xx_it.c:USART1_IRQHandler` |
| **改发射 / 卡弹 / 热量控制** | `module_shoot.c:Shooter_FeederControl`（状态机）+ `Shooter_HeatCtrl` → `sys_const.c:Const_HeatCtrl*`、`Const_Feeder*Speed` |
| **加一个电机** | ①`periph_motor.c:Motor_InitAllMotors` 注册对象与组通道位置 ②`sys_const.c` 加 PID 表 ③对应 `module_*.c` 加 PID 成员与 `PID_InitPIDParam` |
| **换 IMU / 改姿态解算** | `periph_bmi088.c` → `app_ins.c:Ins_Task` → `alg_quaternionEKF.c:QuaternionEKF_Update` |
| **改裁判系统 UI** | `periph_draw.c:Referee_Setup`/`Referee_Update`/`Draw_AddLine` → `periph_referee.c:Referee_SendRobotCustomData` |
| **加裁判数据字段** | `periph_referee.c:Const_Referee_CMD_LIST[]` + 一个 `P_ext_xxx()` 解析函数 + `Referee_RefereeDataTypeDef` 加字段 |
| **改电容功率** | `module_power.c:SuperCap_SetPower`/`SuperCap_CheckPower` → `app_init.c` 的 70 |
| **加离线保护** | `app_watchDog.c:WatchDog_Feed`（汇总 `Dog_Bowl`）+ 各 `*_IsOffline()` |
| **CAN 通信不上** | `util_can.c:Can_InitFilterAndStart`（滤波/启动）→ `Can_RxMessageCallback`（分发）→ `Motor_InitAllMotors`（StdId 对照） |
| **排查任务卡死** | `freertos.c`（创建/优先级）→ `Inc/FreeRTOSConfig.h`（堆、时间片）→ 各任务 `osDelay` |

---

## 14. 建议阅读顺序（给 AI 的施工单）

**第 1 轮（20 分钟，建立骨架）**
1. `Src/main.c` — 看 `Init_InitAll()` 被调在哪一步
2. `Src/Application/app_init.c` — ★初始化全清单，一眼看全部子系统
3. `Src/freertos.c` — 任务名/优先级/栈；**记住它们是空壳**
4. `Inc/System/sys_const.h` + `Src/System/sys_const.c` 前 120 行 — 全局参数池
5. 本文件 §4、§5、§6、§8

**第 2 轮（40 分钟，打通输入）**
6. `Src/Application/app_remote.c` — ★**全工程最核心的 508 行**，所有模式分派都在这
7. `Src/Periphal/periph_remote.c` — DBUS 解码
8. `Inc/Periphal/periph_remote.h` — 数据结构

**第 3 轮（40 分钟，打通三条输出）**
9. `Src/Modules/module_chassis.c` + `.h`
10. `Src/Modules/module_gimbal.c` + `.h`
11. `Src/Modules/module_shoot.c` + `.h`
12. `Src/Periphal/periph_motor.c` — 组/通道/反馈解码/发送
13. `Src/Periphal/periph_DMmotor.c` — MIT 打包

**第 4 轮（30 分钟，打通反馈）**
14. `Src/Application/app_ins.c` + `Inc/Application/app_ins.h`
15. `Src/Utility/util_can.c` — CAN 分发中枢
16. `Src/Algorithm/alg_pid.c` — PID 的 13 个参数含义
17. `Src/Algorithm/alg_quaternionEKF.c`（可跳过内部矩阵运算，只看 `Update` 的输入输出）

**第 5 轮（按需）**
18. `Src/Application/app_autoaim.c` + `stm32f4xx_it.c:USART1_IRQHandler` — 视觉链
19. `Src/Periphal/periph_referee.c` — 裁判系统
20. `Src/Periphal/periph_draw.c` — UI

**不用读**：`Drivers/**`、`Middlewares/**`、`MDK-ARM/gc_chass/**`、`Inc/stm32f4xx_hal_conf.h`（除需确认某个 HAL 模块是否开启时）。

---

## 附录 A：把 GBK 文件转成 UTF-8（供 AI 工具处理）

如需让只接受 UTF-8 的工具读取全部源码，可在仓库外生成一份转码镜像：

```powershell
$root = (Get-Location).Path
$out  = Join-Path $env:TEMP 'djomni_utf8'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$gbk  = [System.Text.Encoding]::GetEncoding(936)
$utf8 = New-Object System.Text.UTF8Encoding($false, $true)

foreach ($f in Get-ChildItem -Path "$root\Src", "$root\Inc" -Recurse -File -Include *.c,*.h) {
    $dest = Join-Path $out $f.FullName.Substring($root.Length + 1)
    New-Item -ItemType Directory -Force -Path (Split-Path $dest -Parent) | Out-Null
    $bytes = [System.IO.File]::ReadAllBytes($f.FullName)
    try   { $text = $utf8.GetString($bytes) }      # 本来就是 UTF-8
    catch { $text = $gbk.GetString($bytes)  }      # 否则按 GBK 解
    [System.IO.File]::WriteAllText($dest, $text, (New-Object System.Text.UTF8Encoding($false)))
}
```

## 附录 B：关键宏与约定

| 名字 | 含义 |
|---|---|
| `forever` | `while(1)`，定义在 `Inc/main.h` |
| `X_INS` / `Y_INS` / `Z_INS` | 0 / 1 / 2，`INS.Gyro[]`、`INS.Accel[]` 的索引 |
| `num` | `motor_num` 枚举的哨兵值，`== 1`，表示只有一路达妙 |
| `MIT_MODE` / `POS_MODE` / `SPD_MODE` / `PSI_MODE` | 达妙 CAN ID 偏移：`0x000` / `0x100` / `0x200` / `0x300`（**与 `mode_e` 枚举 `mit_mode=1` 不是一回事**） |
| `PID_POSITION` / `PID_DELTA` / `PID_Accel` | PID 模式：位置式 / 增量式 / 加速度式 |
| `Filter_LowPassInit(param, ...)` 的 `param` | 低通系数，**`-1` 表示不启用该滤波** |

---

*本文档由静态代码阅读生成，未修改任何源码。所述"陷阱"与"死代码"均经调用点交叉验证。*
