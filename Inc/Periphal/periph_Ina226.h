/*
 * 文件：periph_Ina226.h
 * 描述：森林功率计 V1.4（ESP32-C3 + INA226）CAN 接收端。
 *       协议 1.4.0，1 Mbps 标准帧，全部字段小端。
 *
 * ---------------------------- 调试怎么用 ----------------------------
 * 上电后在 Keil Watch / Live Watch 里添加 Ina226_PowerMeter，展开后按下面顺序看：
 *
 *   1. pwr_frame_cnt == 0
 *        -> 一帧 0x2A0 都没收到。查接线（GH1.25-2P 是 pin1=CANL / pin2=CANH，
 *           和常规相反）、波特率是否 1Mbps、终端电阻（CANH-CANL 应为约 60 欧）。
 *   2. pwr_frame_cnt > 0 但 rx_err_cnt 同时在涨
 *        -> 帧到了但 DLC 不是 8，说明总线上有别的东西撞了这个 ID。
 *   3. pwr_frame_cnt > 0、is_online == 1、rx_err_cnt 不动
 *        -> 链路通。再核对 voltage_v 是否和万用表量到的接近（未校准约 +-2%）。
 *   4. state == Ina226_State_SensorFault（同时 raw_flags 有 INA_FAULT / CLIP）
 *        -> 帧收到了，但功率计自己说测不准。这是传感器问题，不是你接收的问题。
 *   5. last_pwr_data[0..7]
 *        -> 最近一帧 8 个原始字节，可以直接和协议文档里的示例帧
 *           "68 05 32 02 9F 09 1E 22" 对照，肉眼验一遍小端和缩放。
 *   6. 手边没有硬件时，调一次 Ina226_RunSelfTest()，然后看 Ina226_SelfTest：
 *        pass == 1 说明"字节序 + 缩放"这套解析逻辑本身是对的，问题一定在硬件 / 接线。
 */

#ifndef INA226_PERIPH_H
#define INA226_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================================================================
 *  编译期配置
 * ================================================================== */

/* 三个数据帧的长度恒为 8，DLC 不符直接丢弃（不检查 DLC 会读到旧 buffer 的垃圾） */
#define INA226_FRAME_LEN                (8u)

/* 帧 ID：森林功率计 V1.4 */
#define INA226_ID_PWR                   (0x2A0u)    /* PM->C  500Hz(2ms)  功率/电流/电压/限幅系数 */
#define INA226_ID_STATUS                (0x2A1u)    /* PM->C  50Hz(20ms)  flags/mode/seq/原始功率/目标 */
#define INA226_ID_EVENT                 (0x2A2u)    /* PM->C  事件帧（突发，正常时长时间为 0） */

/* 接收超时（ms），协议 §8.3 建议值 */
#define INA226_TIMEOUT_PWR_MS           (20u)       /* 0x2A0 超过 20ms 没来 -> 判定失联 */
#define INA226_TIMEOUT_STATUS_MS        (150u)      /* 0x2A1 超过 150ms 没来 -> 判定失联 */

/* 失联 / 传感器故障时的保守限力系数，与功率计固件 k_safe 一致 */
#define INA226_K_SAFE_LOST              (0.85f)

/* ---------------- FLAGS 位定义（0x2A1 字节 0~1） ---------------- */
#define INA226_F_OVER_SOFT              (1u << 0)   /* bit0  功率超软阈值            参考，无动作 */
#define INA226_F_OVER_HARD              (1u << 1)   /* bit1  功率超硬阈值            立刻降力 */
#define INA226_F_ALERT_POL              (1u << 2)   /* bit2  INA226 硬件 ALERT 触发  最高优先级降力 */
#define INA226_F_INA_FAULT              (1u << 3)   /* bit3  与 INA226 通信失败      数据不可信 */
#define INA226_F_CAN_FAULT              (1u << 4)   /* bit4  功率计侧 CAN 错误       准备数据中断 */
#define INA226_F_Q5_ON                  (1u << 5)   /* bit5  高边总开关导通          0 = 底盘已断电 */
#define INA226_F_CLIP                   (1u << 6)   /* bit6  电流接近 40.96A 量程     读数已偏低，别信 */
#define INA226_F_V_FAULT                (1u << 7)   /* bit7  母线电压越界            降力 + 报警 */
#define INA226_F_MOS_LEAK               (1u << 8)   /* bit8  关断后仍有 >3A          紧急断电检查 */
#define INA226_F_CUT_DONE               (1u << 9)   /* bit9  保护已切断总开关(锁存)  底盘失电 */
#define INA226_F_BLANK                  (1u << 10)  /* bit10 开通盲区(400ms)         正常，但保护未生效 */
#define INA226_F_PEER_OK                (1u << 11)  /* bit11 功率计收到了我的心跳    0 = 我的心跳没发出去 */
#define INA226_F_ENERGY_LOW             (1u << 12)  /* bit12 能量缓冲耗尽(仅BUFFER)  参考 */
#define INA226_F_LIMITING               (1u << 13)  /* bit13 正在限幅(k<0.999)       正常，闭环在工作 */
#define INA226_F_SHUNT_HOT              (1u << 14)  /* bit14 分流器热估算超 2.5W     长时间降力 */
#define INA226_F_ZERO_SET               (1u << 15)  /* bit15 已做过零点校准          0 = 读数误差约 +-2% */

/* ---------------- 事件码（0x2A2 字节 0） ---------------- */
#define INA226_EV_OVER_HARD             (1u)        /* 超硬阈值      EV_VAL = 原始功率 W */
#define INA226_EV_ALERT                 (2u)        /* ALERT 触发    EV_VAL = 原始功率 W */
#define INA226_EV_CLIP                  (3u)        /* 量程削顶      EV_VAL = 原始电流 A */
#define INA226_EV_INA_FAULT             (4u)        /* INA226 掉线   EV_VAL = 0 */
#define INA226_EV_CAN_BUSOFF            (5u)        /* CAN bus-off   EV_VAL = 累计次数 */
#define INA226_EV_MOS_LEAK              (6u)        /* MOS 漏电      EV_VAL = 原始电流 A */
#define INA226_EV_CUT                   (7u)        /* 已切断 Q5     EV_VAL = 触发时功率 W */
#define INA226_EV_V_FAULT               (8u)        /* 母线越界      EV_VAL = 母线电压 V */
#define INA226_EV_ENERGY_LOW            (9u)        /* 能量耗尽      EV_VAL = 剩余能量 J */
#define INA226_EV_SHUNT_HOT             (10u)       /* 分流器过热    EV_VAL = 热估算功率 W */

/* ==================================================================
 *  类型
 * ================================================================== */

typedef enum
{
    Ina226_State_OK          = 0,   /* 数据正常，直接用帧里的 k */
    Ina226_State_WARN        = 1,   /* 收到数据但硬件/软件告警，额外压到 0.8 */
    Ina226_State_SensorFault = 2,   /* 功率计自己说测不准（INA_FAULT / CLIP） */
    Ina226_State_Lost        = 3,   /* 通信超时 */
    Ina226_State_Cut         = 4,   /* 功率计已切断总开关（CUT_DONE / 事件码 7） */
    Ina226_State_NotInit     = 5    /* 未初始化 */
} Ina226_StateEnum;

/*
 * 功率计设备对象 —— 全局命名实例，调试器里直接 Watch Ina226_PowerMeter。
 * 字段分组顺序：静态配置 / 原始观测 / 工程值 / 新鲜度统计 / 判据缓存 / 原始字节。
 * 原始值和工程值都保留：原始值能和协议文档、万用表逐位对照，工程值能直接用。
 */
typedef struct
{
    /* ---------- 静态配置（Ina226_Init 填，运行期只读） ---------- */
    uint8_t  enabled;                       /* 1 = 参与解析；运行中在 Watch 里改成 0 可停掉本模块 */
    uint8_t  init;                          /* 1 = 已初始化 */

    /* ---------- 0x2A0 原始值：与协议字段一一对应，单位写在名字后缀上 ---------- */
    uint16_t raw_p_0p1w;                    /* d[0..1] uint16 LE  x0.1    W         */
    int16_t  raw_i_0p01a;                   /* d[2..3] int16  LE  x0.01   A（有符号）*/
    uint16_t raw_v_0p01v;                   /* d[4..5] uint16 LE  x0.01   V         */
    uint16_t raw_k_0p0001;                  /* d[6..7] uint16 LE  x0.0001 无量纲    */

    /* ---------- 0x2A0 工程值 ---------- */
    float    power_w;                       /* 滤波后功率，闭环控制用这个 */
    float    current_a;                     /* 滤波后电流，负值 = 刹车回馈，不要钳到 0 */
    float    voltage_v;                     /* 母线电压 */
    float    k;                             /* 功率计建议的限力系数 0.0 ~ 1.0 */

    /* ---------- 0x2A1 原始值 ---------- */
    uint16_t raw_flags;                     /* d[0..1] uint16 LE 状态位，见 INA226_F_* */
    uint8_t  raw_mode;                      /* d[2]  0=METER 1=PI 2=RATIO 3=BUFFER 4=MANUAL */
    uint8_t  raw_seq;                       /* d[3]  每帧 +1，255->0，用来推算丢帧 */
    uint16_t raw_p_raw_0p1w;                /* d[4..5] uint16 LE 未滤波功率 x0.1 W（保护判断用） */
    uint16_t raw_target_0p1w;               /* d[6..7] uint16 LE 当前有效目标功率 x0.1 W */

    /* ---------- 0x2A1 工程值 ---------- */
    float    power_raw_w;                   /* 未滤波功率，尖峰在这里看得见 */
    float    target_w;                      /* 当前有效目标功率（已含 电压x电流上限 的收紧） */

    /* ---------- 0x2A2 事件帧（只保留最近一次） ---------- */
    uint8_t  ev_code;                       /* d[0]    事件码，见 INA226_EV_* */
    int16_t  raw_ev_val_0p1;                /* d[1..2] int16 LE 关联量 x0.1 */
    float    ev_val;                        /* 关联量工程值（物理单位随事件码而变） */
    uint32_t ev_t_ms;                       /* d[3..6] uint32 LE 功率计自己的开机毫秒数 */
    uint8_t  ev_flags;                      /* d[7]    FLAGS 低 8 位快照 */

    /* ---------- 新鲜度 / 统计：分不清"值是 0"和"根本没收到"就靠这几项 ---------- */
    volatile uint8_t  is_online;            /* ★ 1 = 通信正常（由 Ina226_Update 刷新） */
    volatile uint32_t last_update_time;     /* 最近一次收到任意有效帧的 HAL_GetTick() */
    volatile uint32_t last_pwr_time;        /* 最近一次收到 0x2A0 */
    volatile uint32_t last_status_time;     /* 最近一次收到 0x2A1 */
    volatile uint32_t last_event_time;      /* 最近一次收到 0x2A2 */
    volatile uint32_t rx_ok_cnt;            /* 累计成功解析帧数（三帧合计） */
    volatile uint32_t rx_err_cnt;           /* 累计错误：非标准帧 / DLC 不符 */
    uint32_t pwr_frame_cnt;                 /* 0x2A0 帧计数 */
    uint32_t status_frame_cnt;              /* 0x2A1 帧计数 */
    uint32_t event_frame_cnt;               /* 0x2A2 帧计数 */
    uint16_t seq_lost_cnt;                  /* 按 SEQ 推算出的累计丢帧数 */
    uint8_t  last_seq;                      /* 上一帧 SEQ，便于人工核对 */
    uint8_t  seq_valid;                     /* 1 = 丢帧统计已开始（收到过至少一帧 0x2A1） */

    /* ---------- 判据缓存：只在 Ina226_Update() 里写，Watch 直接看 ---------- */
    Ina226_StateEnum state;                 /* 状态分级，Keil 里直接显示枚举名 */
    float    k_safe;                        /* 状态机给出的建议 k（本模块只算，不执行） */

    /* ---------- 最后一帧来源 + 原始字节：排查接线 / 对照协议文档 ---------- */
    uint32_t last_stdid;                    /* 最近一帧 StdId */
    uint8_t  last_dlc;                      /* 最近一帧 DLC */
    uint8_t  last_can_port;                 /* 1 = hcan1，2 = hcan2，0 = 未知 */
    uint8_t  last_pwr_data[INA226_FRAME_LEN];       /* 最近一帧 0x2A0 的 8 个原始字节 */
    uint8_t  last_status_data[INA226_FRAME_LEN];    /* 最近一帧 0x2A1 的 8 个原始字节 */
    uint8_t  last_event_data[INA226_FRAME_LEN];     /* 最近一帧 0x2A2 的 8 个原始字节 */
} Ina226_DeviceTypeDef;

/*
 * 解析自检结果 —— 独立的一个全局命名实例。
 * 调用 Ina226_RunSelfTest() 后 Watch 它：pass == 1 说明"字节序 + 缩放"是对的。
 * 这样在没有任何硬件的情况下，也能把"软件解析错"和"硬件没接好"区分开。
 */
typedef struct
{
    uint8_t  pass;                          /* 1 = 全部用例通过 */
    uint8_t  fail_step;                     /* 第一个失败的用例编号，0 = 没有失败 */
    float    p_w;                           /* 用例1：应 138.4  */
    float    i_a;                           /* 用例1：应 5.62   */
    float    v_v;                           /* 用例1：应 24.63  */
    float    k;                             /* 用例1：应 0.8734 */
    float    neg_i_a;                       /* 用例2：应 -12.34 */
    float    raw_p_w;                       /* 用例3：未滤波功率 应 150.0 */
    uint32_t run_cnt;                       /* 已运行次数 */
    uint32_t fail_cnt;                      /* 累计失败次数 */
} Ina226_SelfTestTypeDef;

/* ==================================================================
 *  全局对象（调试器里 Watch 这两个）
 * ================================================================== */
extern Ina226_DeviceTypeDef   Ina226_PowerMeter;
extern Ina226_SelfTestTypeDef Ina226_SelfTest;

/* ==================================================================
 *  API —— 一律传对象指针
 * ================================================================== */

/** 初始化（在 Init_InitAll 里调用一次，要在 CAN 启动之后） */
void                  Ina226_Init(void);

/** 取设备对象指针 */
Ina226_DeviceTypeDef* Ina226_GetDevicePtr(void);

/**
 * CAN 接收入口，在 Can_RxMessageCallback 里调用。
 * 只认 0x2A0 / 0x2A1 / 0x2A2 且只认标准帧，其余返回 0 表示"不是我的帧，请继续分发给别人"。
 * 本函数在中断上下文执行，只做匹配 + 拷贝 + 置标志，不做阻塞操作。
 * @retval 1 = 这帧是本设备的（已消费）；0 = 不是本设备的
 */
uint8_t               Ina226_OnCanFrame(CAN_HandleTypeDef* phcan,
                                        const CAN_RxHeaderTypeDef* pheader,
                                        const uint8_t* rxdata);

/**
 * 周期刷新判据。建议在 10~20ms 的任务里调用（例如 Chassis_Task）。
 * 只做超时判定 + 状态分级 + 算 k_safe，不动数据 ——
 * 不调用它，is_online / state 永远不会变成 LOST。
 */
void                  Ina226_Update(void);

/** 1 = 通信正常；0 = 超时 / 未初始化。名字与返回值语义一致 */
uint8_t               Ina226_IsOnline(void);

/** 取状态分级（读的是 Ina226_Update 刷新的缓存，不重新算） */
Ina226_StateEnum      Ina226_GetState(void);

/** 取建议限力系数（本模块只计算，是否执行由底盘控制决定） */
float                 Ina226_GetSafeK(void);

/** 1 = FLAGS 里该位被置起。mask 用 INA226_F_xxx */
uint8_t               Ina226_IsFlagsBitSet(uint16_t mask);

/** 状态枚举 -> 可读字符串 */
const char*           Ina226_GetStateName(Ina226_StateEnum state);

/** 事件码 -> 可读字符串 */
const char*           Ina226_GetEventName(uint8_t ev_code);

/**
 * 解析自检：喂协议文档里的示例帧，校验字节序和缩放。
 * 不依赖任何硬件，不污染 Ina226_PowerMeter 的在线数据和计数器。
 * @retval 1 = 全部通过；0 = 有失败（详见 Ina226_SelfTest.fail_step）
 */
uint8_t               Ina226_RunSelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* INA226_PERIPH_H */
