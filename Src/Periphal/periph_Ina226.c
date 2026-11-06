/*
 * 文件：periph_Ina226.c
 * 描述：森林功率计 V1.4（ESP32-C3 + INA226）CAN 接收端实现。
 *
 * 本文件只负责【接收 + 解析 + 判据】，不做任何发送：
 *   - 0x2A0 快帧  500Hz  功率 / 电流 / 母线电压 / 限幅系数 k
 *   - 0x2A1 状态帧 50Hz   FLAGS / MODE / SEQ / 未滤波功率 / 当前目标功率
 *   - 0x2A2 事件帧 突发   事件码 / 关联量 / 功率计时间戳 / FLAGS 低 8 位
 *
 * 心跳 (0x2B0) 与 k 的执行属于后续步骤，本文件不涉及。
 *
 * 全部字段小端。字节序写反是这套协议最常见的 bug，
 * 所以留了 Ina226_RunSelfTest()：用协议文档里的示例帧自检，见文件末尾。
 */

#include "periph_Ina226.h"
#include "can.h"
#include <string.h>

/* ==================================================================
 *  全局命名实例 —— 调试时在 Watch 窗口直接展开这一个对象
 * ================================================================== */
Ina226_DeviceTypeDef   Ina226_PowerMeter;
Ina226_SelfTestTypeDef Ina226_SelfTest;

/* ==================================================================
 *  小端读取工具（全部小端，d[0] 是低字节）
 * ================================================================== */

static uint16_t Ina226_ReadU16(const uint8_t* p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t Ina226_ReadI16(const uint8_t* p)
{
    return (int16_t)Ina226_ReadU16(p);
}

static uint32_t Ina226_ReadU32(const uint8_t* p)
{
    return ((uint32_t)p[0])
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* 判断 CAN 句柄 -> 便于在结构体里记录"这帧从哪条总线来" */
static uint8_t Ina226_GetCanPort(CAN_HandleTypeDef* phcan)
{
    if (phcan == &hcan1) return 1u;
    if (phcan == &hcan2) return 2u;
    return 0u;
}

/* ==================================================================
 *  解析一帧的公共部分
 * ================================================================== */

/* 0x2A0：功率快帧 */
static void Ina226_DecodePwr(Ina226_DeviceTypeDef* pdev, const uint8_t* d, uint32_t now)
{
    uint8_t i;

    /* 1) 先留一份原始字节，方便和协议文档逐字节对照 */
    for (i = 0u; i < INA226_FRAME_LEN; i++) pdev->last_pwr_data[i] = d[i];

    /* 2) 原始值 */
    pdev->raw_p_0p1w  = Ina226_ReadU16(&d[0]);
    pdev->raw_i_0p01a = Ina226_ReadI16(&d[2]);
    pdev->raw_v_0p01v = Ina226_ReadU16(&d[4]);
    pdev->raw_k_0p0001 = Ina226_ReadU16(&d[6]);

    /* 3) 工程值 */
    pdev->power_w   = (float)pdev->raw_p_0p1w  * 0.1f;
    pdev->current_a = (float)pdev->raw_i_0p01a * 0.01f;
    pdev->voltage_v = (float)pdev->raw_v_0p01v * 0.01f;
    pdev->k         = (float)pdev->raw_k_0p0001 * 0.0001f;

    /* k 理论范围 0~1，超了说明帧异常，按协议上限钳住 */
    if (pdev->k > 1.0f) pdev->k = 1.0f;
    if (pdev->k < 0.0f) pdev->k = 0.0f;

    /* 4) 统计 */
    pdev->pwr_frame_cnt++;
    pdev->rx_ok_cnt++;
    pdev->last_pwr_time    = now;
    pdev->last_update_time = now;
    pdev->is_online        = 1u;            /* ★ 共享标志位最后写 */
}

/* 0x2A1：状态帧 */
static void Ina226_DecodeStatus(Ina226_DeviceTypeDef* pdev, const uint8_t* d, uint32_t now)
{
    uint8_t i;
    uint8_t expect_seq;

    for (i = 0u; i < INA226_FRAME_LEN; i++) pdev->last_status_data[i] = d[i];

    /* 原始值 */
    pdev->raw_flags        = Ina226_ReadU16(&d[0]);
    pdev->raw_mode         = d[2];
    pdev->raw_seq          = d[3];
    pdev->raw_p_raw_0p1w   = Ina226_ReadU16(&d[4]);
    pdev->raw_target_0p1w  = Ina226_ReadU16(&d[6]);

    /* 工程值 */
    pdev->power_raw_w = (float)pdev->raw_p_raw_0p1w  * 0.1f;
    pdev->target_w    = (float)pdev->raw_target_0p1w * 0.1f;

    /* SEQ 丢帧统计：每帧 +1，255 -> 0。跳号说明中间丢了帧 */
    if (pdev->seq_valid != 0u)
    {
        expect_seq = (uint8_t)(pdev->last_seq + 1u);
        if (pdev->raw_seq != expect_seq)
        {
            uint16_t lost = (uint16_t)(uint8_t)(pdev->raw_seq - expect_seq);
            if ((uint32_t)pdev->seq_lost_cnt + (uint32_t)lost < 0xFFFFu)
                pdev->seq_lost_cnt = (uint16_t)(pdev->seq_lost_cnt + lost);
            else
                pdev->seq_lost_cnt = 0xFFFFu;   /* 饱和，避免回绕成小数字骗人 */
        }
    }
    pdev->last_seq   = pdev->raw_seq;
    pdev->seq_valid  = 1u;

    /* 统计 */
    pdev->status_frame_cnt++;
    pdev->rx_ok_cnt++;
    pdev->last_status_time = now;
    pdev->last_update_time = now;
    pdev->is_online        = 1u;            /* ★ 共享标志位最后写 */
}

/* 0x2A2：事件帧 */
static void Ina226_DecodeEvent(Ina226_DeviceTypeDef* pdev, const uint8_t* d, uint32_t now)
{
    uint8_t i;

    for (i = 0u; i < INA226_FRAME_LEN; i++) pdev->last_event_data[i] = d[i];

    pdev->ev_code         = d[0];
    pdev->raw_ev_val_0p1  = Ina226_ReadI16(&d[1]);
    pdev->ev_val          = (float)pdev->raw_ev_val_0p1 * 0.1f;
    pdev->ev_t_ms         = Ina226_ReadU32(&d[3]);
    pdev->ev_flags        = d[7];

    pdev->event_frame_cnt++;
    pdev->rx_ok_cnt++;
    pdev->last_event_time  = now;
    pdev->last_update_time = now;
    pdev->is_online        = 1u;            /* ★ 共享标志位最后写 */

    /* 事件码 7 = 已切断 Q5，底盘立即失电，属于最高优先级留痕。
       真正的降力动作留给上层调用者，本模块只把事实记在结构体里。 */
}

/* ==================================================================
 *  判据：状态分级 + 建议 k
 * ================================================================== */

/*
 * 顺序很重要（协议 §8.1）：先判"失电/切断"，再判"传感器故障"，
 * 再判"通信超时"，最后才判"告警"。
 */
static Ina226_StateEnum Ina226_ComputeState(const Ina226_DeviceTypeDef* pdev, uint32_t now)
{
    if (pdev->init == 0u) return Ina226_State_NotInit;

    if ((pdev->raw_flags & INA226_F_CUT_DONE) != 0u)
        return Ina226_State_Cut;

    if ((pdev->raw_flags & (INA226_F_INA_FAULT | INA226_F_CLIP)) != 0u)
        return Ina226_State_SensorFault;

    if (((uint32_t)(now - pdev->last_pwr_time)    > (uint32_t)INA226_TIMEOUT_PWR_MS) ||
        ((uint32_t)(now - pdev->last_status_time) > (uint32_t)INA226_TIMEOUT_STATUS_MS))
        return Ina226_State_Lost;

    if ((pdev->raw_flags & (INA226_F_OVER_HARD | INA226_F_ALERT_POL |
                            INA226_F_V_FAULT   | INA226_F_SHUNT_HOT)) != 0u)
        return Ina226_State_WARN;

    return Ina226_State_OK;
}

static float Ina226_ComputeSafeK(const Ina226_DeviceTypeDef* pdev, Ina226_StateEnum state)
{
    switch (state)
    {
        case Ina226_State_OK:
            return pdev->k;

        case Ina226_State_WARN:
            /* 硬件告警：额外压一档，比等功率计下一帧更快 */
            return (pdev->k > 0.8f) ? 0.8f : pdev->k;

        case Ina226_State_SensorFault:
        case Ina226_State_Lost:
            return INA226_K_SAFE_LOST;

        case Ina226_State_Cut:
            return 0.0f;

        case Ina226_State_NotInit:
        default:
            return INA226_K_SAFE_LOST;
    }
}

/* ==================================================================
 *  对外 API
 * ================================================================== */

Ina226_DeviceTypeDef* Ina226_GetDevicePtr(void)
{
    return &Ina226_PowerMeter;
}

void Ina226_Init(void)
{
    Ina226_DeviceTypeDef* pdev = Ina226_GetDevicePtr();

    (void)memset(pdev, 0, sizeof(*pdev));

    pdev->enabled = 1u;
    pdev->init    = 1u;

    /* 还没通信过：k 先给 1.0 表示"不限制"，
       但 state 直接是 Lost，所以 k_safe 是 0.85，由状态机兜底。 */
    pdev->k       = 1.0f;
    pdev->k_safe  = INA226_K_SAFE_LOST;
    pdev->state   = Ina226_State_Lost;

    pdev->last_can_port = 0u;
    pdev->last_stdid    = 0u;
    pdev->last_dlc      = 0u;
}

uint8_t Ina226_OnCanFrame(CAN_HandleTypeDef* phcan,
                          const CAN_RxHeaderTypeDef* pheader,
                          const uint8_t* rxdata)
{
    Ina226_DeviceTypeDef* pdev = Ina226_GetDevicePtr();
    uint32_t now;

    /* 未初始化或被人为关掉：不消费任何帧 */
    if ((pdev->init == 0u) || (pdev->enabled == 0u)) return 0u;
    if ((phcan == NULL) || (pheader == NULL) || (rxdata == NULL)) return 0u;

    /* 功率计只发标准帧，扩展帧一定是别的设备 */
    if (pheader->IDE != CAN_ID_STD) return 0u;

    /* 只认这三个 ID，其余返回 0 让上层继续分发给别的模块 */
    if ((pheader->StdId != INA226_ID_PWR) &&
        (pheader->StdId != INA226_ID_STATUS) &&
        (pheader->StdId != INA226_ID_EVENT))
    {
        return 0u;
    }

    now = HAL_GetTick();

    /* 记下来源，方便在 Watch 里确认它到底挂在哪条 CAN 上 */
    pdev->last_stdid    = pheader->StdId;
    pdev->last_dlc      = (uint8_t)pheader->DLC;
    pdev->last_can_port = Ina226_GetCanPort(phcan);

    /* 数据帧恒为 8 字节。短帧 / 远程帧直接丢弃：
       不检查 DLC 会把上一帧留在缓冲里的垃圾当新数据用。 */
    if ((pheader->RTR != CAN_RTR_DATA) || (pheader->DLC != INA226_FRAME_LEN))
    {
        pdev->rx_err_cnt++;
        return 1u;                          /* 已认领：ID 是本设备的，只是这帧不可用 */
    }

    switch (pheader->StdId)
    {
        case INA226_ID_PWR:
            Ina226_DecodePwr(pdev, rxdata, now);
            break;

        case INA226_ID_STATUS:
            Ina226_DecodeStatus(pdev, rxdata, now);
            break;

        case INA226_ID_EVENT:
            Ina226_DecodeEvent(pdev, rxdata, now);
            break;

        default:
            return 0u;
    }

    return 1u;
}

void Ina226_Update(void)
{
    Ina226_DeviceTypeDef* pdev = Ina226_GetDevicePtr();
    uint32_t now;

    if (pdev->init == 0u)
    {
        pdev->is_online = 0u;
        pdev->state     = Ina226_State_NotInit;
        pdev->k_safe    = INA226_K_SAFE_LOST;
        return;
    }

    now = HAL_GetTick();

    /* 1) 新鲜度：靠时间戳判，不依赖有没有新帧进来 */
    if (((uint32_t)(now - pdev->last_pwr_time)    > (uint32_t)INA226_TIMEOUT_PWR_MS) ||
        ((uint32_t)(now - pdev->last_status_time) > (uint32_t)INA226_TIMEOUT_STATUS_MS))
    {
        pdev->is_online = 0u;
    }
    else
    {
        pdev->is_online = 1u;
    }

    /* 2) 状态分级 */
    pdev->state = Ina226_ComputeState(pdev, now);

    /* 3) 建议限力系数（只算，不执行） */
    pdev->k_safe = Ina226_ComputeSafeK(pdev, pdev->state);
}

uint8_t Ina226_IsOnline(void)
{
    return (Ina226_PowerMeter.is_online != 0u) ? 1u : 0u;
}

Ina226_StateEnum Ina226_GetState(void)
{
    return Ina226_PowerMeter.state;
}

float Ina226_GetSafeK(void)
{
    return Ina226_PowerMeter.k_safe;
}

uint8_t Ina226_IsFlagsBitSet(uint16_t mask)
{
    return ((Ina226_PowerMeter.raw_flags & mask) != 0u) ? 1u : 0u;
}

const char* Ina226_GetStateName(Ina226_StateEnum state)
{
    switch (state)
    {
        case Ina226_State_OK:          return "OK";
        case Ina226_State_WARN:        return "WARN";
        case Ina226_State_SensorFault: return "SENSOR_FAULT";
        case Ina226_State_Lost:        return "LOST";
        case Ina226_State_Cut:         return "CUT";
        case Ina226_State_NotInit:     return "NOT_INIT";
        default:                       return "UNKNOWN";
    }
}

const char* Ina226_GetEventName(uint8_t ev_code)
{
    switch (ev_code)
    {
        case INA226_EV_OVER_HARD:  return "OVER_HARD";
        case INA226_EV_ALERT:      return "ALERT";
        case INA226_EV_CLIP:       return "CLIP";
        case INA226_EV_INA_FAULT:  return "INA_FAULT";
        case INA226_EV_CAN_BUSOFF: return "CAN_BUSOFF";
        case INA226_EV_MOS_LEAK:   return "MOS_LEAK";
        case INA226_EV_CUT:        return "CUT";
        case INA226_EV_V_FAULT:    return "V_FAULT";
        case INA226_EV_ENERGY_LOW: return "ENERGY_LOW";
        case INA226_EV_SHUNT_HOT:  return "SHUNT_HOT";
        default:                   return "UNKNOWN";
    }
}

/* ==================================================================
 *  解析自检 —— 用协议文档里的示例帧验证"字节序 + 缩放"
 *
 *  这一步和硬件完全无关。pass == 1 就能排除"软件解析写错了"，
 *  剩下的问题一定在接线 / 波特率 / 终端电阻 / ID 冲突上。
 * ================================================================== */

/* 浮点近似比较：缩放系数是 0.1 / 0.01 / 0.0001，0.001 的容差足够 */
static uint8_t Ina226_Near(float a, float b)
{
    float diff = a - b;
    if (diff < 0.0f) diff = -diff;
    return (diff <= 0.001f) ? 1u : 0u;
}

uint8_t Ina226_RunSelfTest(void)
{
    Ina226_DeviceTypeDef tmp;
    Ina226_StateEnum     st;
    uint8_t  ok = 1u;
    uint8_t  step = 0u;

    (void)memset(&tmp, 0, sizeof(tmp));
    tmp.init    = 1u;
    tmp.enabled = 1u;
    tmp.k       = 1.0f;

    /* ---- 用例 1：文档 §4 的示例帧 P=138.4W / I=5.62A / V=24.63V / k=0.8734 ----
       整帧: 68 05 32 02 9F 09 1E 22                                     */
    {
        static const uint8_t frame[INA226_FRAME_LEN] =
            { 0x68u, 0x05u, 0x32u, 0x02u, 0x9Fu, 0x09u, 0x1Eu, 0x22u };

        Ina226_DecodePwr(&tmp, frame, 0u);

        Ina226_SelfTest.p_w = tmp.power_w;
        Ina226_SelfTest.i_a = tmp.current_a;
        Ina226_SelfTest.v_v = tmp.voltage_v;
        Ina226_SelfTest.k   = tmp.k;

        step = 1u;
        if ((Ina226_Near(tmp.power_w,   138.4f) == 0u) ||
            (Ina226_Near(tmp.current_a,   5.62f) == 0u) ||
            (Ina226_Near(tmp.voltage_v,  24.63f) == 0u) ||
            (Ina226_Near(tmp.k,         0.8734f) == 0u))
        {
            ok = 0u;
        }
    }

    /* ---- 用例 2：负电流要解成 -12.34A，不能当无符号解成 643.02A ----
       -1234 = 0xFB2E -> d[2]=0x2E d[3]=0xFB                              */
    {
        const uint8_t frame[INA226_FRAME_LEN] =
            { 0x00u, 0x00u, 0x2Eu, 0xFBu, 0x9Fu, 0x09u, 0x10u, 0x27u };

        Ina226_DecodePwr(&tmp, frame, 0u);

        Ina226_SelfTest.neg_i_a = tmp.current_a;

        step = 2u;
        if (Ina226_Near(tmp.current_a, -12.34f) == 0u) ok = 0u;
    }

    /* ---- 用例 3：状态帧的未滤波功率 / 目标功率 / FLAGS ----
       P_RAW = 1500 -> 150.0W，TARGET = 1380 -> 138.0W，FLAGS = 0x2000 (LIMITING) */
    {
        const uint8_t frame[INA226_FRAME_LEN] =
            { 0x00u, 0x20u, 0x02u, 0x10u, 0xDCu, 0x05u, 0x64u, 0x05u };

        Ina226_DecodeStatus(&tmp, frame, 0u);

        Ina226_SelfTest.raw_p_w = tmp.power_raw_w;

        step = 3u;
        if ((Ina226_Near(tmp.power_raw_w, 150.0f) == 0u) ||
            (Ina226_Near(tmp.target_w,    138.0f) == 0u) ||
            (tmp.raw_mode != 2u) ||
            ((tmp.raw_flags & INA226_F_LIMITING) == 0u))
        {
            ok = 0u;
        }
    }

    /* ---- 用例 4：k 原始值超 10000 时必须钳到 1.0，不能给出 k > 1 ----
       12000 = 0x2EE0                                                     */
    {
        const uint8_t frame[INA226_FRAME_LEN] =
            { 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0xE0u, 0x2Eu };

        Ina226_DecodePwr(&tmp, frame, 0u);

        step = 4u;
        if (Ina226_Near(tmp.k, 1.0f) == 0u) ok = 0u;
    }

    /* ---- 记录结果（不污染 Ina226_PowerMeter 的在线数据和计数器） ---- */
    Ina226_SelfTest.run_cnt++;
    Ina226_SelfTest.pass      = ok;
    Ina226_SelfTest.fail_step = (ok != 0u) ? 0u : step;
    if (ok == 0u) Ina226_SelfTest.fail_cnt++;

    /* 顺带验证状态机在"从未收到过帧"时确实判成 LOST + k_safe = 0.85 */
    (void)memset(&tmp, 0, sizeof(tmp));
    tmp.init = 1u;
    st = Ina226_ComputeState(&tmp, 1000u);
    if (st != Ina226_State_Lost) { Ina226_SelfTest.pass = 0u; Ina226_SelfTest.fail_step = 5u; }
    else if (Ina226_Near(Ina226_ComputeSafeK(&tmp, st), INA226_K_SAFE_LOST) == 0u)
    {
        Ina226_SelfTest.pass = 0u; Ina226_SelfTest.fail_step = 6u;
    }
    else { /* 全部通过 */ }

    return Ina226_SelfTest.pass;
}
