/* -------------  IMU_Adapter.h ------------- */
#pragma once
// #define Dbp(...)      // ←　现在所有 Dbp(...) 都变成空语句

#include "im948_CMD.h"

/* ***************************************
   求一阶差分角速度  [deg/s]
   传入:  cur     —— 当前角度
          prevPtr —— 上一次角度(静态保存)
          dt      —— 采样周期(秒)  -- 建议用 0.01 对应 100 Hz
   *************************************** */
float deriv(float cur, float dt = 0.01f);


class IMU_Adapter {
public:
    /* 与旧 Wireless_IMU 完全相同的公共成员 */
    float LTx=0, RTx=0;          // 左/右大腿角度  [deg]
    float LTAVx=0, RTAVx=0;      // 左/右大腿角速度[deg/s]
    float TX1=0, TX2=0, TX3=0, TX4=0;          // 左/右大腿角度  [deg]
    float VTX1=0, VTX2=0, VTX3=0, VTX4=0;      // 左/右大腿角速度[deg/s]
    /* 供 controller 调用的接口 */
    void INIT();          // 上电初始化（相当于 INIT()）
    void INIT_MEAN();     // 零点归一（相当于 INIT_MEAN()）
    void READ();          // 更新四个量（相当于 READ()）
    // 只对左右两路做就地归零（当前姿态记为 0°），不触碰 1~4
    void REZERO_LR(uint16_t warmup_ms = 400, uint16_t N = 200);

    bool ZeroToReference(float refL_deg, float refR_deg, float ref1_deg, float ref2_deg, float ref3_deg, float ref4_deg,
        uint16_t N, uint32_t warmup_ms, uint32_t timeout_ms);
    

private:
    /* ——内部—— */
    void pollOneByte(HardwareSerial& port, uint8_t (&pktFunc)(uint8_t));
    float offL = 0, offR = 0;    // 静止时零偏
    float off1 = 0, off2 = 0;    // 静止时零偏
    float off3 = 0, off4 = 0;    // 静止时零偏
    bool offset_locked = false;   // ★ 新增：offset锁

};
