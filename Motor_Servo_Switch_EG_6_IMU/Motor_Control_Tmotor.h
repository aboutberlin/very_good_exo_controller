//#ifndef _MOTOR_CONTROL_TMOTOR_H
//#define _MOTOR_CONTROL_TMOTOR_H
//#include <FlexCAN.h>
#include <FlexCAN_T4.h>
#include <Arduino.h>
#ifndef __IMXRT1062__
#error "Teensy 3.6 with dual CAN bus is required to run this code"
#endif

//FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

class Motor_Control_Tmotor
{
  public:

    Motor_Control_Tmotor(uint8_t id, int c0);
    ~Motor_Control_Tmotor();
    void initial_CAN();

    // ===== 原 MIT 接口（保留）=====
    void enter_control_mode();
    void exit_control_mode();
    void set_origin();
    void send_cmd(float p_des, float v_des, float kp, float kd, float t_ff);
    void unpack_reply(CAN_message_t msgR); // MIT 回复（不要删）

    // ===== 新增：伺服模式 电流环发送（扩展帧 EID）=====
    // 电流单位：A。内部按文档 *1000 -> int32(mA) 下发
    void send_current_A(float iq_A);

    // ===== 新增：伺服模式 定时上传 8字节解析 =====
    // 解析成功返回 true；并更新下面这些字段
    bool unpack_servo_telemetry(const CAN_message_t &msgR);



    // ===== 你原有的 MIT 字段（若已存在就不要重复定义）=====
    float pos = 0.0f;     // (MIT) rad
    float spe = 0.0f;     // (MIT) rad/s
    float torque = 0.0f;  // (MIT) Nm (你原本用 -T_MAX..T_MAX 映射)
    float temp = 0.0f;    // (MIT) 你原本是 uint_to_float 映射

    // ===== 新增：伺服上传字段（按你文档的标定）=====
    float servo_pos_deg = 0;
    float servo_spd_rpm_elec = 0;   // <-- 建议改名：明确是电气rpm
    float servo_cur_A = 0;
    float servo_temp_C = 0;
    uint8_t servo_error = 0;

    // ---- derived ----
    float servo_spd_rpm_motor = 0;   // 电机轴机械rpm
    float servo_spd_rpm_out   = 0;   // 输出轴机械rpm

    // params
    uint16_t pole_pairs = 21;        // 你的极对数（21对）
    float gear_ratio = 9.0f;         // 你的减速比（9:1）


    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
  private:
    CAN_message_t msgW;
    CAN_message_t msgR;
    // struct CAN_filter_t defaultMask;
    uint8_t ID = 0;          // 你原 MIT 标准帧 ID（11bit）
    uint8_t drive_id = 0;    // 新增：伺服模式的 DriveID（低 8bit），例如 104



    float P_MIN = -12.5; //rad
    float P_MAX = 12.5; //rad
    float V_MIN = -30;  //rad/s
    float V_MAX = 30;   //rad/s
    float T_MIN = -18;  //Nm
    float T_MAX = 18;   //Nm
    float Temp_MIN = 0;  //Nm
    float Temp_MAX = 100;   //Nm
    float Kp_MIN = 0;
    float Kp_MAX = 500;
    float Kd_MIN = 0;
    float Kd_MAX = 5;
    float Test_Pos = 0.0;


    void send_CAN_message();
    int float_to_uint(float x, float x_min, float x_max, uint8_t nbits);
    float uint_to_float(int x_int, float x_min, float x_max, uint8_t nbits);


};
//#endif
