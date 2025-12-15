#include "Motor_Control_Tmotor.h"
//FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

Motor_Control_Tmotor::Motor_Control_Tmotor(uint8_t id, int Can_id)
{
  drive_id = (uint8_t)Can_id;       // 伺服扩展帧 DriveID（例如 104）
}


Motor_Control_Tmotor::~Motor_Control_Tmotor()
{}

void Motor_Control_Tmotor::send_current_A(float iq_A)
{
  // 伺服文档：-60~60A -> -60000~60000
  if (iq_A > 60.0f) iq_A = 60.0f;
  if (iq_A < -60.0f) iq_A = -60.0f;

  int32_t iq_mA = (int32_t)(iq_A * 1000.0f);

  msgW.len = 4;
  msgW.flags.extended = 1;   // 关键：扩展帧
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;

  // control mode ID = 1 (CAN_PACKET_SET_CURRENT)
  uint32_t ext_id = ((uint32_t)drive_id) | ((uint32_t)1u << 8);
  msgW.id = ext_id;

  // 大端：最高字节先发（与你文档 buffer_append_int32 一致）
  msgW.buf[0] = (uint8_t)((iq_mA >> 24) & 0xFF);
  msgW.buf[1] = (uint8_t)((iq_mA >> 16) & 0xFF);
  msgW.buf[2] = (uint8_t)((iq_mA >>  8) & 0xFF);
  msgW.buf[3] = (uint8_t)((iq_mA >>  0) & 0xFF);

  Can3.write(msgW);
}

bool Motor_Control_Tmotor::unpack_servo_telemetry(const CAN_message_t &msgR)
{
  if (msgR.len != 8) return false;

  // 可选：如果你希望只接收某个驱动器的回传，可以用 ID 过滤
  // 很多实现会把 drive_id 放在 EID 低8位；标准帧则直接 msgR.id==drive_id
  if (msgR.flags.extended) {
    uint8_t rx_drive = (uint8_t)(msgR.id & 0xFF);
    if (rx_drive != drive_id) return false;
  } else {
    // 若回传用标准帧：就按标准帧 id == drive_id 过滤
    if ((uint8_t)msgR.id != drive_id) return false;
  }

  int16_t pos_int = (int16_t)((msgR.buf[0] << 8) | msgR.buf[1]);
  int16_t spd_int = (int16_t)((msgR.buf[2] << 8) | msgR.buf[3]);
  int16_t cur_int = (int16_t)((msgR.buf[4] << 8) | msgR.buf[5]);

  int8_t  temp_raw = (int8_t)msgR.buf[6];
  uint8_t err_raw  = (uint8_t)msgR.buf[7];

  servo_pos_deg = (float)pos_int * 0.1f;
  servo_spd_rpm = (float)spd_int * 10.0f;
  servo_cur_A   = (float)cur_int * 0.01f;
  servo_temp_C  = temp_raw;
  servo_error   = err_raw;

  return true;
}




void Motor_Control_Tmotor::enter_control_mode()
{
  msgW.id = ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xFF;
  msgW.buf[1] = 0xFF;
  msgW.buf[2] = 0xFF;
  msgW.buf[3] = 0xFF;
  msgW.buf[4] = 0xFF;
  msgW.buf[5] = 0xFF;
  msgW.buf[6] = 0xFF;
  msgW.buf[7] = 0xFC;
  send_CAN_message();
}
void Motor_Control_Tmotor::exit_control_mode()
{
  msgW.id = ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xFF;
  msgW.buf[1] = 0xFF;
  msgW.buf[2] = 0xFF;
  msgW.buf[3] = 0xFF;
  msgW.buf[4] = 0xFF;
  msgW.buf[5] = 0xFF;
  msgW.buf[6] = 0xFF;
  msgW.buf[7] = 0xFD;
  send_CAN_message();
}
void Motor_Control_Tmotor::set_origin()
{
  msgW.id = ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xFF;
  msgW.buf[1] = 0xFF;
  msgW.buf[2] = 0xFF;
  msgW.buf[3] = 0xFF;
  msgW.buf[4] = 0xFF;
  msgW.buf[5] = 0xFF;
  msgW.buf[6] = 0xFF;
  msgW.buf[7] = 0xFE;
  send_CAN_message();
}
void Motor_Control_Tmotor::send_cmd(float p_des, float v_des, float kp, float kd, float t_ff )
{
  //limit of desired position, desired velocity, kp, kd, & desired torque
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
  //convert floats to unsigned ints
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
  int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  msgW.id = ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = p_int >> 8;  //position 8-H
  msgW.buf[1] = p_int & 0xFF;  //position 8-L
  msgW.buf[2] = v_int >> 4;                       //speed 8-H
  msgW.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); //Speed 4-L Kp-8H
  msgW.buf[4] = kp_int & 0xFF; //Kp 8-L
  msgW.buf[5] = kd_int >> 4;   //Kd 8-H
  msgW.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); //KP 4-L torque 4-H
  msgW.buf[7] = t_int & 0xff;  //torque 8-L
  send_CAN_message();
}
int Motor_Control_Tmotor::float_to_uint(float x, float x_min, float x_max, uint8_t nbits)
{
  float span = x_max - x_min;
  if (x < x_min) {
    x = x_min;
  }
  else if (x > x_max) {
    x = x_max;
  }
  return (int)((x - x_min) * ((float)((1 << nbits) - 1) / span));
}
void Motor_Control_Tmotor::unpack_reply(CAN_message_t msgR)
{
  int receive_id = msgR.buf[0];
  int p_int = (msgR.buf[1] << 8) | msgR.buf[2];
  int v_int = (msgR.buf[3] << 4) | (msgR.buf[4] >> 4);
  int i_int = ((msgR.buf[4] & 0xF) << 8) | msgR.buf[5];
  int temp_int = msgR.buf[6];
  //convert ints to floats
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  float Temp = uint_to_float(temp_int, Temp_MIN, Temp_MAX, 8);
  if (receive_id == ID) {
    pos = p;
    spe = v;
    torque = i;
    temp = Temp;
  }
}
float Motor_Control_Tmotor::uint_to_float(int x_int, float x_min, float x_max, uint8_t nbits)
{
  float span = x_max - x_min;
  float offset_value = x_min;
  return ((float)x_int) * span / ((float)((1 << nbits) - 1)) + offset_value;
}
void Motor_Control_Tmotor::send_CAN_message()
{
  Can3.write(msgW);
  if (Can3.write(msgW))
  {
    //Serial.println("S");
  }
  else
  {
    Serial.println("F");
  }
}


void Motor_Control_Tmotor::initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
}
