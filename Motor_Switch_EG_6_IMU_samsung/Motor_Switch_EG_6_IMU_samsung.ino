/******************** 必要的包含 ********************/
#include <Arduino.h>
#include <math.h>
#include <cstring>
#include <iomanip> 
#include <FlexCAN_T4.h>
#include "Serial_Com.h"
#include "IMU_Adapter.h"
#include "MovingAverage.h"
#include "SdFat.h"
#include "RingBuf.h"
#include "sdlogger.h" 

#include "Sig_Motor_Control.h"
// SIG 所需 ID（如果已定义可跳过）
const uint16_t ID_M1_POSVEL = (0x002<<5) | 0x009; // 0x049
const uint16_t ID_M1_TORQUE = (0x002<<5) | 0x01C; // 0x05C
const uint16_t ID_M2_POSVEL = (0x001<<5) | 0x009; // 0x029
const uint16_t ID_M2_TORQUE = (0x001<<5) | 0x01C; // 0x03C
const uint16_t ID_M1_IQ = (0x002<<5) | 0x014; // 0x064
const uint16_t ID_M2_IQ = (0x001<<5) | 0x014; // 0x034

#define KT_1 0.67f
#define KT_2 0.67f



// ====== Motor brand switch (0 = SIG, 1 = TMOTOR) ======
#define MOTOR_BRAND 0   

#ifndef max_torque_cfg
#define DEBUG_PRINT 1   // 改成 0 关闭所有调试打印
#endif
#define GUI_WRITE_ENABLE   1   // 把它改成0就是锁死
// --- LOGTAG 全局变量 ---
static char logtag[11] = {0};       // 最多10字符 + '\0'
static bool logtag_valid = false;   // 是否有标签可写入
static bool logtag_persist = false; // 是否持续打印


const unsigned long PRINT_INTERVAL_US = 100000; // 10 Hz
static unsigned long prev_print_us = 0;

/******************** 常量/类型 ********************/
const float DEG2RAD = PI / 180.0f;

// Teensy CAN3
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// SdLogger：自动按前缀+递增命名，例如 walking_log_0001.csv
SdLogger logger(BUILTIN_SDCARD, F("walking_log_"), F(".csv"));

// SDIO 配置（如你后续需要直接用 SdFat，可继续用）
#define SD_CONFIG SdioConfig(FIFO_SDIO)
// 初始化状态 bitmask
// bit0: CAN
// bit1: IMU
// bit2: SD logger
// bit3: Motor
// bit4: Serial_Com
// bit5: BLE(Serial5)
// 你后面还想加别的就继续往上加
uint16_t g_init_status = 0;
// 频率设置
double cyclespersec_ctrl = 100;   // [Hz] 控制循环频率（CAN上限1000Hz）
double cyclespersec_ble  = 20;    // [Hz] 蓝牙发送频率
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000.0 / cyclespersec_ctrl);
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000.0 / cyclespersec_ble);
static float tau_cmd_L_filt = 0.0f;
static float tau_cmd_R_filt = 0.0f;
/******************** 设备对象 ********************/
// 单电机示例（你的 ID/总线保持不变）
int CAN_ID = 3;                       // 总线地址（你的代码里就是3）
/*MOTOR*/    
float initial_pos_1 = 0;       
float initial_pos_2 = 0;     
Motor_Control_Tmotor sig_m1(0x002, CAN_ID);
Motor_Control_Tmotor sig_m2(0x001, CAN_ID);

// 通讯与传感
Serial_Com Serial_Com;                // 你已有的串口通讯类
IMU_Adapter imu;                      // IMU 适配器

/******************** 蓝牙缓冲（与你一致） ********************/
char datalength_ble = 32;
char data_ble[60] = {0};
uint8_t data_rs232_rx[60] = {0};  // <-- char → uint8_t

// IMU 初始化状态（新增）
volatile uint8_t imu_init_ok = 0;    // 0=未通过, 1=通过

// 你声明过的延时/环形缓存（日志里有用到）
double RLTx_delay[100]   = {};
double torque_delay[100] = {};
int doi = 0;
int delayindex   = 0;    

//*** Motor Mode Set ***//   
int ctl_method = 1;    // 0 for using RL controller, 1 for using other normal controller  
int ctl_mode = 0;      // 0 for torque control, 1 for mit control    
int ctl_type = 0;      // 0 for motion, 1 for force tracking, 2 for direct torque   

int sensor_type = 0;   // 0 for using IMU, 1 for using encoder   
int l_ctl_dir = -1;      //确实是左脚，1是向上
int r_ctl_dir = 1;     //确实是右脚，1是向上



/******************** 控制/算法相关占位（将来替换） ********************/
// —— 这些量目前只为“能编译+能记录”，实际算法稍后接回 ——

// 电机命令 & 实测
float  M1_torque_meas = 0.0f, M2_torque_meas = 0.0f;
double M1_torque_command = 0.0, M2_torque_command = 0.0;

// IMU 与派生量（日志里会用到）
double LTx_filtered = 0, LTx_filtered_last = 0;
double RTx_filtered = 0, RTx_filtered_last = 0;
double RLTx              = 0;
double RLTx_filtered     = 0;



// 其它在日志/蓝牙中被引用的占位
float  S_torque_command_left  = 0.0f;
float  S_torque_command_right = 0.0f;
float  gait_freq = 0;

volatile float  torque_rate   = 150.0f;  // Nm/s
float  max_torque_cfg = 0.0f; 
static uint8_t  gait_inited = 0;


// ---- Gate 输入预测需要的状态（每腿各一个）----
static float xL_prev = 0.0f;
static float xR_prev = 0.0f;
float M1_prev = 0.0f, M2_prev = 0.0f;

/******************** 时间变量 ********************/
unsigned long t_0 = 0;
unsigned long current_time = 0;
unsigned long previous_time = 0;        // 控制循环节拍
unsigned long previous_time_ble = 0;    // BLE 发送节拍






/******************** 前置声明 ********************/
void initial_CAN();
void initial_Sig_motor();         // 这里给出一个“空实现”，防止链接错误
void receive_motor_feedback();
void Receive_ble_Data();
void Transmit_ble_Data();

/******************** setup ********************/
void setup() {
  delay(3000);

  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  // BLE 串口
  Serial5.begin(115200);
  Serial5.setTimeout(5);

  // 你自有的串口通讯初始化
  Serial_Com.INIT();
  delay(300);

  // 缓存清零
  memset(RLTx_delay,   0, sizeof(RLTx_delay));
  memset(torque_delay, 0, sizeof(torque_delay));

  // CAN 初始化
  initial_CAN();
  Serial.println("[OK] CAN bus setup");

  // IMU 初始化
  imu.INIT();
  delay(2000);
  imu.INIT_MEAN();        // 自动零偏
  Serial.println("[OK] IMU setup done");
  imu_init_ok = 1;

  // 电机初始化（占位函数，确保存在；具体初始化放你自己的库里）
  initial_Sig_motor();
  Serial.println("[OK] T-Motor init done");

  // SD 日志
  if (logger.begin()) {
    Serial.print(F("SD Logging file: "));
    Serial.println(logger.filename());
    logger.println(F(
      "Time_ms,imu_RTx,imu_LTx,imu_1_left_shaking,imu_2_right_shaking,imu_3_left_foot,imu_4_right_foot,"
      "RLTx_delay,torque_delay,tau_raw_L,tau_raw_R,"
      "S_torque_command_left,S_torque_command_right,M1_torque_command,M2_torque_command,Rescaling_gain,"
      "imu_Rvel,imu_Lvel,freq_avg,"
      "R_flexion,L_flexion,extra3,R_extension,L_extension"
    ));
    logger.flush();
    g_init_status = 1;
  } else {
    Serial.println(F("SD card init or file create failed!"));
    // 这里就别置 INIT_SD 这位了
  }
  t_0 = micros();
}

// 0.5*(tanh(k*(x - p_on)) + 1)  带阈值的软门
inline float smooth_gate_p(float x, float k, float p_on){
  return 0.5f * (tanhf(k * (x - p_on)) + 1.0f);
}
// 一阶线性预测：用当前斜率估计 lead_s 后的 x
inline float lead_predict(float x, float x_prev, float lead_ms, float Ts){
  const float lead_s = 0.001f * lead_ms;      // ms -> s
  float dx = (x - x_prev) / Ts;               // 斜率

  // —— 极简防抖：斜率限幅（先给保守值，后面再调）——
  const float DX_MAX = 200.0f;                // 依据你 x 的量级微调
  if (dx >  DX_MAX) dx =  DX_MAX;
  if (dx < -DX_MAX) dx = -DX_MAX;

  return x + lead_s * dx;
}
inline float slew(float target, float prev, float rate, float Ts){
  float diff = target - prev, maxDiff = rate * Ts;
  if (fabs(diff) > maxDiff) target = prev + copysignf(maxDiff, diff);
  return target;
}
inline float clip_torque(float t){
  float max_abs = fabsf(max_torque_cfg);              // 允许 GUI 传 负数也能容错成绝对值
  return (t >  max_abs) ?  max_abs :
         (t < -max_abs) ? -max_abs : t;
}

// 限幅到 [lo, hi]
static inline double clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// 辅助：把两个字节拼成 int16_t
static inline int16_t rd_i16(const uint8_t *buf, int idx) {
  return (int16_t)( buf[idx] | (buf[idx+1] << 8) );
}



int8_t flex_sign_L = -1; // 左腿屈相抓负半波
int8_t flex_sign_R = +1; // 右腿屈相抓正半波

// 门控提前比例: lead_ms = lead_frac * T_gait_ms
// 你现在把 lead_frac 设成了 60.0f。如果你希望“6% 的周期”，那它应该是 0.06f
// ===== 环形缓冲（仅存“屈相(负值)”）=====
#define EXT_BUF_LEN 400  // 足够覆盖更大的延迟（例如 Ts=10ms 时可覆盖4s）
static float flexL_hist[EXT_BUF_LEN] = {0.0f};
static float flexR_hist[EXT_BUF_LEN] = {0.0f};
static int   ext_i = 0; // 写指针（随周期递增）



/****************************************/

/****************************************
 * 控制参数（主控真正使用）
 * 说明：这些值是控制逻辑读取的“真值”
 ****************************************/
// 这三个目前是占位/不使用，但我们仍然保持解析，方便协议对齐
float Rescaling_gain    = 5.0f;
float Flex_Assist_gain  = 2.5f;
float Ext_Assist_gain   = 2.5f;
float Assist_delay_gain = 0.0f;
float scale_all         = 0.2f;

int8_t phase_offset_L   = 0;
int8_t phase_offset_R   = 0;

float ext_gain          = 0.5f;
float ext_phase_frac_L  = 0.3f;
float ext_phase_frac_R  = 0.3f;
float lead_frac         = 0.06f;
float gate_p_on         = 2.5f;

// ====== 用户输入 / 调参 ======
float kappa = 3.0f;        // 助力强度 gain κ
float delay_s = 0.25f;     // 延迟 Δt (second)

// ====== 内部变量 ======
const int BUF_LEN = 200;   // 100Hz  → 200 samples ≈ 2s buffer
float y_buffer[BUF_LEN];
int buf_idx = 0;

// 低通滤波
float LPF(float prev, float raw, float alpha) {
  return (1.0f - alpha) * prev + alpha * raw;
}

void loop() {
  imu.READ();
  current_time = micros();

  if (current_time - previous_time >= Tinterval_ctrl_micros) {
    previous_time = current_time;

    receive_motor_feedback();
    Receive_ble_Data();

    float q_r_raw = imu.RTx * DEG_TO_RAD;
    float q_l_raw = imu.LTx * DEG_TO_RAD;

    static float q_r = 0.0f, q_l = 0.0f;
    // 如果第一次运行，让滤波初值等于第一次采到的原始值（避免从 0 突变）
    static bool lpf_inited = false;
    if (!lpf_inited) { q_r = q_r_raw; q_l = q_l_raw; lpf_inited = true; }

    q_r = LPF(q_r, q_r_raw, 0.05f);
    q_l = LPF(q_l, q_l_raw, 0.05f);

    float y = sin(q_r) - sin(q_l);

    // 第一次初始化 buffer（如果尚未）
    // 先推进写指针（写入当前位置），这种约定：buf_idx 指向下一个要写的位置
    buf_idx = (buf_idx + 1) % BUF_LEN;   // advance
    y_buffer[buf_idx] = y;               // now write

    // 计算 Ndelay（用 round 更稳）
    float Ts = Tinterval_ctrl_micros / 1e6f;
    int Ndelay = (int)lrintf(delay_s / Ts);   // 四舍五入
    // 保护 Ndelay：至少 0，最多 BUF_LEN-1
    if (Ndelay < 0) Ndelay = 0;
    if (Ndelay >= BUF_LEN) Ndelay = BUF_LEN - 1;

    // 读取延迟索引（相对于当前写指针）
    int idx_d = buf_idx - Ndelay;
    if (idx_d < 0) idx_d += BUF_LEN;

    float tau = kappa * y_buffer[idx_d];

    // 输出
    M1_torque_command = -tau * r_ctl_dir;
    M2_torque_command = +tau * l_ctl_dir;

    M1_torque_command = clip_torque(M1_torque_command);
    M2_torque_command = clip_torque(M2_torque_command);
  }




    if (!imu_init_ok) {
      M1_torque_command = 0;
      M2_torque_command = 0;
    }
                                                        /* aaaaaaaaa */


    // 4) 下发电机命令（MIT/力矩等，按你库的 send_cmd 接口）
    #if MOTOR_BRAND == 0
      for (int i = 0; i < 4; ++i) receive_motor_feedback();
      sig_m1.sig_torque_cmd(M1_torque_command/9.76);
      sig_m2.sig_torque_cmd(M2_torque_command/9.76);
    #else
      for (int i = 0; i < 4; ++i) receive_motor_feedback();
      // TMOTOR 下发（扭矩 only）
      sig_m1.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M1_torque_command);
      sig_m2.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M2_torque_command);
    #endif
  

    // 5) 记录（可扩展）：与你给的“可扩展 SD 打印”一致
    if (logger.isOpen()) {
      // 1) time (ms)
      logger.print(current_time / 1000UL); logger.print(',');

      // 2) floats，保留4位小数（按你原始列）
      logger.print(imu.RTx, 4);                logger.print(',');
      logger.print(imu.LTx, 4);                logger.print(',');
      logger.print(imu.TX1, 4);                logger.print(',');
      logger.print(imu.TX2, 4);                logger.print(',');
      logger.print(imu.TX3, 4);                logger.print(',');
      logger.print(imu.TX4, 4);                logger.print(',');
      logger.print(RLTx_delay[doi], 4);        logger.print(',');
      logger.print(torque_delay[doi], 4);      logger.print(',');
      logger.print(0.0f, 4);              logger.print(',');
      logger.print(0.0f, 4);              logger.print(',');
      logger.print(0.0f, 4);  logger.print(',');
      logger.print(0.0f, 4); logger.print(',');
      logger.print(M1_torque_command, 4);      logger.print(',');
      logger.print(M2_torque_command, 4);      logger.print(',');
      logger.print(0.0f, 4);         logger.print(',');
      logger.print(imu.RTAVx, 4);              logger.print(',');
      logger.print(imu.LTAVx, 4);              logger.print(',');
      logger.print(0.0f, 4);               logger.print(',');
      logger.print(0.0f, 4);                   logger.print(',');
      logger.print(0.0f, 4);                   logger.print(',');
      if (logtag_valid) {
        logger.print(logtag);
        logger.print(',');
        if (!logtag_persist) {
          logtag_valid = false;  // 只打印一次则清掉
        }
      } else {
        logger.print(0.0f, 4);
        logger.print(',');
      }
      
      logger.print(0.0f, 4);                   logger.print(',');
      logger.print(0.0f, 4);
      logger.println();
      static int log_flush_count = 0;
      if (++log_flush_count >= 10) {   // 每10行 flush 一次
        logger.flush();
        log_flush_count = 0;
      }
    }


  if (current_time - previous_time_ble >= Tinterval_ble_micros) {
    previous_time_ble = current_time;
    Transmit_ble_Data();
  }



  
  // === 10 Hz 串口打印 ===
  if (current_time - prev_print_us >= PRINT_INTERVAL_US) {
    prev_print_us = current_time;

    // 这里如果你有 smooth_gate(...) 就直接用；没有就先直接打印原值或留 0
    #if DEBUG_PRINT
    // Serial.printf("RTx=%.2f°  LTx=%.2f°  |  fR=%.2fHz  fL=%.2fHz  avg=%.2fHz\r\n",
    //   imu.RTx, imu.LTx,
    //   cycle_ms, ext_ms_L, ext_ms_R);
    Serial.printf("fR=%.2fHz  fL=%.2fHz  avg=%.2fHz  |  ωL=%.2frad/s  ωR=%.2frad/s  |  τL=%.2f  τR=%.2f\r\n",
      imu.RTx, imu.LTx, max_torque_cfg,
      imu.RTx, imu.LTx,
      M1_torque_command/9.76, M1_torque_command/9.76);
    // Serial.printf("Rescale=%.2f  Flex=%.2f  Ext=%.2f  |  CmdM1=%.2f  CmdM2=%.2f  |  Delay=%.1fms\r\n",
    //   Rescaling_gain, Flex_Assist_gain, Ext_Assist_gain,
    //   M1_torque_command, M2_torque_command,
    //   Assist_delay_gain);
    #endif
  }
}


















float estimateFreqFromRLTx(float signal, unsigned long nowMicros) {
  // 参数可调
  const float HYST_POS = +2.0f;        // 上阈值 (+2°)
  const float HYST_NEG = -2.0f;        // 下阈值 (-2°)
  const unsigned long MIN_DT_US = 250000;   // 最短周期 0.25s → 最大 4Hz
  const unsigned long MAX_DT_US = 2000000;  // 最长周期 2.0s → 最小 0.5Hz

  // 内部状态
  static bool armed = false;             
  static unsigned long lastCross = 0;
  static float freqHz = 0.0f;

  // 1️⃣ 当信号跌到负区时武装
  if (!armed && signal <= HYST_NEG) {
    armed = true;
  }

  // 2️⃣ 从负区上升到正区时触发一次周期测量
  if (armed && signal >= HYST_POS) {
    unsigned long dt = nowMicros - lastCross;

    if (lastCross > 0 && dt >= MIN_DT_US && dt <= MAX_DT_US) {
      freqHz = 1.0e6f / (float)dt;
    }

    lastCross = nowMicros;
    armed = false;  // 等待下一个下降→上升
  }

  return freqHz;
}





// 永远不会动的实现##############################################



void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("CAN bus setup done...");
  delay(200);
}

void initial_Sig_motor() {
  #if MOTOR_BRAND == 0
    sig_m1.sig_torque_ctl_mode_start();    
    delay(200);   
    sig_m2.sig_torque_ctl_mode_start();        
    sig_m1.sig_motor_start();    
    sig_m1.request_pos_vel();    
    sig_m2.sig_motor_start();    
    sig_m2.request_pos_vel();     
    sig_m1.sig_torque_cmd(0.01);    
    delay(200);    
    // sig_m2.request_torque();   
    sig_m2.sig_torque_cmd(0.01);      
    delay(200);   
    for (int i =0; i < 1000; i++)
    {
      receive_motor_feedback();     
    }
    delay(1000);   
    initial_pos_1 = sig_m1.pos;     
    initial_pos_2 = sig_m2.pos;     
    delay(500);  
    /////// command initial setting ///////
    M1_torque_command = 0.0;         
    M2_torque_command = 0.0;   
  #else
    // TMOTOR 初始化逻辑（基于你之前的 tmotor 片段）
    sig_m1.enter_control_mode(); // 若 tmotor 没有此函数，改为 sig_m1.send_cmd(0,...)
    delay(50);
    sig_m2.enter_control_mode();
    delay(50);
    sig_m1.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    sig_m2.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    uint32_t t0 = millis();
    while (millis() - t0 < 500) {
      CAN_message_t mr;
      while (Can3.read(mr)) {
        sig_m1.unpack_reply(mr);
        sig_m2.unpack_reply(mr);
      }
    }
    initial_pos_1 = sig_m1.pos;
    initial_pos_2 = sig_m2.pos;
    M1_torque_command = 0.0;
    M2_torque_command = 0.0;
    Serial.println("TMOTOR init done.");
  #endif
  }
  


// 统一接收反馈：SIG 与 TMOTOR 共用一个接口
void receive_motor_feedback() {
  #if MOTOR_BRAND == 0
      // ---- SIG 路径：按 msgR.id 解析 pos/iq 等 ----
      CAN_message_t msgR;
      while (Can3.read(msgR)) {
          switch (msgR.id) {
              case ID_M1_POSVEL:
                  sig_m1.unpack_pos_vel(msgR, initial_pos_1);
                  break;
              case ID_M1_IQ: {
                  float iq = *(float *)&msgR.buf[4];
                  sig_m1.torque = iq * KT_1;
                  break;
              }
              case ID_M2_POSVEL:
                  sig_m2.unpack_pos_vel(msgR, initial_pos_2);
                  break;
              case ID_M2_IQ: {
                  float iq = *(float *)&msgR.buf[4];
                  sig_m2.torque = iq * KT_2;
                  break;
              }
              default:
                  // 如果有其它 SIG 消息也可能在这里处理
                  break;
          }
      }
      // SIG：把实测扭矩同步到全局测量变量（如果你还同时用sig_m?.torque也可以）
      M1_torque_meas = sig_m1.torque;
      M2_torque_meas = sig_m2.torque;
  
  #else
      // ---- TMOTOR 路径：直接 unpack_reply() 并同步 torque 字段 ----
      CAN_message_t mr;
      while (Can3.read(mr)) {
          sig_m1.unpack_reply(mr);
          sig_m2.unpack_reply(mr);
      }
      // 同步实测扭矩（Nm）
      M1_torque_meas = sig_m1.torque;
      M2_torque_meas = sig_m2.torque;
  #endif
  }
  
/******************** BLE：接收 ********************/
/******************** BLE：接收（带占位、32字节帧） ********************/
void Receive_ble_Data() {
  static uint8_t hdr[3] = {0};

  while (Serial5.available()) {

    // 1) 帧头滚动窗口，寻找 0xA5 0x5A <len>
    hdr[0] = hdr[1];
    hdr[1] = hdr[2];
    hdr[2] = Serial5.read();

    const uint8_t FRAME_LEN = 32;         // 整帧长度（含头）
    const uint8_t PAYLOAD_LEN = FRAME_LEN - 3; // 去掉3字节头，剩余payload
    if (!(hdr[0] == 0xA5 && hdr[1] == 0x5A && hdr[2] == FRAME_LEN)) {
      continue;
    }

    // 2) 确认串口里够一整帧payload
    if (Serial5.available() < PAYLOAD_LEN) {
      return; // 数据没全，不阻塞
    }

    // 3) 读payload进 data_rs232_rx[0 .. PAYLOAD_LEN-1]
    Serial5.readBytes((char*)data_rs232_rx, PAYLOAD_LEN);


    // 在接收代码里加判断
    if (data_rs232_rx[0] == 'L' && data_rs232_rx[1] == 'G') {
      uint8_t n = data_rs232_rx[2];
      if (n > 10) n = 10;
      memset(logtag, 0, sizeof(logtag));
      memcpy(logtag, &data_rs232_rx[3], n);
      logtag_valid = true;
      logtag_persist = (data_rs232_rx[13] & 0x01);
    
      Serial.print("LOGTAG received: ");
      Serial.println(logtag);
      return;  // 这帧是 LOGTAG，不走原有参数解析
    }
    
    /****************************************
     * 4) 解析 payload -> new_* 临时变量
     *    映射规则见上表
     ****************************************/

    float new_Rescaling_gain    = rd_i16((uint8_t*)data_rs232_rx, 0)  / 100.0f;   // 占位
    float new_Flex_Assist_gain  = rd_i16((uint8_t*)data_rs232_rx, 2)  / 100.0f;   // 占位
    float new_Ext_Assist_gain   = rd_i16((uint8_t*)data_rs232_rx, 4)  / 100.0f;   // 占位

    float new_Assist_delay_gain = (float)((uint8_t)data_rs232_rx[6]);            // 0~99

    int8_t new_phase_offset_L   = (int8_t)data_rs232_rx[7];                      // 占位
    int8_t new_phase_offset_R   = (int8_t)data_rs232_rx[8];                      // 占位

    float new_gate_k            = rd_i16((uint8_t*)data_rs232_rx,  9) / 100.0f;  // 0~10
    float new_gate_p_on         = rd_i16((uint8_t*)data_rs232_rx, 11) / 100.0f;  // 0~10

    float new_lead_frac         = rd_i16((uint8_t*)data_rs232_rx, 13) / 1000.0f; // 0~0.1

    float new_ext_phase_frac_L  = rd_i16((uint8_t*)data_rs232_rx, 15) / 1000.0f; // 0~0.5
    float new_ext_phase_frac_R  = rd_i16((uint8_t*)data_rs232_rx, 17) / 1000.0f; // 0~0.5

    float new_ext_gain          = rd_i16((uint8_t*)data_rs232_rx, 19) / 100.0f;  // -3~3
    float new_scale_all         = rd_i16((uint8_t*)data_rs232_rx, 21) / 100.0f;  // -1~1
    float new_max_torque_cfg = rd_i16((uint8_t*)data_rs232_rx, 23) / 100.0f;  // ★ [23..24]

    // bytes [23..28] （甚至 [23..PAYLOAD_LEN-1]）为占位/保留
    // 将来可以在这里继续解析更多字段，比如 max_torque_cfg 等
    // 现在我们就先忽略它们:
    // int16_t future_param = rd_i16((uint8_t*)data_rs232_rx, 23) / 100.0f;  // 示例
    // ...

    /****************************************
     * 5) 限幅保护
     ****************************************/

    new_Rescaling_gain    = clampf(new_Rescaling_gain,    0.0f, 10.0f);
    new_Flex_Assist_gain  = clampf(new_Flex_Assist_gain,  0.0f, 10.0f);
    new_Ext_Assist_gain   = clampf(new_Ext_Assist_gain,   0.0f, 10.0f);

    new_Assist_delay_gain = clampf(new_Assist_delay_gain, 0.0f, 99.0f);

    // 如果你想限制相位偏移，比如 [-90, 90]，可以在这里加
    // new_phase_offset_L = (int8_t)constrain(new_phase_offset_L, -90, 90);
    // new_phase_offset_R = (int8_t)constrain(new_phase_offset_R, -90, 90);

    new_gate_k            = clampf(new_gate_k,            0.0f, 50.0f);
    new_gate_p_on         = clampf(new_gate_p_on,         0.0f, 10.0f);

    new_lead_frac         = clampf(new_lead_frac,         0.0f, 0.1f);
    new_ext_phase_frac_L  = clampf(new_ext_phase_frac_L,  0.0f, 0.5f);
    new_ext_phase_frac_R  = clampf(new_ext_phase_frac_R,  0.0f, 0.5f);

    // 如果你只允许负向助力（不推反向），把上界改成 0.0f
    new_ext_gain          = clampf(new_ext_gain,         -3.0f, 3.0f);
    new_scale_all         = clampf(new_scale_all,        -1.0f, 1.0f);
    new_max_torque_cfg = clampf(new_max_torque_cfg, 0.0f, 15.0f);

    /****************************************
     * 6) 根据开关, 决定是否写入全局控制变量
     ****************************************/
#if GUI_WRITE_ENABLE
    Rescaling_gain    = new_Rescaling_gain;
    Flex_Assist_gain  = new_Flex_Assist_gain;
    Ext_Assist_gain   = new_Ext_Assist_gain;

    delay_s = new_Assist_delay_gain;
    phase_offset_L    = new_phase_offset_L;
    phase_offset_R    = new_phase_offset_R;

    kappa            = new_gate_k;
    gate_p_on         = new_gate_p_on;

    lead_frac         = new_lead_frac;
    ext_phase_frac_L  = new_ext_phase_frac_L;
    ext_phase_frac_R  = new_ext_phase_frac_R;

    ext_gain          = new_ext_gain;
    scale_all         = new_scale_all;
    max_torque_cfg    = new_max_torque_cfg;
#else
    // LOCKED模式: 我们不更新全局参数
    // 你可以在这里记录new_*用于debug (比如放到一个shadow_*里)
#endif
  }
}

/******************** BLE：发送（与你一致） ********************/
void Transmit_ble_Data() {
  const uint8_t FRAME_LEN = 32;
  // 这里用 millis 作为时间基准（0.01s 精度）


  // 0) 时间戳：厘秒（0.01s）, 用 uint16_t 发送，小端；~655.35s回绕
  uint16_t t_cs = (uint16_t)((millis() / 10) & 0xFFFF);

  // 1) 量化为 int16 = 值 * 100（小端）
  int16_t L_ang100   = (int16_t)roundf(imu.LTx * 100.0f);
  int16_t R_ang100   = (int16_t)roundf(imu.RTx * 100.0f);
  #if MOTOR_BRAND == 0
    int16_t L_tau100 = (int16_t)roundf(sig_m1.torque * 100.0f);
    int16_t R_tau100 = (int16_t)roundf(sig_m2.torque * 100.0f);
  #else
    int16_t L_tau100 = (int16_t)roundf(sig_m1.torque * 100.0f);
    int16_t R_tau100 = (int16_t)roundf(sig_m2.torque * 100.0f);
  #endif

  int16_t L_cmd100   = (int16_t)roundf(M1_torque_command * 100.0f);
  int16_t R_cmd100   = (int16_t)roundf(M2_torque_command * 100.0f);
  int16_t mt100 = (int16_t)roundf(max_torque_cfg * 100.0f);

  // 新增：步态频率 Hz×100 → int16，小端；典型 0.5~2.5 Hz
  extern float gait_freq; // 你在别处计算好的步频 [Hz]
  int16_t gf100 = (int16_t)roundf(gait_freq * 100.0f);

  // 2) 帧头
  data_ble[0] = 0xA5;
  data_ble[1] = 0x5A;
  data_ble[2] = FRAME_LEN;

  // 3) 载荷（从 index=3 开始，以下是小端）
  data_ble[3]  = (uint8_t)(t_cs & 0xFF);
  data_ble[4]  = (uint8_t)((t_cs >> 8) & 0xFF);

  data_ble[5]  = (uint8_t)(L_ang100 & 0xFF);
  data_ble[6]  = (uint8_t)((L_ang100 >> 8) & 0xFF);
  data_ble[7]  = (uint8_t)(R_ang100 & 0xFF);
  data_ble[8]  = (uint8_t)((R_ang100 >> 8) & 0xFF);

  data_ble[9]  = (uint8_t)(L_tau100 & 0xFF);
  data_ble[10] = (uint8_t)((L_tau100 >> 8) & 0xFF);
  data_ble[11] = (uint8_t)(R_tau100 & 0xFF);
  data_ble[12] = (uint8_t)((R_tau100 >> 8) & 0xFF);

  data_ble[13] = (uint8_t)(L_cmd100 & 0xFF);
  data_ble[14] = (uint8_t)((L_cmd100 >> 8) & 0xFF);
  data_ble[15] = (uint8_t)(R_cmd100 & 0xFF);
  data_ble[16] = (uint8_t)((R_cmd100 >> 8) & 0xFF);

  // 4) IMU 初始化状态 + 当前 max_torque_cfg（Nm×100）——保持原位置
  data_ble[17] = imu_init_ok ? 1 : 0;

  data_ble[18] = (uint8_t)(mt100 & 0xFF);
  data_ble[19] = (uint8_t)((mt100 >> 8) & 0xFF);

  // 5) 新增：gait_freq at payload[20..21]（即 data_ble[23..24]）
  data_ble[20] = g_init_status & 0xFF;   // 只发低 8 位够用了
  data_ble[21] = 0;  // payload[18] 仍空着
  data_ble[22] = 0;  // payload[19] 仍空着

  data_ble[23] = (uint8_t)(gf100 & 0xFF);
  data_ble[24] = (uint8_t)((gf100 >> 8) & 0xFF);

  // 6) 其余占位清零（payload[21+] -> data_ble[25..31]）
  for (int i = 25; i <= 31; ++i) data_ble[i] = 0;

  // 7) 发送
  Serial5.write((uint8_t*)data_ble, (size_t)FRAME_LEN);

}

/******************** 其它小工具（保留你的签名） ********************/
double derivative(double dt, double derivative_prev[], double *actual_in_ptr, double *prev_in_ptr){
  int i;
  double diff = 0.0, diff_sum = 0.0;
  if (dt != 0.0){
    for (i = 0; i < 3; i++){
      diff_sum += derivative_prev[i];
    }
    diff = (diff_sum + (*actual_in_ptr - *prev_in_ptr) / dt) / (i + 1);
  } else {
    diff = derivative_prev[3];
  }
  return diff;
}
