/******************** 必要的包含 ********************/
#include <Arduino.h>
#include <math.h>
#include <cstring>
#include <iomanip> 
#include <FlexCAN_T4.h>
#include "Serial_Com.h"
#include "IMU_Adapter.h"
#include "MovingAverage.h"
#include "Motor_Control_Tmotor.h"
#include "SdFat.h"
#include "RingBuf.h"
#include "sdlogger.h" 
#ifndef DEBUG_PRINT
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
int l_ctl_dir = 1;      //确实是左脚，1是向上
int r_ctl_dir = -1;     //确实是右脚，1是向上



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
void receive_tm_feedback();
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
      "M1_prev_afterslew,M2_prev_afterslew,extra3,extra4,extra5"
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
float gate_k            = 1.0f;
float gate_p_on         = 2.5f;

// ===== Extension 复制/回放配置（按你要求的默认值，gui不修改）=====
static const bool  use_ext_copy   = true;   // 开关
volatile bool  ext_enable_L = true;        // 只在左腿复制 extension
volatile bool  ext_enable_R = true;       // 右腿默认不复制
/******************** loop ********************/
void loop() {
  imu.READ();
  Serial_Com.READ2();
  current_time = micros();
  const float Ts = (float)Tinterval_ctrl_micros / 1e6f;  // 控制周期 (s)

  // Serial.printf("RTx=%.2f° LTx=%.2f°\r\n", imu.RTx, imu.LTx);
 // === 简单失效/恢复判定 ===
  if (fabs(imu.RTx) > 80.0f || fabs(imu.LTx) > 80.0f) {
    if (imu_init_ok) {
      Serial.println("[WARN] IMU角度超过 ±80°，关闭辅助");
    }
    imu_init_ok = 0;
  } else {
    if (!imu_init_ok) {
      Serial.println("[OK] IMU角度回到安全范围，恢复辅助");
    }
    imu_init_ok = 1;
  }
  // —— 控制频率节拍（例如 100Hz）——
  if (current_time - previous_time >= Tinterval_ctrl_micros) {
    previous_time = current_time;
    // 1) 接收 CAN 回包，刷新电机反馈
    receive_tm_feedback();

    // 2) 接收 BLE 下发（非阻塞，随时吸收）
    Receive_ble_Data();
    RLTx = imu.RTx - imu.LTx;
    LTx_filtered_last = LTx_filtered;
    LTx_filtered      = 0.9f * LTx_filtered_last + 0.1f * imu.LTx;
    RTx_filtered_last = RTx_filtered;
    RTx_filtered      = 0.9f * RTx_filtered_last + 0.1f * imu.RTx;
    RLTx_filtered = RTx_filtered - LTx_filtered;
    /************ 更新步态周期与初始化状态 ************/
    static float RL_prev = 0.0f;
    static uint32_t t_last_cross = 0;

    const float RL_HYST_DEG = 3.0f;   // 跨零滞回阈值 ±3°
    const float TGAIT_MIN_MS = 350.0f;
    const float TGAIT_MAX_MS = 1800.0f;
    const float TGAIT_SMOOTH_A = 0.35f;  // 平滑系数

    // 检测负->正跨越
    if (RL_prev <= -RL_HYST_DEG && RLTx_filtered >= RL_HYST_DEG) {
      uint32_t now_ms = current_time / 1000UL;

      if (t_last_cross > 0) {
        float dt = (float)(now_ms - t_last_cross);
        if (dt >= TGAIT_MIN_MS && dt <= TGAIT_MAX_MS) {
          gait_inited = 1;  // ✅ 标记已检测到步态
          // 指数平滑周期
        }
      }
      t_last_cross = now_ms;
    }
    /************ 更新步态周期与初始化状态 ************/


    RLTx_delay[doi] = RLTx_filtered;
    gait_freq = estimateFreqFromRLTx(RLTx_filtered, current_time);

// ################################
    int Assist_delay_dynamic = Assist_delay_gain;  // 基础值来自 GUI
    const float BASE_FREQ = 0.7f;      // 基准步频 (Hz)
    const int   BASE_DELAY = Assist_delay_gain;  // 当前 GUI 设置对应 0.7Hz 时的基准
    if (gait_freq > BASE_FREQ) {
      Assist_delay_dynamic = (int)(BASE_DELAY * (BASE_FREQ / gait_freq));
      if (Assist_delay_dynamic < 5) Assist_delay_dynamic = 5;   // 最小延迟样本保护
    }
    delayindex = doi - Assist_delay_dynamic;
    if (delayindex < 0)              delayindex += 100;
    else if (delayindex >= 100)      delayindex -= 100;
    // === 各腿相位偏移（可保留，默认0不影响）===
    int idx_L = (delayindex + phase_offset_L + 100) % 100;
    int idx_R = (delayindex + phase_offset_R + 100) % 100;
    // === 推进缓冲指针 ===
    doi = (doi + 1) % 100;

    /* ---------- 原始对称扭矩（极简版） ---------- */
    const float phL = RLTx_delay[idx_L];
    const float phR = RLTx_delay[idx_R];

    float tau_raw_L = 0.0f;
    float tau_raw_R = 0.0f;

    // 左腿（RLTx>=0 表示右腿伸展、左腿屈曲 → 取负号）
    if (fabsf(phL) <= 120.0f)
        tau_raw_L = (phL >= 0.0f ? -1.0f : +1.0f) * phL;

    // 右腿（RLTx>=0 表示右腿伸展 → 取正号）
    if (fabsf(phR) <= 120.0f)
        tau_raw_R = (phR >= 0.0f ? +1.0f : -1.0f) * phR;

    /* --- 每腿功率门控输入 --- */
    // 注意：右腿速度取负
    RLTx = imu.RTx - imu.LTx;
    LTx_filtered_last = LTx_filtered;
    LTx_filtered      = 0.9f * LTx_filtered_last + 0.1f * imu.LTx;
    RTx_filtered_last = RTx_filtered;
    RTx_filtered      = 0.9f * RTx_filtered_last + 0.1f * imu.RTx;
    RLTx_filtered = RTx_filtered - LTx_filtered;

    /* --- 角速度(°/s) ---> rad/s 更物理，不过比例因子无关宏旨 --- 而且不能用imu读取的！！*/
    const float LTx_vel = (LTx_filtered - LTx_filtered_last) * (PI/180.0f) / Ts;
    const float RTx_vel = (RTx_filtered - RTx_filtered_last) * (PI/180.0f) / Ts;
    
    const float xL_raw = tau_raw_L * LTx_vel;
    const float xR_raw = tau_raw_R * (-RTx_vel);


    /********* 功率门控：仅用 lead_frac * T_gait_ms，去掉 gate_lead_ms/leadms *********/

    // 1) 计算门控超前时间（ms）
    //    - gait 已初始化：lead_ms = lead_frac * T_gait_ms
    //    - 未初始化：用一个保底 35ms（可按需改）
    //    - 做基本夹紧，避免极端
    float lead_ms = 35.0f;  // fallback
    if (gait_inited && gait_freq > 0.01f) {
      float T_gait_ms = 1000.0f / gait_freq;        // Hz → ms
      lead_ms = lead_frac * T_gait_ms;               // 例如 0.06 * 周期
    }
    if (lead_ms < 5.0f)   lead_ms = 5.0f;
    if (lead_ms > 120.0f) lead_ms = 120.0f;

    // 2) 线性前视预测（基于斜率）
    const float xL_pred = lead_predict(xL_raw, xL_prev, lead_ms, Ts);
    const float xR_pred = lead_predict(xR_raw, xR_prev, lead_ms, Ts);
    xL_prev = xL_raw;   // 记住本帧 raw，下一帧当作“上一帧”
    xR_prev = xR_raw;

    // 3) 平滑门控（阈值在 gate_p_on，斜率由 gate_k）
    const float gate_L = smooth_gate_p(xL_prev, gate_k, gate_p_on);
    const float gate_R = smooth_gate_p(xR_prev, gate_k, gate_p_on);

    // 4) 门后“原始”扭矩（此处仍不施加任何增益，保持线性）
    //    这里的 tau_raw_* 是你上一步“对称扭矩（极简版）”算出来的
    float tau_gate_L = tau_raw_L * gate_L;
    float tau_gate_R = tau_raw_R * gate_R;

    // 5) 一阶低通
    tau_cmd_L_filt = 0.85f * tau_cmd_L_filt + (1.0f - 0.85f) * tau_gate_L;
    tau_cmd_R_filt = 0.85f * tau_cmd_R_filt + (1.0f - 0.85f) * tau_gate_R;

    // 6) 输出给下游（后面再做增益、slew、饱和、下发）
    S_torque_command_left  = tau_cmd_L_filt* scale_all;
    S_torque_command_right = tau_cmd_R_filt* scale_all;

    float S_src_L = tau_cmd_L_filt* scale_all;  // LPF 后
    float S_src_R = tau_cmd_R_filt* scale_all;

    auto keep_if_flex = [](float S, int8_t sign){ return ((sign > 0) ? (S > 0.0f) : (S < 0.0f)) ? S : 0.0f; };
    float flexL_now = keep_if_flex(S_src_L, flex_sign_L);  // 例如 flex_sign_L=-1 代表“负值为屈”
    float flexR_now = keep_if_flex(S_src_R, flex_sign_R);

    int w = ext_i % EXT_BUF_LEN;
    flexL_hist[w] = flexL_now;
    flexR_hist[w] = flexR_now;


    float T_gait_ms1 = 1000.0f / gait_freq;        // Hz → ms

    // —— 计算 extension 延迟时间（ms）= 百分比 × 当前周期 ——
    // 未估到步态时，退回到固定 ms（你原先的 ext_delay_ms）
    float ext_ms_L = ext_phase_frac_L * T_gait_ms1;
    float ext_ms_R = ext_phase_frac_R * T_gait_ms1;
    // Serial.printf("RTx=%.2f°  LTx=%.2f°  |  fR=%.2fHz  fL=%.2fHz  avg=%.2fHz\r\n",
    //   imu.RTx, imu.LTx,
    //   T_gait_ms1, ext_ms_L, ext_ms_R);
    // —— 安全夹紧（避免极端）——
    auto clampf = [](float x, float a, float b){ return (x<a)?a:((x>b)?b:x); };
    ext_ms_L = clampf(ext_ms_L, 30.0f, 1200.0f);
    ext_ms_R = clampf(ext_ms_R, 30.0f, 1200.0f);

    // —— ms → 样本数：N = round( ext_ms / Ts ) ——
    // 注意 Ts 是“秒”，所以 / (1000*Ts)
    int extN_L = (int)lrintf(ext_ms_L / (1000.0f * Ts));
    int extN_R = (int)lrintf(ext_ms_R / (1000.0f * Ts));
    if (extN_L >= EXT_BUF_LEN) extN_L = EXT_BUF_LEN - 1;
    if (extN_R >= EXT_BUF_LEN) extN_R = EXT_BUF_LEN - 1;
    if (extN_L < 1) extN_L = 1;
    if (extN_R < 1) extN_R = 1;

    // —— 回读延迟后的 flexion（复制成 extension = 取相反号 × 增益）——
    int rL = ext_i - extN_L;
    int rR = ext_i - extN_R;
    // 正模
    if (rL < 0) rL += ((-rL / EXT_BUF_LEN) + 1) * EXT_BUF_LEN;
    if (rR < 0) rR += ((-rR / EXT_BUF_LEN) + 1) * EXT_BUF_LEN;
    rL %= EXT_BUF_LEN;  rR %= EXT_BUF_LEN;

    float S_L_ext = 0.0f, S_R_ext = 0.0f;
    if (use_ext_copy) {
      if (ext_enable_L) S_L_ext = -ext_gain * flexL_hist[rL];
      if (ext_enable_R) S_R_ext = -ext_gain * flexR_hist[rR];
    }



    // —— 叠加，得到最终 S ——
    //（这里保持你前面 S_src_* 的定义：LPF 后 or 门后）
    S_torque_command_left  = S_src_L + S_L_ext;
    S_torque_command_right = S_src_R + S_R_ext;
    S_torque_command_left = gait_freq*1.2 * S_torque_command_left;
    S_torque_command_right = gait_freq*1.2 * S_torque_command_right;
    // 推进写指针
    ext_i++;










    M1_torque_command = S_torque_command_right * r_ctl_dir;
    M2_torque_command = S_torque_command_left  * l_ctl_dir;
    M1_torque_command = slew(M1_torque_command, M1_prev, torque_rate, Ts);
    M2_torque_command = slew(M2_torque_command, M2_prev, torque_rate, Ts);
    M1_prev = M1_torque_command;
    M2_prev = M2_torque_command;

    /* --- 饱和 --- */
    M1_torque_command = clip_torque(M1_torque_command);
    M2_torque_command = clip_torque(M2_torque_command);


    if (!imu_init_ok) {
      M1_torque_command = 0;
      M2_torque_command = 0;
    }
                                                        /* aaaaaaaaa */


    // 3) TODO: 在这里放入你的“算法/控制器”计算，写入 M1_torque_command / M2_torque_command
    //    —— 当前阶段我们不做任何控制：保持命令=0 或者来自上位机的占位值 ——

    // 4) 下发电机命令（MIT/力矩等，按你库的 send_cmd 接口）
    sig_m1.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M1_torque_command);
    sig_m2.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M2_torque_command);

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
      logger.print(tau_raw_L, 4);              logger.print(',');
      logger.print(tau_raw_R, 4);              logger.print(',');
      logger.print(S_torque_command_left, 4);  logger.print(',');
      logger.print(S_torque_command_right, 4); logger.print(',');
      logger.print(M1_torque_command, 4);      logger.print(',');
      logger.print(M2_torque_command, 4);      logger.print(',');
      logger.print(Rescaling_gain, 4);         logger.print(',');
      logger.print(imu.RTAVx, 4);              logger.print(',');
      logger.print(imu.LTAVx, 4);              logger.print(',');
      logger.print(gait_freq, 4);               logger.print(',');
      logger.print(M1_prev, 4);                   logger.print(',');
      logger.print(M2_prev, 4);                   logger.print(',');
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
      M1_torque_command, M2_torque_command);
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
  sig_m1.enter_control_mode();
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

  Serial.println("tmotor torque-only (t_ff) path ready.");
}


void receive_tm_feedback() {
  CAN_message_t mr;
  while (Can3.read(mr)) {
    sig_m1.unpack_reply(mr);
    sig_m2.unpack_reply(mr);
  }
  // 同步实测扭矩（Nm）
  M1_torque_meas = sig_m1.torque;
  M2_torque_meas = sig_m2.torque;
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

    new_gate_k            = clampf(new_gate_k,            0.0f, 10.0f);
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

    Assist_delay_gain = new_Assist_delay_gain;
    phase_offset_L    = new_phase_offset_L;
    phase_offset_R    = new_phase_offset_R;

    gate_k            = new_gate_k;
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
  int16_t L_tau100   = (int16_t)roundf(sig_m1.torque * 100.0f);
  int16_t R_tau100   = (int16_t)roundf(sig_m2.torque * 100.0f);
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
