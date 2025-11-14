/* -------------  IMU_Adapter.cpp ------------- */
#include "IMU_Adapter.h"

// —— 放到 IMU_Adapter.cpp 顶部 —— //
static inline float d2r(float d){ return d * 0.017453292519943295f; }
static inline float r2d(float r){ return r * 57.29577951308232f; }
static float circ_mean_deg(const float *a, int n){
  double sx=0, sy=0; for(int i=0;i<n;i++){ sx+=cos(d2r(a[i])); sy+=sin(d2r(a[i])); }
  return (float)r2d(atan2(sy,sx));
}
static float circ_std_deg(const float *a, int n){
  double sx=0, sy=0; for(int i=0;i<n;i++){ sx+=cos(d2r(a[i])); sy+=sin(d2r(a[i])); }
  double R = sqrt(sx*sx+sy*sy)/n; if(R<=1e-6) return 180.f;
  double var = -2.0*log(R); return (float)r2d(sqrt(var));
}



float deriv(float cur, float dt)
{
    static float prev = 0.0f;
    float vel = (cur - prev) / dt;
    prev = cur;
    return vel;
}

/* ------- 1. 初始化 -------- */
void IMU_Adapter::INIT() {
    SerialImuLeft.begin(460800);
    SerialImuRight.begin(460800);
    SerialImuP1.begin(460800);
    SerialImuP2.begin(460800);
    SerialImuP3.begin(460800);
    SerialImuP4.begin(460800);

    /* 唤醒 & 设置参数（左右各一条） */
    Cmd_03_Left();             delay(200);
    Cmd_03_Right();            delay(200);
    Cmd_03_1();            delay(200);
    Cmd_03_2();            delay(200);
    Cmd_03_3();            delay(200);
    Cmd_03_4();            delay(200);
/* 左右 IMU 统一改为 */
/* 只改 Tag：0x44  */
/* 保留角速度，库默认支持 10 B 帧 */
    Cmd_12_Left (5,255,0,0,3,100, 2,4,9, 0x44);  // Euler-X + Gyro-X
    Cmd_12_Right(5,255,0,0,3,100, 2,4,9, 0x44);
    Cmd_12_1(5,255,0,0,3,100, 2,4,9, 0x44);
    Cmd_12_2(5,255,0,0,3,100, 2,4,9, 0x44);
    Cmd_12_3(5,255,0,0,3,100, 2,4,9, 0x44);
    Cmd_12_4(5,255,0,0,3,100, 2,4,9, 0x44);
    delay(200);

    Cmd_19_Left();             delay(200);
    Cmd_19_Right();            delay(200);
    Cmd_19_1();            delay(200);
    Cmd_19_2();            delay(200);
    Cmd_19_3();            delay(200);
    Cmd_19_4();            delay(200);
}

void IMU_Adapter::INIT_MEAN() {
    if (offset_locked) {
        Serial.println("[IMU] INIT_MEAN skipped (already locked).");
        return;
    }

    // 预热，清空缓冲，避免脏数据
    while (SerialImuLeft.available())  SerialImuLeft.read();
    while (SerialImuRight.available()) SerialImuRight.read();
    while (SerialImuP1.available()) SerialImuP1.read();
    while (SerialImuP2.available()) SerialImuP2.read();
    while (SerialImuP3.available()) SerialImuP3.read();
    while (SerialImuP4.available()) SerialImuP4.read();

    delay(2000);

    const int N = 200;
    float sumL = 0, sumR = 0;
    float sum1 = 0, sum2 = 0;
    float sum3 = 0, sum4 = 0;

    for (int i=0; i<N; i++) {
        READ();
        sumL += AngleXLeft;
        sumR += AngleXRight;
        sum1 += AngleX1;
        sum2 += AngleX2;
        sum3 += AngleX3;
        sum4 += AngleX4;
        delay(5);
    }

    float offL_tmp = sumL / N;
    float offR_tmp = sumR / N;
    float off1_tmp = sum1 / N;
    float off2_tmp = sum2 / N;
    float off3_tmp = sum3 / N;
    float off4_tmp = sum4 / N;
    // 合理性检查：不在 60–120° 就兜底设为 97
    auto sane = [](float x){ return (x >= 60.f && x <= 120.f) ? x : 97.f; };

    offL = sane(offL_tmp);
    offR = sane(offR_tmp);
    off1 = sane(off1_tmp);
    off2 = sane(off2_tmp);
    off3 = sane(off3_tmp);
    off4 = sane(off4_tmp);
    offset_locked = true;

    Serial.printf("[IMU] INIT_MEAN done: rawL=%.2f rawR=%.2f -> offL=%.2f offR=%.2f\n",
                  offL_tmp, offR_tmp, offL, offR);
    Serial.printf("[IMU] INIT_MEAN done: raw1=%.2f raw2=%.2f -> off1=%.2f off2=%.2f\n",
                    off1_tmp, off2_tmp, off1, off2);
    Serial.printf("[IMU] INIT_MEAN done: raw3=%.2f raw4=%.2f -> off3=%.2f off4=%.2f\n",
                        off3_tmp, off4_tmp, off3, off4);
}

void IMU_Adapter::REZERO_LR(uint16_t warmup_ms, uint16_t N) {
    // 只清左右两路的串口缓冲，避免脏数据
    while (SerialImuLeft.available())  SerialImuLeft.read();
    while (SerialImuRight.available()) SerialImuRight.read();

    delay(warmup_ms);   // 给模块一点恢复时间（400~600ms够用）

    float sumL = 0.0f, sumR = 0.0f;
    for (uint16_t i = 0; i < N; ++i) {
        READ();                 // 统一读一次（会更新六路），但我们只用 L/R
        sumL += AngleXLeft;
        sumR += AngleXRight;
        delay(5);
    }

    // ✅ 就地置零：直接把当前均值作为偏置；不做 60–120° 夹紧
    float offL_tmp = sumL / (float)N;
    float offR_tmp = sumR / (float)N;

    offL = offL_tmp;
    offR = offR_tmp;

    Serial.printf("[IMU] REZERO_LR: rawL=%.2f rawR=%.2f -> offL=%.2f offR=%.2f\r\n",
                  offL_tmp, offR_tmp, offL, offR);
}


/* ------- 3. 每次循环更新 ------- */
/* ------- 3. 每次循环更新 ------- */
void IMU_Adapter::READ(){
    while(SerialImuLeft.available())
        pollOneByte(SerialImuLeft , Cmd_GetPkt_Left);
    while(SerialImuRight.available())
        pollOneByte(SerialImuRight, Cmd_GetPkt_Right);
    while(SerialImuP1.available())
        pollOneByte(SerialImuP1, Cmd_GetPkt_1);
    while(SerialImuP2.available())
        pollOneByte(SerialImuP2, Cmd_GetPkt_2);
    while(SerialImuP3.available())
        pollOneByte(SerialImuP3, Cmd_GetPkt_3);
    while(SerialImuP4.available())
        pollOneByte(SerialImuP4, Cmd_GetPkt_4);
    LTx   = AngleXLeft  - offL;
    RTx   = AngleXRight - offR;
    LTAVx = VelXLeft;
    RTAVx = VelXRight;


    TX1   = AngleX1  - off1;
    TX2   = AngleX2 - off2;
    TX3   = AngleX3 - off3;
    TX4   = AngleX4 - off4;
    
    VTX1 = VelX1;
    VTX2 = VelX2;
    VTX3 = VelX3;
    VTX4 = VelX4;
// #if 1          // ← 改成 1 才打印！默认关
//     static uint8_t dbg = 0;          // 每 10 帧打印一次
//     if (++dbg >= 100) {
//         dbg = 0;
//         if (Serial.availableForWrite() > 64) {      // **非阻塞**
//             Serial.printf("Lθ %.1f , Rθ %.1f | Lω %.1f , Rω %.1f\r\n",
//                           LTx, RTx, LTAVx, RTAVx);
//         }
//     }
// #endif
}


/* ------- 辅助：一字节进状态机 ------- */
void IMU_Adapter::pollOneByte(HardwareSerial& port,
                              uint8_t (&pktFunc)(uint8_t))
{
    U8 byte = port.read();
    pktFunc(byte);         // im948_CMD 自带解包状态机
}


bool IMU_Adapter::ZeroToReference(float refL_deg, float refR_deg,float ref1_deg,float ref2_deg,float ref3_deg,float ref4_deg,
    uint16_t N, uint32_t warmup_ms, uint32_t timeout_ms)
{
while (SerialImuLeft.available())  SerialImuLeft.read();
while (SerialImuRight.available()) SerialImuRight.read();
while (SerialImuP1.available()) SerialImuP1.read();
while (SerialImuP2.available()) SerialImuP2.read();
while (SerialImuP3.available()) SerialImuP3.read();
while (SerialImuP4.available()) SerialImuP4.read();

delay(warmup_ms);

const float GYRO_TH = 3.0f;   // °/s，判静止
const float JUMP_TH = 15.0f;  // °，单点突变(包错/抖动)剔除
const float STD_TH  = 1.5f;   // °，整体稳定性门限

const uint32_t t0 = millis();
std::vector<float> L, R; L.reserve(N); R.reserve(N);
std::vector<float> ak1, ak2; ak1.reserve(N); ak2.reserve(N);
std::vector<float> ak3, ak4; ak3.reserve(N); ak4.reserve(N);
float lastL = AngleXLeft, lastR = AngleXRight;
float last1 = AngleX1, last2 = AngleX2;
float last3 = AngleX3, last4 = AngleX4;



while (L.size() < N && (millis() - t0) < timeout_ms) {
READ();  // 更新 AngleX*/VelX*
// —— 静止门（两侧都要稳）——
if (fabsf(VelXLeft)  > GYRO_TH) { delay(5); continue; }
if (fabsf(VelXRight) > GYRO_TH) { delay(5); continue; }
if (fabsf(VelX1) > GYRO_TH) { delay(5); continue; }
if (fabsf(VelX2) > GYRO_TH) { delay(5); continue; }
if (fabsf(VelX3) > GYRO_TH) { delay(5); continue; }
if (fabsf(VelX4) > GYRO_TH) { delay(5); continue; }
// —— 单点异常门 ——（避免偶发错包、跳变）
if (!L.empty() && fabsf(AngleXLeft  - lastL) > JUMP_TH) { delay(5); continue; }
if (!R.empty() && fabsf(AngleXRight - lastR) > JUMP_TH) { delay(5); continue; }
if (!ak1.empty() && fabsf(AngleX1  - last1) > JUMP_TH) { delay(5); continue; }
if (!ak2.empty() && fabsf(AngleX2 - last2) > JUMP_TH) { delay(5); continue; }
if (!ak3.empty() && fabsf(AngleX3  - last3) > JUMP_TH) { delay(5); continue; }
if (!ak4.empty() && fabsf(AngleX4 - last4) > JUMP_TH) { delay(5); continue; }

L.push_back(AngleXLeft);  lastL = AngleXLeft;
R.push_back(AngleXRight); lastR = AngleXRight;
ak1.push_back(AngleX1); lastR = AngleX1;
ak2.push_back(AngleX2); lastR = AngleX2;
ak3.push_back(AngleX3); lastR = AngleX3;
ak4.push_back(AngleX4); lastR = AngleX4;
delay(5);
}
if (L.size() < N*0.8f) return false;  // 数据不足

float mL = circ_mean_deg(L.data(), (int)L.size());
float mR = circ_mean_deg(R.data(), (int)R.size());
float sL = circ_std_deg (L.data(), (int)L.size());
float sR = circ_std_deg (R.data(), (int)R.size());


float m1 = circ_mean_deg(ak1.data(), (int)ak1.size());
float m2 = circ_mean_deg(ak2.data(), (int)ak2.size());
float m3 = circ_mean_deg(ak3.data(), (int)ak3.size());
float m4 = circ_mean_deg(ak4.data(), (int)ak4.size());
float s1 = circ_std_deg (ak1.data(), (int)ak1.size());
float s2 = circ_std_deg (ak2.data(), (int)ak2.size());
float s3 = circ_std_deg (ak3.data(), (int)ak3.size());
float s4 = circ_std_deg (ak4.data(), (int)ak4.size());

if (sL > STD_TH || sR > STD_TH) return false; // 不够稳

// 让“参考角=0” => 站立为 0
offL = mL - refL_deg;   // refL_deg/refR_deg 你现在传的是 0
offR = mR - refR_deg;
off1 = m1 - ref1_deg;   // refL_deg/refR_deg 你现在传的是 0
off2 = m2 - ref2_deg;
off3 = m3 - ref3_deg;   // refL_deg/refR_deg 你现在传的是 0
off4 = m4 - ref4_deg;
return true;
}
