#include "Motors3pi.h"

// 单头文件实现宏（只在这个 .ino 里写一次）
#define ENCODERS3PI_IMPLEMENTATION
#include "Encoders3pi.h"

// ================== 默认测试参数（你要更精细就改这里） ==================
// Deadzone：小步进更合适
static const int16_t DZ_PWM_START = 0;
static const int16_t DZ_PWM_END   = 120;
static const int16_t DZ_PWM_STEP  = 2;
static const uint8_t DZ_REPEATS   = 3;
static const uint16_t DZ_WARMUP_MS  = 250;
static const uint16_t DZ_MEASURE_MS = 800;
static const uint16_t DZ_SAMPLE_MS  = 20;
static const float DZ_MOVE_CPS_TH   = 80.0f; // 判断“动了”的阈值（counts/s）

// RatioScan：先粗扫（步进10、每点5次、每次2s），通常足够判断 k(PWM) 是否平
static const int16_t RS_PWM_MIN   = 60;
static const int16_t RS_PWM_MAX   = 200;
static const int16_t RS_PWM_STEP  = 10;
static const uint8_t RS_REPEATS   = 5;
static const uint16_t RS_WARMUP_MS  = 400;
static const uint16_t RS_MEASURE_MS = 2000;
static const uint16_t RS_SAMPLE_MS  = 20;
static const bool RS_MEASURE_REVERSE = false; // 先 false，流程跑通再改 true

// ======================================================================

Motors3pi motors(LOW, LOW);

static String readLineNonBlocking()
{
  static String buf;
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\n')
    {
      String line = buf;
      buf = "";
      line.trim();
      return line;
    }
    if (c != '\r') buf += c;
  }
  return String();
}

static float measureCpsSingleWheel(bool leftWheel, int16_t pwm,
                                   uint16_t warmupMs, uint16_t measureMs, uint16_t sampleMs)
{
  // 清残留
  (void)Encoders3pi::getCountsAndResetLeft();
  (void)Encoders3pi::getCountsAndResetRight();

  if (leftWheel) { motors.setLeft(pwm); motors.setRight(0); }
  else           { motors.setLeft(0);   motors.setRight(pwm); }

  delay(warmupMs);

  int32_t sum = 0;
  uint32_t start = millis();
  uint32_t t = start;

  while ((uint32_t)(millis() - start) < measureMs)
  {
    while ((uint32_t)(millis() - t) < sampleMs) {}
    t += sampleMs;

    int16_t d = leftWheel ? Encoders3pi::getCountsAndResetLeft()
                          : Encoders3pi::getCountsAndResetRight();
    sum += d;
  }

  motors.coast();
  delay(200);

  return (float)sum * 1000.0f / (float)measureMs; // counts/s
}

static void measureCpsBoth(int16_t pwm,
                           uint16_t warmupMs, uint16_t measureMs, uint16_t sampleMs,
                           float &cpsL, float &cpsR)
{
  (void)Encoders3pi::getCountsAndResetLeft();
  (void)Encoders3pi::getCountsAndResetRight();

  motors.set(pwm, pwm);
  delay(warmupMs);

  int32_t sumL = 0, sumR = 0;
  uint32_t start = millis();
  uint32_t t = start;

  while ((uint32_t)(millis() - start) < measureMs)
  {
    while ((uint32_t)(millis() - t) < sampleMs) {}
    t += sampleMs;

    sumL += Encoders3pi::getCountsAndResetLeft();
    sumR += Encoders3pi::getCountsAndResetRight();
  }

  motors.coast();
  delay(250);

  cpsL = (float)sumL * 1000.0f / (float)measureMs;
  cpsR = (float)sumR * 1000.0f / (float)measureMs;
}

static void runDeadzone()
{
  Serial.println("# START Deadzone");
  Serial.println("DZ,wheel,pwm,repeat,cps,is_moving");

  for (uint8_t w = 0; w < 2; w++)
  {
    bool leftWheel = (w == 0);
    const char* wname = leftWheel ? "L" : "R";

    for (int16_t pwm = DZ_PWM_START; pwm <= DZ_PWM_END; pwm += DZ_PWM_STEP)
    {
      for (uint8_t r = 1; r <= DZ_REPEATS; r++)
      {
        float cps = measureCpsSingleWheel(leftWheel, pwm, DZ_WARMUP_MS, DZ_MEASURE_MS, DZ_SAMPLE_MS);
        int moving = (fabs(cps) >= DZ_MOVE_CPS_TH) ? 1 : 0;

        Serial.print("DZ,"); Serial.print(wname); Serial.print(",");
        Serial.print(pwm); Serial.print(",");
        Serial.print(r); Serial.print(",");
        Serial.print(cps, 3); Serial.print(",");
        Serial.println(moving);
      }
    }
  }

  Serial.println("# done");
}

static void runRatioScan()
{
  Serial.println("# START RatioScan");
  Serial.println("DATA,dir,pwm,repeat,cpsL,cpsR,ratio");

  int dirs[2] = { +1, -1 };
  int dirCount = RS_MEASURE_REVERSE ? 2 : 1;

  for (int di = 0; di < dirCount; di++)
  {
    int dir = dirs[di];

    for (int16_t pwm0 = RS_PWM_MIN; pwm0 <= RS_PWM_MAX; pwm0 += RS_PWM_STEP)
    {
      int16_t pwm = (int16_t)(dir * pwm0);

      for (uint8_t r = 1; r <= RS_REPEATS; r++)
      {
        float cpsL, cpsR;
        measureCpsBoth(pwm, RS_WARMUP_MS, RS_MEASURE_MS, RS_SAMPLE_MS, cpsL, cpsR);

        float ratio = (cpsR != 0.0f) ? (cpsL / cpsR) : 0.0f;

        Serial.print("DATA,");
        Serial.print(dir); Serial.print(",");
        Serial.print(pwm); Serial.print(",");
        Serial.print(r); Serial.print(",");
        Serial.print(cpsL, 3); Serial.print(",");
        Serial.print(cpsR, 3); Serial.print(",");
        Serial.println(ratio, 6);
      }
    }
  }

  Serial.println("# done");
}

static void printHelp()
{
  Serial.println("# Commands:");
  Serial.println("#   DZ      -> run Deadzone scan (no-load, one wheel at a time)");
  Serial.println("#   RS      -> run Ratio scan (no-load, both wheels same PWM)");
  Serial.println("#   STOP    -> coast motors immediately");
  Serial.println("#   HELP    -> show this");
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  motors.begin();
  Encoders3pi::begin();
  Encoders3pi::flip(false);

  motors.coast();

  Serial.println("# READY (send DZ or RS)");
  printHelp();
}

void loop()
{
  String cmd = readLineNonBlocking();
  if (cmd.length() == 0) return;

  cmd.toUpperCase();

  if (cmd == "HELP")
  {
    printHelp();
  }
  else if (cmd == "STOP")
  {
    motors.coast();
    Serial.println("# STOPPED");
  }
  else if (cmd == "DZ")
  {
    motors.coast();
    delay(50);
    Serial.println("# ACK DZ");   // 新增：告诉MATLAB“命令已收到，马上开始吐数据”
    runDeadzone();
    Serial.println("# READY (send DZ or RS)");
  }
  else if (cmd == "RS")
  {
    motors.coast();
    delay(50);
    Serial.println("# ACK RS");   // 新增
    runRatioScan();
    Serial.println("# READY (send DZ or RS)");
  }
  else
  {
    Serial.print("# Unknown cmd: ");
    Serial.println(cmd);
    printHelp();
  }
}
