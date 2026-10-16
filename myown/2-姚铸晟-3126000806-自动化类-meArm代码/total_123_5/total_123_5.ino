#include <Servo.h>
#include <SoftwareSerial.h>

// 下方标注"优化"字样的变量/常量/函数均为AI辅助

// setup与loop放置在程序末尾

Servo base, fArm, rArm, claw;

/* 按键板D7 -> 主控板D10 只收不传*/
SoftwareSerial keyLink(10, 12);

const int baseMin = 0;    const int baseMax = 180;
const int rArmMin = 0;    const int rArmMax = 120;
const int fArmMin = 90;   const int fArmMax = 180;
const int clawMin = 60;   const int clawMax = 120;

const int CLAW_OPEN  = 60;    // 张开
const int CLAW_CLOSE = 120;   // 闭合

/* 加减速 */
int DSD = 40;                 
const int DSD_MIN = 15;
const int DSD_MAX = 100;
const int DSD_STEP = 5;

const char AXIS_X = 'b';
const char AXIS_Y = 'r';
const char AXIS_Z = 'f';
const char axisOrder[3] = {'x', 'y', 'z'};

/* 脉宽 544us → 0 度，2400us → 180 度；一个周期 20ms(50Hz) */
const unsigned long PWM_PERIOD_US = 20000UL;
const int PULSE_MIN_US = 544;
const int PULSE_MAX_US = 2400;

/* 夹爪夹住之后的停顿时间*/
const unsigned long CLAW_SETTLE_MS = 300;

/* 松开物体之后多停这么久，让机械臂稳一稳，免得刚放手就把物体带倒 */
const unsigned long STEP_PAUSE = 150;

/* 数值越大 走得越慢 */
const int MOVE_STEP_MS = 40;
const int CLAW_STEP_MS = 50; 

const float L1 = 80.0;      // 桌面 → 肩关节高度
const float L2 = 80.0;      // 后臂（肩 → 肘）
const float L3 = 150.0;     // 前臂（肘 → 笔尖）


float BASE_CENTER = 90.0;    // θ1 = 0（底盘朝正前方）时的 base 舵机读数
int   BASE_DIR    = 1;       // θ1 增大时 base 读数变化方向

float RARM_CENTER = 0.0;    // θ2 = 0（后臂水平）时的 rArm 舵机读数
int   RARM_DIR    = 1;       // θ2 增大（抬高）时 rArm 读数变化方向

float FARM_CENTER = 180.0;    // θ3 = 0（前后臂共线）时的 fArm 舵机读数
int   FARM_DIR    = 1;       // θ3 增大时 fArm 读数变化方向

float clawAng = CLAW_CLOSE;

const float PEN_DOWN_Z = 1.0;      // 落笔高度留1mm误差

const float PEN_UP_Z   = 13.0;     // 抬笔高度

const float DRAW_ROT_DEG = -90.0;  // 图形绕起笔点的旋转角（顺时针为负）

const float LINE_DX = 0.0;         // 直线 x 方向增量 mm
const float LINE_DY = 45.0;        // 直线 y 方向增量 mm

const float N_W = 27.0;            // N 的宽度
const float N_H = 45.0;            // N 的高度

/* 起笔点坐标 */
const float ORIGIN_X = 144.3;
const float ORIGIN_Y = 0.0;
const float ORIGIN_Z = -9.9;       // 该姿态的笔尖高度（模型值）

const float STEP_MM = 1.0;         // 每个 tick 前进多少 mm
const unsigned long TICK_MS = 40;  // 每多少毫秒走一步

const float RAMP_FRAC = 0.15;      // 起步/收尾的"缓加速"占每段的百分比 用于慢慢减速 慢慢加速

/* 优化:拐点减速 */
const float CORNER_FRAC = 0.14;
const float CORNER_MIN  = 0.50;

/* 优化:每个 tick 舵机角度最大变化量 */
const float ARM_MAX_STEP_DEG = 8.0f;

const float STEP_APPROACH_MM = 1.5;// "空走"（抬笔转移）时每步多少 mm
const unsigned long APPROACH_MS = 28;  // 空走的每步间隔


/* 
px[]/py[] 存放"要经过的点"，单位 mm。
8 个点 × 2 个坐标 × 4 字节(float) = 64 字节 
*/
const byte MAX_PTS = 8;
float px[MAX_PTS];
float py[MAX_PTS];
byte  pn = 0;                      // 当前路径有几个点

/* ---- 绘图状态机 ----
 *    IDLE    —— 待机
 *    RUNNING —— 正在画
 *    PAUSED  —— 暂停(进度保留)
 */
const byte ST_IDLE    = 0;
const byte ST_RUNNING = 1;
const byte ST_PAUSED  = 2;
byte drawState = ST_IDLE;// 初始状态

/* ---- 绘图进度（segIdx / segT = 当前画到第几段、段内进度）---- */
byte  segIdx = 0;                  // 当前在第几段（从 px[segIdx] 到 px[segIdx+1]）
float segT   = 0.0;                // 段内进度 0.0 ~ 1.0
float segStep = 0.0;               // 每个 tick 的进度增量（= STEP_MM / 段长）
float segLen = 1.0;                // 当前段长度（用于算增量）
unsigned long lastTickMs = 0;      // 上次走步的时刻

/* ---- 笔尖当前纸面坐标 ---- */
float tx = ORIGIN_X;               // 笔尖纸面坐标 x
float ty = ORIGIN_Y;               // 笔尖纸面坐标 y
float tz = PEN_UP_Z;               // 笔尖高度（默认抬着笔走）

float pathZ = PEN_DOWN_Z;

struct Step { char s; int a; };

const Step SEQ_A[] = {
  { 'b',  60 },
  { 'r',  40 },
  { 'f', 150 },
  { 'c', 120 },
  { 'f',  90 },
  { 'b', 120 },
  { 'r',  45 },
  { 'f', 135 },
  { 'r',  20 },
  { 'c',  60 },
  { 'r',  90 },
  { 'f',  90 },
  { 'b',  90 }
};

const Step SEQ_B[] = {
  { 'f', 150 },
  { 'c', 120 },
  { 'f', 130 },
  { 'b',  60 },
  { 'r',  80 },
  { 'c',  60 },
  { 'f',  90 },
  { 'r',  90 },
  { 'b',  90 }
};

const Step SEQ_C[] = {
  { 'b', 150 },
  { 'r',  30 },
  { 'f', 150 },
  { 'c', 120 },
  { 'f',  90 },
  { 'b',  80 },
  { 'f', 150 },
  { 'c',  60 },
  { 'f',  90 },
  { 'r',  90 },
  { 'b',  90 }
};

const Step* const SEQ[3] = { SEQ_A, SEQ_B, SEQ_C };
const byte SEQ_LEN[3] = {
  sizeof(SEQ_A) / sizeof(Step),
  sizeof(SEQ_B) / sizeof(Step),
  sizeof(SEQ_C) / sizeof(Step)
};

const int MAX_FRAMES = 200;             // 最多 200 帧（20 秒 @10Hz）

byte recBase[MAX_FRAMES];               // 每帧的 base 角度
byte recR[MAX_FRAMES];                  // 每帧的 rArm 角度
byte recF[MAX_FRAMES];                  // 每帧的 fArm 角度
byte recClaw[MAX_FRAMES];               // 每帧的 claw 角度
int  recCount = 0;                      // 实际录了多少帧

const unsigned long REC_PERIOD_MS = 100;      // 采样周期（10Hz，每 0.1 秒一帧）
unsigned long lastRecMs = 0;
bool recording = false;                        // 是否正在录制

const int PLAY_STEP_MS = 20;            // 每 20ms 写一次角度（= 舵机 PWM 刷新周期）

bool busy = false;                      // 只在执行自动动作序列时为 true

const unsigned long SERIAL_IDLE_MS = 50;
const unsigned long SERIAL_MAX_LEN = 32;
String cmdBuf = "";
unsigned long lastCharMs = 0;

const int pinJoy_LX = A0;
const int pinJoy_LY = A1;
const int pinJoy_RX = A2;
const int pinJoy_RY = A3;

/* 每个轴的方向系数（+1 / −1） */
const int DIR_LX = -1;
const int DIR_LY = -1;
const int DIR_RX = -1;
const int DIR_RY = -1;

const int DEADZONE = 60;
const int MAX_STEP = 1;
const int UPDATE_MS = 20;
const int AXIS_SAMPLES = 4;

/* 用标准模拟摇杆的中点 512 当默认值 */
int cLX = 512, cLY = 512, cRX = 512, cRY = 512;
unsigned long lastJoyMs = 0;

/*    串口发 'm' 在两套功能间切换
 *    SYS_CTRL: 任务一~三（摇杆遥控 / A B C 夹取放置 / 按键 1~4 录制回放）
 *    SYS_DRAW: 任务五（直线 / 字母 N / 暂停 续画 取消）
 *    分流情况:
 *      · 电脑 USB 串口 -> handleSerialLine()：'1' 在 DRAW 下是"画直线"
 *      · 按键板软串口  -> onKeyChar()     ：'1' 在 DRAW 下是"暂停"
 *    两路分开收、分开解释，所以同一个字符在两条通道里可以各有各的含义。
 *    按键板映射（任务三那块 key_03 板，TX D7 → 主控 D10）：
 *      CTRL: '1' 循环夹取  '2' 录制开关  '3' 回放  '4' 回中
 *      DRAW: '1' 暂停      '2' 继续      '3' N  '4' 取消(先抬笔再回中)
 * ==========================================================================*/
const byte SYS_CTRL = 0;
const byte SYS_DRAW = 1;
byte sysMode = SYS_CTRL;

/* 上电初始状态 */
const int STAND_ANG = 90;

/* DRAW初始状态 */
const int DRAW_STAND_B = 30;
const int DRAW_STAND_R = 40;
const int DRAW_STAND_F = 90;

/* 
 * 按 1 / 3 之后的“入场姿态”：
 * ① 底盘先转到正前方（DRAW_START_B = 90，“准备开始画”）
 * ② 大臂再落到 30°（DRAW_START_R = 30，笔尖压到最低 = “触纸”）
 */
const int DRAW_START_B = 90;
const int DRAW_START_R = 30;
const int DRAW_START_F = 90;

/* 画完收笔的四段姿态：
 *      ① 竖直抬笔    r/f **联动**抬到 PEN_LIFT_Z(60mm)
 *      ② b → 30    r/f 保持抬升终点值
 *      ③ f → 90    r 经 75 ⇒ 降到 118mm
 *      ④ r → 40    落回 home (30, 40, 90)
 */
const int DRAW_FINISH_LIFT_R = 75;

/* 抬笔高度：竖直抬到它、并且在它上面完成横扫。
 *   finishPath()（画完收笔）和 goDrawStandby()（按键 4 回中）共用这一个高度。 */
const float PEN_LIFT_Z = 60.0f;

const unsigned long PEN_LIFT_STEP_MS = 110;

/* 收笔动作各自的时长 */
const unsigned long FINISH_SWING_MS  = 2400;   // ② b → 30（r/f 保持抬升终点值，z 恒定）
const unsigned long FINISH_FOLD_MS   = 1200;   // ③ f → 90（r 经 75，在 home 方位空处下降）
const unsigned long FINISH_SETTLE_MS =  700;   // ④ r 75 → 40（35°）

/* 优化:收笔前的“松力回退”（mm / 步长 / 步时）为消除摩擦力*/
const float PEN_BACKOFF_MM    = 5.0f;   // （末端仍会被摩擦带走，多退一点）
const float BACKOFF_STEP_MM   = 0.5f;
const unsigned long BACKOFF_STEP_MS = 45;      // 每 0.5mm 停 45ms ≈ 11mm/s（很慢）

/* DRAW状态下缓慢回中 */
const unsigned long DRAW_STANDBY_MS = 3000;

/* 按键板那一路单独一个行缓冲（USB 那一路用 cmdBuf）  */
String keyBuf = "";
unsigned long lastKeyCharMs = 0;

void setup();
void loop();

/* —— 任务一~三 —— */
void pollSerial();
void onKeyChar(char c);
void onButton1();
void onButton2();
void onButton3();
void onButton4();
void startRecording();
void stopRecording();
void playRecording();
void armIniPosCtrl();
void handleSerialLine(String line);
void handleLineCtrl(String line, String word);
void runCmdCtrl(char cmd, String arg);
void servoCmd(char servoName, int toPos, int servoDelay);
int  servoIndex(char name);
void servoWriteAt(int idx, int angle);
int  servoReadAt(int idx);
bool angleInRange(char name, int v);
void clawOpen();
void clawClose();
void speedUp();
void speedDown();
void updateJoystick();

/* —— 任务五 —— */
bool  ikSolve(float x, float y, float z, float &sB, float &sR, float &sF);
bool  armSetXYZRaw(float x, float y, float z);
void  armSetServos(float sB, float sR, float sF);
float rad2deg(float r);
float deg2rad(float d);
void  moveToXYZSlow(float x, float y, float z);
void  drawTick();
void  evalPath(byte seg, float t, float &ox, float &oy);
float segLength(byte seg);
void  beginPath();
void  liftPenVertical(float zTop, unsigned long stepMs, int &rOut, int &fOut);
void  finishPath();
void  pausePath();
void  resumePath();
void  cancelPath();
void  goDrawStandby();
void  goCtrlStandby();
void  buildLinePath();
void  buildNPath();
bool  armReadXYZ(float &ox, float &oy, float &oz);
void  handleLineDraw(String line, String word);

/* —— 分发与工具 —— */
void  switchSysMode();
void  printCtrlMenu();
void  printDrawMenu();
void  printArmDuty();
void  printServoDuty(char name);
int   angleToPulseUs(int angle);
int   readAxisAvg(int pin);
int   axisToStep(int joyPin, int center, int dir);
void  nudgeServo(Servo &s, int step, int minA, int maxA);
void  moveServosSlow(int tb, int tr, int tf, int tc, unsigned long ms);

/* —— 任务二序列 —— */
void  doTask(int idx);
void  servoSync3(char n1, int p1, char n2, int p2, char n3, int p3, int dsd);
void  multiServoCmd(String line);
char  axisToServo(char axis);


void doTask(int idx){
  if (idx < 0 || idx > 2) return;
  if (SEQ_LEN[idx] == 0) return;
  if (busy || recording) return;

  busy = true;

  const char  name = (idx == 0) ? 'A' : (idx == 1) ? 'B' : 'C';
  const Step* seq  = SEQ[idx];
  const byte  n    = SEQ_LEN[idx];

  /* A start: b60 r40 f150 c120 f90 ... b90 done */
  Serial.print(name);
  Serial.print(F(" start: "));

  for (byte i = 0; i < n; i++) {
    char s = seq[i].s;
    int  a = seq[i].a;

    Serial.print(s);
    Serial.print(a);
    Serial.print(' ');

    int k = servoIndex(s);
    if (k < 0) continue;
    if (!angleInRange(s, a)) continue;

    if (k == 3) {
      servoCmd('c', a, CLAW_STEP_MS);
      delay(CLAW_SETTLE_MS);
    } else {
      servoCmd(s, a, MOVE_STEP_MS);
      delay(STEP_PAUSE);
    }
  }

  Serial.println(F("done"));

  busy = false;
}

void onKeyChar(char c){
  /* DRAW（任务五）的按键：
   *   1 -> 画直线      3 -> 画字母 N
   *   2 -> 暂停 / 继续（同一个键来回切）
   *   4 -> 取消并回中（竖直抬笔 → 收爪 → 缓慢回 (30,40,90)） 
   */
  if (sysMode == SYS_DRAW) {
    switch (c) {
      case '1': buildLinePath(); beginPath(); break;
      case '2':
        if      (drawState == ST_RUNNING) pausePath();
        else if (drawState == ST_PAUSED)  resumePath();
        break;
      case '3': buildNPath(); beginPath(); break;
      case '4': cancelPath(); break;
      case '\n': case '\r': case ' ': break;
      default: break;
    }
    return;
  }

  /* CTRL（任务一~三）的按键 
   *   1 = recycle A/B/C   2 = 录制开始/结束   3 = display   4 = home 
   */
  switch (c) {
    case '1': onButton1(); break;
    case '2': onButton2(); break;
    case '3': onButton3(); break;
    case '4': onButton4(); break;
    case '?': Serial.println(F("+LINK OK")); break;
    case '\n': case '\r': case ' ': break;
    default: break;
  }
}

void onButton1(){
  if (recording || busy) return;
  /*  按键1 = "recycle A/B/C"：按一次把 A、B、C 三个动作依次做完，
   *   串口上依次打出三行 "X start: ... done"。 */
  for (byte i = 0; i < 3; i++) doTask(i);
}

void onButton2(){
  if (busy) return;
  /* 录制开关 */
  if (recording) stopRecording();
  else           startRecording();
}

void startRecording(){
  recCount = 0;                            // 从头开始（旧数据作废）
  recording = true;
  lastRecMs = millis();
  Serial.println(F("+rec start"));
}

void stopRecording(){
  recording = false;
  Serial.print(F("+rec stop. frames="));
  Serial.println(recCount);
}

void onButton3(){
  if (recording || busy) return;
  playRecording();                          // display：回放上面录的那段
}

void onButton4(){
  if (recording || busy) return;
  Serial.println(F("+go home"));
  armIniPosCtrl();                          // 四路回中：90/90/90 + 爪张开
}

void playRecording(){
  if (recCount < 2) { Serial.println(F("+no rec")); return; }

  busy = true;                             // 播放期间禁用摇杆，防止干扰

  Serial.print(F("+display "));
  Serial.print(recCount);
  Serial.println(F(" frames"));

  /* 平滑挪到第 0 帧 */
  moveServosSlow(recBase[0], recR[0], recF[0], recClaw[0], 0);

  for (int i = 0; i < recCount; i++) {

    /* ★ 第一帧直接写过去（没有"上一帧"可以插值） */
    int curB = recBase[i], curR = recR[i], curF = recF[i], curC = recClaw[i];

    if (i == 0) {
      base.write(curB); rArm.write(curR); fArm.write(curF); claw.write(curC);
      delay(REC_PERIOD_MS);
      continue;
    }

    /* 优化: 通过插帧使过程更顺滑*/
    int preB = recBase[i - 1], preR = recR[i - 1], preF = recF[i - 1], preC = recClaw[i - 1];

    int sub = (int)(REC_PERIOD_MS / PLAY_STEP_MS);   // 每帧切几小步
    if (sub < 1) sub = 1;                            // 兜底：至少一步

    int stepMs = (int)(REC_PERIOD_MS / sub);         // 每小步实际延时，保证总时长 = 一帧

    for (int k = 1; k <= sub; k++) {
      /* 线性插值：位置 = 上一帧 + (本帧-上一帧) × k/sub */
      int vB = preB + (int)((long)(curB - preB) * k / sub);
      int vR = preR + (int)((long)(curR - preR) * k / sub);
      int vF = preF + (int)((long)(curF - preF) * k / sub);
      int vC = preC + (int)((long)(curC - preC) * k / sub);

      base.write(vB); rArm.write(vR); fArm.write(vF); claw.write(vC);

      /* ★ 每一步延时 = 本帧应持续的时间 ÷ 细分数
       *   这样"细分前后总时间不变"，节奏才和录制时一致。 */
      delay(stepMs);
    }
  }

  moveServosSlow(recBase[0], recR[0], recF[0], recClaw[0], 0);

  Serial.println(F("+done"));
  busy = false;
}

void clawOpen(){  servoCmd('c', CLAW_OPEN,  DSD); }
void clawClose(){ servoCmd('c', CLAW_CLOSE, DSD); }

void speedUp(){   DSD -= DSD_STEP; if (DSD < DSD_MIN) DSD = DSD_MIN; }
void speedDown(){ DSD += DSD_STEP; if (DSD > DSD_MAX) DSD = DSD_MAX; }

void armIniPosCtrl(){
  busy = true;
  servoCmd('b', 90, MOVE_STEP_MS);
  servoCmd('r', 90, MOVE_STEP_MS);
  servoCmd('f', 90, MOVE_STEP_MS);
  servoCmd('c', CLAW_OPEN, CLAW_STEP_MS);
  busy = false;
}

char axisToServo(char axis){
  if (axis == 'x') return AXIS_X;
  if (axis == 'y') return AXIS_Y;
  if (axis == 'z') return AXIS_Z;
  return 0;
}

void servoSync3(char n1, int p1, char n2, int p2, char n3, int p3, int dsd){
  if (!angleInRange(n1, p1) || !angleInRange(n2, p2) || !angleInRange(n3, p3)) return;

  int i1 = servoIndex(n1);
  int i2 = servoIndex(n2);
  int i3 = servoIndex(n3);

  int c1 = servoReadAt(i1);
  int c2 = servoReadAt(i2);
  int c3 = servoReadAt(i3);

  int d1 = p1 - c1;
  int d2 = p2 - c2;
  int d3 = p3 - c3;

  /* 同时到达 */
  int n = abs(d1);
  if (abs(d2) > n) n = abs(d2);
  if (abs(d3) > n) n = abs(d3);
  if (n == 0) return;

  for (int s = 1; s <= n; s++) {
    servoWriteAt(i1, c1 + (int)((long)d1 * s / n));
    servoWriteAt(i2, c2 + (int)((long)d2 * s / n));
    servoWriteAt(i3, c3 + (int)((long)d3 * s / n));
    delay(dsd);
  }
}

void multiServoCmd(String line){
  String part[3];
  int partNum = 0;
  int start = 0;

  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line.charAt(i) == ',') {
      if (partNum >= 3) {
        Serial.println(F("+Warn: max 3 fields"));
        return;
      }
      part[partNum] = line.substring(start, i);
      partNum++;
      start = i + 1;
    }
  }

  char servoN[3];
  int  target[3];

  for (int k = 0; k < partNum; k++) {
    part[k].trim();

    char axis = 0;
    char first = part[k].charAt(0);
    if (first == 'x' || first == 'X' || first == 'y' || first == 'Y' ||
        first == 'z' || first == 'Z') {
      if      (first == 'x' || first == 'X') axis = 'x';
      else if (first == 'y' || first == 'Y') axis = 'y';
      else                                   axis = 'z';
      part[k] = part[k].substring(1);
    }

    target[k] = part[k].toInt();
    servoN[k] = (axis != 0) ? axisToServo(axis) : axisToServo(axisOrder[k]);
  }

  /* 只写了一路或两路时：缺的那一路用"它当前的角度"补上，于是原地不动 */
  for (int k = partNum; k < 3; k++) {
    servoN[k] = axisToServo(axisOrder[k]);
    target[k] = servoReadAt(servoIndex(servoN[k]));
  }

  servoSync3(servoN[0], target[0], servoN[1], target[1], servoN[2], target[2], DSD);
  printArmDuty();
}

int angleToPulseUs(int angle){
  return map(angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
}

void printServoDuty(char name){
  int i = servoIndex(name);
  if (i < 0) return;
  int pulse = angleToPulseUs(servoReadAt(i));
  Serial.print(name);
  Serial.print('=');
  Serial.print(pulse * 100.0 / PWM_PERIOD_US, 2);
  Serial.print('%');
}

void printArmDuty(){
  printServoDuty('b'); Serial.print(' ');
  printServoDuty('r'); Serial.print(' ');
  printServoDuty('f'); Serial.println();
}

void moveServosSlow(int tb, int tr, int tf, int tc, unsigned long ms){
  int cb = base.read(), cr = rArm.read(), cf = fArm.read(), cc = claw.read();
  if (tb == cb && tr == cr && tf == cf && tc == cc) return;

  int span = abs(tb - cb);
  if (abs(tr - cr) > span) span = abs(tr - cr);
  if (abs(tf - cf) > span) span = abs(tf - cf);
  if (abs(tc - cc) > span) span = abs(tc - cc);

  if (ms == 0) {
    ms = (unsigned long)span * 12UL;
    if (ms < 300UL)  ms = 300UL;
    if (ms > 1500UL) ms = 1500UL;
  }

  int steps = (int)(ms / PLAY_STEP_MS);
  if (steps < 1) steps = 1;
  if (steps > span) steps = span;          // 比度数还细没有意义
  int stepMs = (int)(ms / steps);
  if (stepMs < 1) stepMs = 1;

  for (int k = 1; k <= steps; k++) {
    float u = (float)k / (float)steps;
    float e = u * u * (3.0f - 2.0f * u);   // 缓入缓出

    base.write(cb + (int)((tb - cb) * e));
    rArm.write(cr + (int)((tr - cr) * e));
    fArm.write(cf + (int)((tf - cf) * e));
    claw.write(cc + (int)((tc - cc) * e));
    delay(stepMs);
  }

  base.write(tb); rArm.write(tr); fArm.write(tf); claw.write(tc);   // 精确落位
  clawAng = tc;
}

int servoIndex(char name){
  if (name == 'b') return 0;
  if (name == 'r') return 1;
  if (name == 'f') return 2;
  if (name == 'c') return 3;
  return -1;
}

void servoWriteAt(int idx, int angle){
  if      (idx == 0) base.write(angle);
  else if (idx == 1) rArm.write(angle);
  else if (idx == 2) fArm.write(angle);
  else if (idx == 3) { claw.write(angle); clawAng = angle; }
}

int servoReadAt(int idx){
  if (idx == 0) return base.read();
  if (idx == 1) return rArm.read();
  if (idx == 2) return fArm.read();
  if (idx == 3) return claw.read();
  return -1;
}

bool angleInRange(char name, int v){
  switch (name) {
    case 'b': return (v >= baseMin && v <= baseMax);
    case 'r': return (v >= rArmMin && v <= rArmMax);
    case 'f': return (v >= fArmMin && v <= fArmMax);
    case 'c': return (v >= clawMin && v <= clawMax);
    default:  return false;
  }
}

void servoCmd(char servoName, int toPos, int servoDelay){
  int idx = servoIndex(servoName);
  if (idx < 0) return;
  if (!angleInRange(servoName, toPos)) return;
  int fromPos = servoReadAt(idx);

  if (fromPos <= toPos) {
    for (int i = fromPos; i <= toPos; i++) { servoWriteAt(idx, i); delay(servoDelay); }
  } else {
    for (int i = fromPos; i >= toPos; i--) { servoWriteAt(idx, i); delay(servoDelay); }
  }
}

int readAxisAvg(int pin){
  long sum = 0;
  for (int i = 0; i < AXIS_SAMPLES; i++) sum += analogRead(pin);
  return (int)(sum / AXIS_SAMPLES);
}


int axisToStep(int joyPin, int center, int dir){
  int d = readAxisAvg(joyPin) - center;
  if (abs(d) <= DEADZONE) return 0;

  int span = (d > 0) ? (1023 - center - DEADZONE) : (center - DEADZONE);
  if (span < 1) span = 1;

  int mag  = abs(d) - DEADZONE;
  int step = (int)((long)mag * MAX_STEP / span);
  if (step < 1) step = 1;
  if (step > MAX_STEP) step = MAX_STEP;

  return (d > 0 ? step : -step) * dir;
}

void nudgeServo(Servo &s, int step, int minA, int maxA){
  if (step == 0) return;
  int cur = s.read();
  int to  = cur + step;
  if (to < minA) to = minA;
  if (to > maxA) to = maxA;
  if (to != cur) s.write(to);
}

void updateJoystick(){
  if (millis() - lastJoyMs < UPDATE_MS) return;
  lastJoyMs = millis();

  nudgeServo(base, axisToStep(pinJoy_LX, cLX, DIR_LX), baseMin, baseMax);
  nudgeServo(rArm, axisToStep(pinJoy_LY, cLY, DIR_LY), rArmMin, rArmMax);
  nudgeServo(fArm, axisToStep(pinJoy_RY, cRY, DIR_RY), fArmMin, fArmMax);
  nudgeServo(claw, axisToStep(pinJoy_RX, cRX, DIR_RX), clawMin, clawMax);
}

float deg2rad(float d){ return d * 0.01745329252f; }   // 度 → 弧度
float rad2deg(float r){ return r * 57.295779513f;  }   // 弧度 → 度

bool ikSolve(float x, float y, float z, float &sB, float &sR, float &sF){

  /* ---------- 第 1 步：底盘角 θ1 ---------- */
  float th1 = rad2deg(atan2(y, x));

  /* ---------- 第 2 步：降维成平面问题 ---------- */
  float a  = sqrt(x * x + y * y);        // 水平半径
  float zp = z - L1;                     // 相对肩关节的高度
  float d  = sqrt(a * a + zp * zp);      // 肩关节到笔尖的距离

  /* ---------- 第 3 步：余弦定理求 θ3 ---------- */
  float cosT3 = (d * d - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);

  if (cosT3 >  1.0f) cosT3 =  1.0f;
  if (cosT3 < -1.0f) cosT3 = -1.0f;

  float th3a =  rad2deg(acos(cosT3));    // 解 1：肘关节一个方向
  float th3b = -th3a;                    // 解 2：肘关节另一个方向（镜像）

  float th2a = rad2deg(atan2(zp, a)) - rad2deg(atan2(L3 * sin(deg2rad(th3a)), L2 + L3 * cos(deg2rad(th3a))));
  float th2b = rad2deg(atan2(zp, a)) - rad2deg(atan2(L3 * sin(deg2rad(th3b)), L2 + L3 * cos(deg2rad(th3b))));

  float sRa = RARM_CENTER + RARM_DIR * th2a;
  float sRb = RARM_CENTER + RARM_DIR * th2b;
  float sFa = FARM_CENTER + FARM_DIR * th3a;
  float sFb = FARM_CENTER + FARM_DIR * th3b;

  const float TRAVEL_EPS = 1.0f;
  bool okA = (sRa >= rArmMin - TRAVEL_EPS && sRa <= rArmMax + TRAVEL_EPS) &&
             (sFa >= fArmMin - TRAVEL_EPS && sFa <= fArmMax + TRAVEL_EPS);
  bool okB = (sRb >= rArmMin - TRAVEL_EPS && sRb <= rArmMax + TRAVEL_EPS) &&
             (sFb >= fArmMin - TRAVEL_EPS && sFb <= fArmMax + TRAVEL_EPS);

  float th2, th3;
  if (okA && okB) {
    int curR = rArm.read();
    int curF = fArm.read();
    float costA = fabs(sRa - curR) + fabs(sFa - curF);
    float costB = fabs(sRb - curR) + fabs(sFb - curF);
    if (costA <= costB) { th2 = th2a; th3 = th3a; }
    else                { th2 = th2b; th3 = th3b; }
  } else if (okA) {
    th2 = th2a; th3 = th3a;
  } else if (okB) {
    th2 = th2b; th3 = th3b;
  } else {
    /* 两个解都超出舵机行程 */
    return false;
  }

  /* ---------- 换算成舵机角度 ---------- */
  sB = BASE_CENTER + BASE_DIR * th1;
  sR = RARM_CENTER + RARM_DIR * th2;
  sF = FARM_CENTER + FARM_DIR * th3;

  return true;
}

int limitDelta(int from, int to){
  int d = to - from;
  if (d >  (int)ARM_MAX_STEP_DEG) d =  (int)ARM_MAX_STEP_DEG;
  if (d < -(int)ARM_MAX_STEP_DEG) d = -(int)ARM_MAX_STEP_DEG;
  return from + d;
}

void armSetServos(float sB, float sR, float sF){
  base.write(limitDelta(base.read(), (int)(sB + 0.5f)));
  rArm.write(limitDelta(rArm.read(), (int)(sR + 0.5f)));
  fArm.write(limitDelta(fArm.read(), (int)(sF + 0.5f)));
}

bool armSetXYZRaw(float x, float y, float z){
  float sB, sR, sF;
  if (!ikSolve(x, y, z, sB, sR, sF)) return false;

  armSetServos(sB, sR, sF);
  return true;
}

void evalPath(byte seg, float t, float &ox, float &oy){
  /* 线性插值：x = x0 + (x1 − x0) × t */
  ox = px[seg]     + (px[seg + 1] - px[seg])     * t;
  oy = py[seg]     + (py[seg + 1] - py[seg])     * t;
}

float segLength(byte seg){
  float dx = px[seg + 1] - px[seg];
  float dy = py[seg + 1] - py[seg];
  return sqrt(dx * dx + dy * dy);
}

/* 优化: 转弯时速度变化*/
float speedProfile(float t){
  float f = 1.0f;

  /* ① 整条路径的起笔 / 收笔（只作用在第 0 段开头、最后一段结尾） */
  if (segIdx == 0 && t < RAMP_FRAC) {
    f = 0.45f + 0.55f * (t / RAMP_FRAC);
  } else if (segIdx == (byte)(pn - 2) && t > 1.0f - RAMP_FRAC) {
    f = 0.45f + 0.55f * ((1.0f - t) / RAMP_FRAC);
  }

  /* ② 内部拐点：进弯先减速（首段没有"上一个拐点"） */
  if (segIdx > 0 && t < CORNER_FRAC) {
    float g = CORNER_MIN + (1.0f - CORNER_MIN) * (t / CORNER_FRAC);
    if (g < f) f = g;
  }
  /* ③ 内部拐点：出弯再加速（末段之后也没有拐点） */
  if (segIdx < (byte)(pn - 2) && t > 1.0f - CORNER_FRAC) {
    float g = CORNER_MIN + (1.0f - CORNER_MIN) * ((1.0f - t) / CORNER_FRAC);
    if (g < f) f = g;
  }

  return f;
}

void drawTick(){
  if (millis() - lastTickMs < TICK_MS) return;   // 还没到时间
  lastTickMs = millis();
  if (segT >= 1.0f) {
    armSetXYZRaw(px[segIdx + 1], py[segIdx + 1], pathZ);   // 精确落到终点

    segIdx++;                                   // 进入下一段
    if (segIdx >= pn - 1) {                     // 所有段都画完了
      finishPath();
      return;
    }
    segT = 0.0f;                                // 新段从 0 开始
    segLen = segLength(segIdx);
    if (segLen < 0.001f) segLen = 0.001f;       // 防止除以 0（两点重合）
    segStep = STEP_MM / segLen;                 // 重算每步的进度增量
    return;                                     // 这一 tick 就跑完终点，够了
  }

  /* ---- 情况 B：正常走一小步 ---- */
  float x, y;
  evalPath(segIdx, segT, x, y);                 // 求当前步的纸面坐标

  /* 逆解 + 写舵机 */
  armSetXYZRaw(x, y, pathZ);

  /* 推进进度。★ 注意乘了 speedProfile：
   *   缓加速区每步走得少（t 增量小）→ 慢
   *   中间区每步走满 STEP_MM      → 快 */
  segT += segStep * speedProfile(segT);
}

void moveToXYZSlow(float x, float y, float z){
  float x0 = tx;
  float y0 = ty;
  float z0 = tz;

  float dx = x - x0;
  float dy = y - y0;
  float dz = z - z0;
  float dist = sqrt(dx * dx + dy * dy + dz * dz);
  int steps = (int)(dist / STEP_APPROACH_MM) + 1;
  if (steps > 400) steps = 400;            // 上限保护：再远也就 400 步

  bool ok = true;
  for (int i = 1; i <= steps; i++) {
    float u = (float)i / (float)steps;

    /* 优化:
     *       u=0   → k=0，速度 0        起步慢慢加速
     *       u=0.5 → k=0.5，速度最快
     *       u=1   → k=1，速度 0        收尾慢慢减速
     */
    float k = u * u * (3.0f - 2.0f * u);

    float xi = x0 + dx * k;
    float yi = y0 + dy * k;
    float zi = z0 + dz * k;

    /* 逆解 + 写舵机 */
    float sB, sR, sF;
    if (!ikSolve(xi, yi, zi, sB, sR, sF)) {   // 中途解不出来（比如越界了）
      ok = false;
      break;
    }
    armSetServos(sB, sR, sF);
    delay(APPROACH_MS);
  }

  if (ok) {
    /* 记录"当前实际位置"，供下一次空走当起点 */
    tx = x; ty = y; tz = z;
  } else {
    float fx, fy, fz;
    if (armReadXYZ(fx, fy, fz)) { tx = fx; ty = fy; tz = fz; }
  }
}

void beginPath(){
  if (pn < 2) return;
  if (drawState == ST_RUNNING) return;

  Serial.print(F("+draw "));
  Serial.print(pn);
  Serial.println(F(" pts"));

  /* 
   *      ① b → 90     “准备开始画”：只把底盘转到正前方，r / f 保持 home 值
   *      ② r → 30     “触纸”：大臂落下来，笔尖压到最低
   *      ③ 实测起笔点 → 铺路径 → 落笔开画
   */
  clawAng = claw.read();
  moveServosSlow(DRAW_START_B, DRAW_STAND_R, DRAW_STAND_F, claw.read(), 0);  // ① b → 90
  moveServosSlow(DRAW_START_B, DRAW_START_R, DRAW_STAND_F, claw.read(), 0);  // ② r → 30

  /* ---- ③ 实测起笔点，并把“相对增量”的路径平移到它上面 ---- */
  float x0 = ORIGIN_X, y0 = ORIGIN_Y, z0 = ORIGIN_Z;
  bool gotStart = armReadXYZ(x0, y0, z0);
  if (!gotStart) { x0 = ORIGIN_X; y0 = ORIGIN_Y; z0 = ORIGIN_Z; }

  /* 优化:图形自身绕起笔点旋转 DRAW_ROT_DEG */
  if (DRAW_ROT_DEG != 0.0f) {
    float cr = cos(deg2rad(DRAW_ROT_DEG));
    float sr = sin(deg2rad(DRAW_ROT_DEG));
    for (byte i = 0; i < pn; i++) {
      float lx = px[i], ly = py[i];
      px[i] = cr * lx - sr * ly;
      py[i] = sr * lx + cr * ly;
    }
  }

  for (byte i = 0; i < pn; i++) { px[i] += x0; py[i] += y0; }   // 增量 → 绝对坐标
  pathZ = z0;                                                   // 落笔高度 = 实测值

  segIdx  = 0;
  segT    = 0.0f;
  segLen  = segLength(0);
  if (segLen < 0.001f) segLen = 0.001f;
  segStep = STEP_MM / segLen;

  /* ④ 落笔：把笔尖精确压到起笔点上 */
  armSetXYZRaw(px[0], py[0], pathZ);

  /* ⑤ 开跑（非阻塞） */
  lastTickMs = millis();
  drawState  = ST_RUNNING;
}

void liftPenVertical(float zTop, unsigned long stepMs, int &rOut, int &fOut){
  float t2 = deg2rad((float)rArm.read());
  float t3 = deg2rad((float)fArm.read() - 180.0f);
  float a  = L2 * cos(t2) + L3 * cos(t2 + t3);       // 笔尖水平距离
  float z0 = L1 + L2 * sin(t2) + L3 * sin(t2 + t3);  // 笔尖高度

  rOut = (int)rArm.read();                           // 默认原样返回（已经在高处就直接走）
  fOut = (int)fArm.read();
  if (z0 >= zTop - 1.0f) return;

  const float D_MIN = sqrt((float)(L2 * L2 + L3 * L3)) + 0.1f;  // ≈170.1（θ3 = −90° 边界，留点余量）
  const float D_MAX = (float)(L2 + L3) - 1.0f;                  // ≈229（别把臂拉直）

  int cb = (int)base.read();                         // 抬笔时底盘不动
  int cc = claw.read();                              // 爪也不动

  const float STEP_MM = 9.0f;                        // 每步升高约 9mm
  int steps = (int)((zTop - z0) / STEP_MM + 0.5f);
  if (steps < 2)  steps = 2;
  if (steps > 26) steps = 26;

  for (int i = 1; i <= steps; i++) {
    float z   = z0 + (zTop - z0) * ((float)i / (float)steps);
    float dzp = z - L1;
    float lo  = (fabs(dzp) >= D_MIN) ? 0.0f : sqrt(D_MIN * D_MIN - dzp * dzp);
    float hi  = (fabs(dzp) >= D_MAX) ? 1e6f : sqrt(D_MAX * D_MAX - dzp * dzp);
    a = constrain(a, lo, hi);                        // 收进可行区间（尽量不动 a）

    float d   = sqrt(a * a + dzp * dzp);
    float cT3 = (d * d - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);
    if (cT3 >  1.0f) cT3 =  1.0f;                    // 浮点保险
    if (cT3 < -1.0f) cT3 = -1.0f;
    float t3n = -acos(cT3);                          // 本机只有“肘上”解（θ3 ≤ 0）
    float t2n = atan2(dzp, a) - atan2(L3 * sin(t3n), L2 + L3 * cos(t3n));
    rOut = (int)(RARM_CENTER + RARM_DIR * rad2deg(t2n) + 0.5f);
    fOut = (int)(FARM_CENTER + FARM_DIR * rad2deg(t3n) + 0.5f);
    moveServosSlow(cb, rOut, fOut, cc, stepMs);
  }
}

void finishPath(){
  /* 优化: 松力回退：沿**最后一段**的反方向，慢慢退回 PEN_BACKOFF_MM 毫米。 */
  {
    float dx = px[pn - 1] - px[pn - 2];
    float dy = py[pn - 1] - py[pn - 2];
    float L  = sqrt(dx * dx + dy * dy);
    if (L > 0.001f) {
      dx /= L;  dy /= L;                        // 最后一段的单位方向
      float cx = px[pn - 1], cy = py[pn - 1];
      int   n  = (int)(PEN_BACKOFF_MM / BACKOFF_STEP_MM + 0.5f);
      for (int i = 0; i < n; i++) {
        float nx = cx - dx * BACKOFF_STEP_MM;
        float ny = cy - dy * BACKOFF_STEP_MM;
        if (!armSetXYZRaw(nx, ny, pathZ)) break;    // 退不过去就停在这
        cx = nx;  cy = ny;
        delay(BACKOFF_STEP_MS);
      }
    }
  }

  int cc = claw.read();               // 爪是闭合的，全程不动

  /* 竖直抬笔 */
  int rTop, fTop;
  liftPenVertical(PEN_LIFT_Z, PEN_LIFT_STEP_MS, rTop, fTop);

  /* 底盘归 home 方位（30°）*/
  moveServosSlow(DRAW_STAND_B, rTop, fTop, cc, FINISH_SWING_MS);

  /* 前臂归 90° */
  moveServosSlow(DRAW_STAND_B, DRAW_FINISH_LIFT_R, DRAW_STAND_F, cc, FINISH_FOLD_MS);

  /* 大臂落回 home 的 40° */
  moveServosSlow(DRAW_STAND_B, DRAW_STAND_R, DRAW_STAND_F, cc, FINISH_SETTLE_MS);

  float fx, fy, fz;
  if (armReadXYZ(fx, fy, fz)) { tx = fx; ty = fy; tz = fz; }

  drawState = ST_IDLE;
  Serial.println(F("+done"));
}

void buildLinePath(){
  pn = 2;
  px[0] = 0.0f;                  py[0] = 0.0f;
  px[1] = LINE_DX;               py[1] = LINE_DY;
}

/*  
 * 四个顶点按 " 左下 → 左上 → 右下 → 右上 " 的顺序，一笔画完 3 段。
 */
void buildNPath(){
  pn = 4;
  px[0] = 0.0f;                  py[0] = 0.0f;            // P0 左下
  px[1] = 0.0f;                  py[1] = N_H;             // P1 左上（竖线）
  px[2] = N_W;                   py[2] = 0.0f;            // P2 右下（斜线）
  px[3] = N_W;                   py[3] = N_H;             // P3 右上（竖线）
}

bool armReadXYZ(float &ox, float &oy, float &oz){
  if (BASE_DIR == 0 || RARM_DIR == 0 || FARM_DIR == 0) return false;

  float th1 = (base.read() - BASE_CENTER) / (float)BASE_DIR;
  float th2 = (rArm.read() - RARM_CENTER) / (float)RARM_DIR;
  float th3 = (fArm.read() - FARM_CENTER) / (float)FARM_DIR;

  float a  = L2 * cos(deg2rad(th2)) + L3 * cos(deg2rad(th2 + th3));
  oz = L1 + L2 * sin(deg2rad(th2)) + L3 * sin(deg2rad(th2 + th3));
  ox = a * cos(deg2rad(th1));
  oy = a * sin(deg2rad(th1));
  return true;
}

void pausePath(){
  if (drawState != ST_RUNNING) return;
  drawState = ST_PAUSED;
  Serial.println(F("+pause"));
}

void resumePath(){
  if (drawState != ST_PAUSED) return;
  lastTickMs = millis();          // 优化:把计时基准重置成"现在"，节奏更干净
  drawState  = ST_RUNNING;
  Serial.println(F("+go"));
}

void cancelPath(){
  if (drawState != ST_IDLE) {
    drawState = ST_IDLE;                    // 停推进
    segIdx = 0;
    segT   = 0.0f;                          // 进度清零
  }
  Serial.println(F("+cancel: home"));
  goDrawStandby();
}

void goDrawStandby(){
  /* 先把笔抬起来 再回中 */
  int rTop = (int)rArm.read();                     // 抬升终点（默认原地，不抬时也不报错）
  int fTop = (int)fArm.read();
  liftPenVertical(PEN_LIFT_Z, PEN_LIFT_STEP_MS, rTop, fTop);

  /*  收爪：此时已经离纸，夹爪怎么动都碰不到纸面 */
  if (claw.read() != CLAW_CLOSE) servoCmd('c', CLAW_CLOSE, 30);

  /* ③ 缓慢回 home：笔已在高处，整机一起走是安全的  */
  moveServosSlow(DRAW_STAND_B, DRAW_STAND_R, DRAW_STAND_F, claw.read(), DRAW_STANDBY_MS);

  float fx, fy, fz;
  if (armReadXYZ(fx, fy, fz)) { tx = fx; ty = fy; tz = fz; }
}

void goCtrlStandby(){
  /* 还在画 / 已暂停 → 先停推进 + 原地抬笔 */
  if (drawState != ST_IDLE) {
    drawState = ST_IDLE;
    moveToXYZSlow(tx, ty, PEN_UP_Z);
    segIdx = 0;
    segT   = 0.0f;
  }

  moveServosSlow(STAND_ANG, STAND_ANG, STAND_ANG, CLAW_OPEN, 0);
  clawAng = CLAW_OPEN;

  float fx, fy, fz;
  if (armReadXYZ(fx, fy, fz)) { tx = fx; ty = fy; tz = fz; }
}

void pollSerial(){
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    lastCharMs = millis();

    if (c == '\n' || c == '\r') {
      if (cmdBuf.length() > 0) { handleSerialLine(cmdBuf); cmdBuf = ""; }
    } else {
      cmdBuf += c;
      if (cmdBuf.length() > SERIAL_MAX_LEN) cmdBuf = "";
    }
  }
  if (cmdBuf.length() > 0 && (millis() - lastCharMs >= SERIAL_IDLE_MS)) {
    handleSerialLine(cmdBuf);
    cmdBuf = "";
  }

  /* ---- 按键板（key_03）软串口 D10 <-- 按键板 D7 ---- */
  while (keyLink.available() > 0) {
    char c = (char)keyLink.read();
    lastKeyCharMs = millis();

    if (c == '\n' || c == '\r') {
      if (keyBuf.length() > 0) { onKeyChar(keyBuf.charAt(0)); keyBuf = ""; }
    } else {
      keyBuf += c;
      if (keyBuf.length() > SERIAL_MAX_LEN) keyBuf = "";
    }
  }
  /* 按键板发的是单字符、也常常不带换行，靠"静默超时"断句 */
  if (keyBuf.length() > 0 && (millis() - lastKeyCharMs >= SERIAL_IDLE_MS)) {
    onKeyChar(keyBuf.charAt(0));
    keyBuf = "";
  }
}

void printCtrlMenu(){
  Serial.println(F("+MODE CTRL:"));
  Serial.println(F("1.2  O->open  S->close  H->fast  L->slow"));
  Serial.println(F("1.3  x10,y30,z20 -> sync move + duty"));
  Serial.println(F("2.1  A / B / C -> pick & place"));
  Serial.println(F("3.1  1:recycle A/B/C  2:record  3:display  4:home"));
  Serial.println(F("     m -> to DRAW"));
}

void printDrawMenu(){
  Serial.println(F("+MODE DRAW:"));
  Serial.println(F("5.1  1 -> line        3 -> N"));
  Serial.println(F("5.2  2 -> pause/resume   4 -> cancel"));
  Serial.println(F("     m -> to CTRL"));
}

void switchSysMode(){
  if (sysMode == SYS_CTRL) {
    sysMode = SYS_DRAW;
    clawAng = claw.read();
    float fx, fy, fz;
    if (armReadXYZ(fx, fy, fz)) { tx = fx; ty = fy; tz = fz; }
    printDrawMenu();
    goDrawStandby();
  } 
  else 
  {
    sysMode = SYS_CTRL;
    busy = false;
    printCtrlMenu();
    goCtrlStandby();
  }
}

void handleSerialLine(String line){
  line.trim();
  if (line.length() == 0) return;

  String word = line;
  word.toLowerCase();

  if (word == "m") { switchSysMode(); return; }

  if (sysMode == SYS_DRAW) handleLineDraw(line, word);
  else                     handleLineCtrl(line, word);
}

/* ---- 任务一~三 模式的指令 ----
 *   1.2  O / open   张爪，回 "claw open"
 *        S / close  闭合，回 "claw close"
 *        H / high   加速，回 "dsd=xx"
 *        L / low    减速，回 "dsd=xx"
 *   1.3  xx,xx,xx   三舵机**同时到达**目标角度，然后同一行打 b/r/f 占空比
 *   2.1  A / B / C  执行夹取动作，每一步打印在同一行
 *   3.1  1 / 2 / 3 / 4  等同按键板
 *   m    切换模式
 */
void handleLineCtrl(String line, String word){
  if (line.length() == 1 &&
      ((line[0] >= '1' && line[0] <= '4') || line[0] == '?')) {
    onKeyChar(line[0]);
    return;
  }

  /* 任务二：上位机发 A / B / C */
  if (line == "A") { doTask(0); return; }
  if (line == "B") { doTask(1); return; }
  if (line == "C") { doTask(2); return; }

  /* 1.2 夹爪 / 速度 —— 每种动作只回一行极简反馈 */
  if (word == "open"  || word == "o") { servoCmd('c', CLAW_OPEN,  DSD); Serial.println(F("claw open"));  return; }
  if (word == "close" || word == "s") { servoCmd('c', CLAW_CLOSE, DSD); Serial.println(F("claw close")); return; }
  if (word == "high"  || word == "h") { speedUp();   Serial.print(F("DSD=")); Serial.println(DSD); return; }
  if (word == "low"   || word == "l") { speedDown(); Serial.print(F("DSD=")); Serial.println(DSD); return; }

  /* 1.3 含逗号 -> 三舵机同时到达 + 同行占空比 */
  if (line.indexOf(',') > 0) { multiServoCmd(line); return; }

  char   cmd = line.charAt(0);
  String arg = line.substring(1);
  arg.trim();
  runCmdCtrl(cmd, arg);
}


void runCmdCtrl(char cmd, String arg){
  if (cmd == 'b' || cmd == 'c' || cmd == 'f' || cmd == 'r') {
    if (arg.length() == 0) return;
    /* 优化:正在跑序列 / 录制时拒绝手动指令，防止重入把序列打断 */
    if (busy || recording) return;
    servoCmd(cmd, arg.toInt(), DSD);
  }
}


void handleLineDraw(String line, String word)
{
  if (line == "1") { buildLinePath(); beginPath(); return; }          // 直线
  if (line == "3") { buildNPath();    beginPath(); return; }          // 字母 N
  if (line == "2") {                                                  // 暂停 / 继续（来回切）
    if      (drawState == ST_RUNNING) pausePath();
    else if (drawState == ST_PAUSED)  resumePath();
    return;
  }
  if (line == "4") { cancelPath(); return; }                          // 取消（回 DRAW 中位）
}


void setup(){
  base.attach(9, PULSE_MIN_US, PULSE_MAX_US);   delay(200);
  rArm.attach(8, PULSE_MIN_US, PULSE_MAX_US);   delay(200);
  fArm.attach(7, PULSE_MIN_US, PULSE_MAX_US);   delay(200);
  claw.attach(6, PULSE_MIN_US, PULSE_MAX_US);   delay(200);

  /* 上电先摆成 **CTRL 的初始姿态**：(90, 90, 90) + 爪张开。 */
  base.write(STAND_ANG);   delay(10);
  rArm.write(STAND_ANG);   delay(10);
  fArm.write(STAND_ANG);   delay(10);
  claw.write(CLAW_OPEN);   delay(10);

  Serial.begin(9600);
  keyLink.begin(9600);
  cmdBuf.reserve(SERIAL_MAX_LEN);
  keyBuf.reserve(SERIAL_MAX_LEN);

  delay(300);

  printCtrlMenu();
}

void loop(){
  /* 串口两路 */
  pollSerial();

  if (sysMode == SYS_CTRL) {
    if (recording) {
      if (millis() - lastRecMs >= REC_PERIOD_MS) {
        lastRecMs = millis();
        if (recCount < MAX_FRAMES) {
          recBase[recCount] = (byte)base.read();
          recR[recCount]    = (byte)rArm.read();
          recF[recCount]    = (byte)fArm.read();
          recClaw[recCount] = (byte)claw.read();
          recCount++;
        } else {
          stopRecording();              // 缓冲存满，自动收尾
        }
      }
    }
    /* 摇杆：执行自动动作序列时让路 */
    if (!busy) updateJoystick();

  } else {
    /* 任务五：画图走步 */
    if (drawState == ST_RUNNING) drawTick();
  }
}
