#include <Servo.h>

Servo servoArr[4];                                 // 四个舵机对象

const char servoName[4] = {'b', 'r', 'f', 'c'};    // 每个下标对应的代号
const int  servoPin[4]  = {9, 8, 7, 6};            // 每个下标对应的引脚

const int  servoMinA[4] = {0,   0,   90,  60};     // 下限 base / rArm / fArm / claw
const int  servoMaxA[4] = {180, 120, 180, 120};    // 上限 base / rArm / fArm / claw

const int clawOpenPos  = 60;                       // 张开 = 60 度
const int clawClosePos = 120;                      // 闭合 = 120 度

int DSD = 30;                                      // 当前速度值（ms/度），H / L 调整
const int DSD_MIN  = 5;                            // 最快上限
const int DSD_MAX  = 100;                          // 最慢上限
const int DSD_STEP = 5;                            // 每发一次 H / L 调整多少

const char AXIS_X = 'b';                           // 指令里的 x → 舵机 'b'
const char AXIS_Y = 'r';                           // 指令里的 y → 舵机 'r'
const char AXIS_Z = 'f';                           // 指令里的 z → 舵机 'f'

const char axisOrder[3] = {'x', 'y', 'z'};

const int pinJoy_LX = A0;                          // 左摇杆 水平（左右推）
const int pinJoy_LY = A1;                          // 左摇杆 垂直（上下推）
const int pinJoy_RX = A2;                          // 右摇杆 水平（左右推）
const int pinJoy_RY = A3;                          // 右摇杆 垂直（上下推）

const int DIR_LX = -1;
const int DIR_LY = -1;
const int DIR_RX = -1;
const int DIR_RY = -1;

const int DEADZONE = 60;                           // 死区：摇杆松手回不到精确的 512
const int MAX_STEP = 1;                            // 每个刷新周期最多转几度
const int UPDATE_MS = 20;                          // 摇杆刷新周期（20ms = 每秒 50 次）
const int AXIS_SAMPLES = 4;                        // 每路模拟量采 4 次取平均，抗抖动
// 优化

const bool MODE_CMD = true;                        // 指令模式：摇杆不动，只认串口
const bool MODE_JOY = false;                       // 摇杆模式：摇杆实时控制
bool mode = MODE_JOY;                              // 开机默认摇杆模式

const unsigned long PWM_PERIOD_US = 20000UL;       // 一个脉冲周期 20ms
const int PULSE_MIN_US = 544;                      // 0 度对应的脉宽
const int PULSE_MAX_US = 2400;                     // 180 度对应的脉宽

const unsigned long SERIAL_IDLE_MS = 50;           // 静默超时：这么久没新字符就当一条指令说完

String cmdBuf = "";                                // 正在攒的那一条指令
unsigned long lastCharMs = 0;                      // 上次收到字符的时刻

int cLX, cLY, cRX, cRY;                            // 四路摇杆的中位读数（开机静默标定）
unsigned long lastJoyMs = 0;                       // 上次刷新摇杆的时刻

int servoIndex(char name){
  if (name == 'b') return 0;                       // 底盘
  if (name == 'r') return 1;                       // 后臂
  if (name == 'f') return 2;                       // 前臂
  if (name == 'c') return 3;                       // 夹爪
  return -1;                                       // 不认识的代号
}

void servoWrite(char name, int ang){
  int i = servoIndex(name);
  if (i < 0) return;
  servoArr[i].write(ang);
}

int servoRead(char name){
  int i = servoIndex(name);
  if (i < 0) return 0;
  return servoArr[i].read();
}

bool inRange(char name, int v){
  int i = servoIndex(name);
  if (i < 0) return false;
  return (v >= servoMinA[i] && v <= servoMaxA[i]);
}

void servoMove(char name, int toPos, int dsd){
  int i = servoIndex(name);

  if (i < 0) {
    Serial.println(F("+Warning: unknown servo name!"));
    return;
  }
  if (!inRange(name, toPos)) {                     // 行程检查：越界就拒绝
    Serial.print(F("+Warning: angle out of limit, allowed: "));
    Serial.print(servoMinA[i]);
    Serial.print(F(" ~ "));
    Serial.println(servoMaxA[i]);
    return;
  }

  int from = servoArr[i].read();                   // 起点 = 程序记着的当前角度

  if (from <= toPos) {                             // 往大的方向转
    for (int a = from; a <= toPos; a++) {
      servoArr[i].write(a);
      delay(dsd);
    }
  } else {                                         // 往小的方向转
    for (int a = from; a >= toPos; a--) {
      servoArr[i].write(a);
      delay(dsd);
    }
  }
}

void servoSync3(char n1, int p1, char n2, int p2, char n3, int p3, int dsd){
  int i1 = servoIndex(n1);
  int i2 = servoIndex(n2);
  int i3 = servoIndex(n3);

  if (i1 < 0 || i2 < 0 || i3 < 0) {
    Serial.println(F("+Warning: unknown servo name in sync command!"));
    return;
  }
  if (!inRange(n1, p1) || !inRange(n2, p2) || !inRange(n3, p3)) {
    Serial.println(F("+Warning: angle out of limit, sync command refused."));
    return;
  }
  int cur1 = servoArr[i1].read();
  int cur2 = servoArr[i2].read();
  int cur3 = servoArr[i3].read();

  while (cur1 != p1 || cur2 != p2 || cur3 != p3) {
    if      (cur1 < p1) cur1++;
    else if (cur1 > p1) cur1--;

    if      (cur2 < p2) cur2++;
    else if (cur2 > p2) cur2--;

    if      (cur3 < p3) cur3++;
    else if (cur3 > p3) cur3--;

    servoArr[i1].write(cur1);                      // 三路一起写，这才叫同步
    servoArr[i2].write(cur2);
    servoArr[i3].write(cur3);

    delay(dsd);                                    // 等这一小步走完
  }
}

/* 回初始姿态 */
void armHome(){
  servoMove('b', 90, DSD);                         // 底盘回正前方
  servoMove('r', 90, DSD);                         // 后臂 90 度
  servoMove('f', 90, DSD);                         // 前臂 90 度
  servoMove('c', clawOpenPos, DSD);                // 夹爪张开
}

void clawOpen(){
  servoMove('c', clawOpenPos, DSD);
}

void clawClose(){
  servoMove('c', clawClosePos, DSD);
}

/* 提速 / 降速 */
void speedUp(){
  DSD -= DSD_STEP;
  if (DSD < DSD_MIN) DSD = DSD_MIN;
}

void speedDown(){
  DSD += DSD_STEP;
  if (DSD > DSD_MAX) DSD = DSD_MAX;
}

/*
 *          脉宽  544us  →  舵机转到   0 度
 *          脉宽 2400us  →  舵机转到 180 度
 *          角度 → 脉宽：  pulse = map(angle, 0, 180, 544, 2400)
 *          脉宽 → 占空比：duty  = pulse ÷ 20000 × 100%
 */
int angleToPulseUs(int angle){
  return map(angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
}

void printDuty(int pulseUs){
  float duty = pulseUs * 100.0 / PWM_PERIOD_US;
  Serial.print(duty, 2);
  Serial.print('%');
}

/* 打印一路舵机：代号 + 角度 + 脉宽 + 占空比 */
void printPwmLine(char name){
  int i = servoIndex(name);
  if (i < 0) return;

  int ang   = servoArr[i].read();
  int pulse = angleToPulseUs(ang);

  Serial.print(name);
  Serial.print(F(": "));
  Serial.print(ang);
  Serial.print(F("deg, "));
  Serial.print(pulse);
  Serial.print(F("us, duty "));
  printDuty(pulse);
}

/* o / status —— 四路舵机的角度 / 脉宽 / 占空比，打印一次 */
void printStatus(){
  Serial.println(F("+ Status (angle / pulse / duty)"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  "));
    printPwmLine(servoName[i]);
    Serial.println(F(""));
  }
}

int readAxisAvg(int pin){
  long sum = 0;
  for (int i = 0; i < AXIS_SAMPLES; i++) sum += analogRead(pin);
  return (int)(sum / AXIS_SAMPLES);
}

/* 开机标定摇杆中位：只记数值，不打印 */
void calibrateJoystick(){
  cLX = readAxisAvg(pinJoy_LX);
  cLY = readAxisAvg(pinJoy_LY);
  cRX = readAxisAvg(pinJoy_RX);
  cRY = readAxisAvg(pinJoy_RY);
}

int axisToStep(int pin, int center, int dir){
  int d = readAxisAvg(pin) - center;
  if (abs(d) <= DEADZONE) return 0;

  int step = MAX_STEP;
  return (d > 0 ? step : -step) * dir;
}

void nudgeServo(int i, int step){
  if (step == 0) return;

  int cur = servoArr[i].read();
  int to  = cur + step;
  if (to < servoMinA[i]) to = servoMinA[i];
  if (to > servoMaxA[i]) to = servoMaxA[i];
  if (to != cur) servoArr[i].write(to);
}

void updateJoystick(){
  if (millis() - lastJoyMs < UPDATE_MS) return;
  lastJoyMs = millis();

  nudgeServo(0, axisToStep(pinJoy_LX, cLX, DIR_LX));   // 左摇杆左右 → 底盘
  nudgeServo(1, axisToStep(pinJoy_LY, cLY, DIR_LY));   // 左摇杆上下 → 后臂
  nudgeServo(2, axisToStep(pinJoy_RY, cRY, DIR_RY));   // 右摇杆上下 → 前臂
  nudgeServo(3, axisToStep(pinJoy_RX, cRX, DIR_RX));   // 右摇杆左右 → 夹爪
}

char axisToServo(char axis){
  if (axis == 'x') return AXIS_X;
  if (axis == 'y') return AXIS_Y;
  if (axis == 'z') return AXIS_Z;
  return 0;
}

/* ----------------------------------------------------------------------------
 *  任务一.3：三舵机协同   "x10,y30,z20"
 *      "x10,y30,z20" / "X10,Y30,Z20" / "x10, y30, z20" / "10,30,20" / "z20,x10"
 *  执行完只报三路的占空比（题目要求的输出），中间的解析回显已删。
 * ------------------------------------------------------------------------- */
void multiServoCmd(String line){
  /* ---- 第 ① 步：按逗号切段 ---- */
  String part[3];
  int partNum = 0;
  int start = 0;

  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line.charAt(i) == ',') {
      if (partNum >= 3) {
        Serial.println(F("+Warning: too many fields (max 3), refused."));
        return;
      }
      part[partNum] = line.substring(start, i);
      partNum++;
      start = i + 1;
    }
  }

  /* ---- 第 ② 步：每一段取出"舵机"和"角度" ---- */
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
    target[k] = servoRead(servoN[k]);
  }

  /* ---- 三路一起走 ---- */
  servoSync3(servoN[0], target[0], servoN[1], target[1], servoN[2], target[2], DSD);

  /* ---- 任务一.3 明确要求："分别输出对应角度的 PWM 占空比" ----
   *  动作做完再报，因为占空比是按"当前角度"算的，中途报的是中间态。 */
  for (int k = 0; k < 3; k++) {
    Serial.print(F("  "));
    printPwmLine(servoN[k]);
    Serial.println(F(""));
  }
}

/* help —— 只打这六行 */
void printHelp(){
  Serial.println(F("O / open  -> claw open"));
  Serial.println(F("S / close -> claw close"));
  Serial.println(F("H / high  -> speed up"));
  Serial.println(F("L / low   -> speed down"));
  Serial.println(F("o / status -> angle + pulse + duty of 4 servos"));
  Serial.println(F("i / home   -> go to initial"));
}

/* ----------------------------------------------------------------------------
 *  runCmd() —— 处理"单字母 + 数字"类短指令
 *      b/r/f/c + 数字 → 该舵机转到这个绝对角度，例如 b90
 *      m → 静默切换 指令 / 摇杆 模式（功能保留，不再打印）
 *      o → 四路舵机 角度/脉宽/占空比
 *      i → 回初始姿态
 * ------------------------------------------------------------------------- */
void runCmd(char cmd, String arg){

  /* ---- 舵机绝对角度，字母 + 数字，例如 b90 ---- */
  if (cmd == 'b' || cmd == 'r' || cmd == 'f' || cmd == 'c') {
    if (arg.length() == 0) {
      Serial.println(F("+Warning: need a number, e.g. b90"));
      return;
    }
    servoMove(cmd, arg.toInt(), DSD);
    return;
  }

  if (cmd == 'm') {                                // 切换工作模式（静默）
    mode = (mode == MODE_CMD) ? MODE_JOY : MODE_CMD;
    return;
  }

  if (cmd == 'o') {                                // 状态：角度 / 脉宽 / 占空比
    printStatus();
    return;
  }

  if (cmd == 'i') {                                // 回初始姿态
    armHome();
    return;
  }

  Serial.println(F("Unknown command. (send 'help')"));
}

/* ----------------------------------------------------------------------------
 *  handleSerialLine() —— 一条完整指令的分发中心
 *    (A) open/close/high/low（也认单字母 O/S/H/L）
 *    (B) help / status / home
 *    (C) 含逗号 → 三舵机协同
 *    (D) 其余 → 第一个字符是命令，后面是参数
 * ------------------------------------------------------------------------- */
void handleSerialLine(String line){
  line.trim();
  if (line.length() == 0) return;

  String word = line;
  word.toLowerCase();

  /* ---- (A) 任务一.2 的四个固定指令 ---- */
  if (word == "open"  || line == "O") { clawOpen();  return; }
  if (word == "close" || line == "S") { clawClose(); return; }
  if (word == "high"  || line == "H") { speedUp();   return; }
  if (word == "low"   || line == "L") { speedDown(); return; }

  /* ---- (B) 长单词指令 ---- */
  if (word == "help")   { printHelp();   return; }
  if (word == "status") { printStatus(); return; }
  if (word == "home" || word == "init") { armHome(); return; }

  /* ---- (C) 任务一.3：含逗号 → 三舵机协同 ---- */
  if (line.indexOf(',') > 0) {
    multiServoCmd(line);
    return;
  }

  /* ---- (D) 其余短指令 ---- */
  char   cmd = line.charAt(0);
  String arg = line.substring(1);
  arg.trim();
  runCmd(cmd, arg);
}

void setup(){
  for (int i = 0; i < 4; i++) {
    servoArr[i].attach(servoPin[i]);
    delay(200);
  }

  servoWrite('b', 90);            delay(10);       // 底盘朝正前方
  servoWrite('r', 90);            delay(10);       // 后臂 90 度
  servoWrite('f', 90);            delay(10);       // 前臂 90 度
  servoWrite('c', clawOpenPos);   delay(10);       // 夹爪张开

  Serial.begin(9600);
  Serial.println(F("Mode: JOY-STICK  (send 'help' for commands)"));
  delay(300);
  calibrateJoystick();                             // 静默标定，不打印中位值
}

void loop(){
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    lastCharMs = millis();

    if (c == '\n' || c == '\r') {
      if (cmdBuf.length() > 0) {
        handleSerialLine(cmdBuf);
        cmdBuf = "";
      }
    } else {
      cmdBuf += c;
    }
  }

  /* 兜底：对方发指令不带换行符时，靠"静默超时"判断这一条说完了 */
  if (cmdBuf.length() > 0 && millis() - lastCharMs >= SERIAL_IDLE_MS) {
    handleSerialLine(cmdBuf);
    cmdBuf = "";
  }

  if (mode == MODE_JOY) {
    updateJoystick();
  }
}
