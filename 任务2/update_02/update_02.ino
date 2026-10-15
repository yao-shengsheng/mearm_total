#include <Servo.h>

Servo base, fArm, rArm, claw;

const int baseMin = 0;    const int baseMax = 180;
const int rArmMin = 0;    const int rArmMax = 120;
const int fArmMin = 90;   const int fArmMax = 180;
const int clawMin = 60;   const int clawMax = 120;

const int CLAW_OPEN  = 60;     // 完全张开
const int CLAW_CLOSE = 120;    // 完全闭合（= 抓取）

/* 速度 数值越大 → 走得越慢。*/
const int MOVE_STEP_MS = 40;
const int CLAW_STEP_MS = 50;

/* ==================== 停顿 ==================== */
const unsigned long CLAW_SETTLE_MS = 400;   // 合爪 / 张爪后停一下，等舵机停稳
const unsigned long STEP_PAUSE     = 150;   // 两个动作之间的小停顿


const int INIT_B = 90;
const int INIT_R = 90;
const int INIT_F = 90;
const int INIT_C = CLAW_OPEN;      // 60 = 张开

/* =============================================================================
 *  ★★★ 调参区：A / B / C 的三条序列 ★★★
 *  写法：{ '代号', 角度 }，按顺序从上往下执行。
 * ===========================================================================*/
struct Step { char s; int a; };

/* ================= 物体 A ================= */
const Step SEQ_A[] = {
  { 'b',  60 },   // 底盘转到 A 方位
  { 'r',  40 },   // 后臂降到抓取高度
  { 'f', 150 },   // 前臂伸出到位
  { 'c', 120 },   // 夹爪闭合 → 抓住
  { 'f',  90 },   // 前臂收回（把物体提离桌面）
  { 'b', 120 },   // 底盘转到放置方位
  { 'r',  45 },   // 后臂降到放置高度
  { 'f', 135 },   // 前臂伸出到位
  { 'r',  20 },   // 如果能装上 就加在这里 r往下走
  { 'c',  60 },   // 夹爪张开 → 放下
  { 'r',  90 },   // 回初始姿态
  { 'f',  90 },
  { 'b',  90 }
};

/* ================= 物体 B ================= */
const Step SEQ_B[] = {
  { 'f', 150 },   // 前臂伸出
  { 'c', 120 },   // 夹爪闭合
  { 'f', 130 },   // 前臂收回
  { 'b',  60 },   // 底盘转走
  { 'r',  80 },   // 后臂降到放置高度
  { 'c',  60 },   // 夹爪张开
  { 'f',  90 },   // 前臂收回
  { 'r',  90 },   // 回初始姿态
  { 'b',  90 }
};

/* ================= 物体 C ================= */
const Step SEQ_C[] = {
  { 'b', 150 },   // 底盘转到 C 方位
  { 'r',  30 },   // 后臂降到抓取高度
  { 'f', 150 },   // 前臂伸出到位
  { 'c', 120 },   // 夹爪闭合 → 抓住
  { 'f',  90 },   // 前臂收回
  { 'b',  80 },   // 底盘转到放置方位
  { 'f', 150 },   // 前臂伸出到位
  { 'c',  60 },   // 夹爪张开 → 放下
  { 'f',  90 },   // 回初始姿态
  { 'r',  90 },
  { 'b',  90 }
};

/* ==================== 三张表的索引 ==================== */
const Step* const SEQ[3] = { SEQ_A, SEQ_B, SEQ_C };
const byte SEQ_LEN[3] = {
  sizeof(SEQ_A) / sizeof(Step),
  sizeof(SEQ_B) / sizeof(Step),
  sizeof(SEQ_C) / sizeof(Step)
};

void doTask(int idx);                          // 打印并执行第 idx 条数组
void servoCmd(char servoName, int toPos, int servoDelay);   // 只动一个舵机
int  servoIndex(char name);                    // 舵机代号 → 下标（'b'→0 … 'c'→3）
void servoWriteAt(int idx, int angle);         // 按下标写角度
int  servoReadAt(int idx);                     // 按下标读角度
bool angleInRange(char name, int v);           // 角度是否在行程内


void setup(){
  base.attach(9);   delay(200);       // 底盘 → 9
  rArm.attach(8);   delay(200);       // 后臂 → 8
  fArm.attach(7);   delay(200);       // 前臂 → 7
  claw.attach(6);   delay(200);       // 夹爪 → 6

  /* 摆初始姿态 = INIT { b90, r90, f90, c60 } */
  base.write(INIT_B);  delay(10);
  rArm.write(INIT_R);  delay(10);
  fArm.write(INIT_F);  delay(10);
  claw.write(INIT_C);  delay(10);

  Serial.begin(9600);
  Serial.println(F("[Task 2] send A / B / C to run"));
}

void loop(){
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if      (c == 'A') doTask(0);
    else if (c == 'B') doTask(1);
    else if (c == 'C') doTask(2);
  }
}

/* ============================================================================
 *  doTask() —— 把第 idx 条数组依次打印出来
 * ==========================================================================*/
void doTask(int idx){

  if (idx < 0 || idx > 2) return;

  const char  name = (idx == 0) ? 'A' : (idx == 1) ? 'B' : 'C';
  const Step* seq  = SEQ[idx];
  const byte  n    = SEQ_LEN[idx];

  if (n == 0) return;

  /* ---- 数组开始 ---- */
  Serial.print(name);
  Serial.println(F(" start"));

  for (byte i = 0; i < n; i++) {

    char s = seq[i].s;
    int  a = seq[i].a;

    /* ---- 把这一条数组元素直接打印出来 ---- */
    Serial.print(s);
    Serial.print(' ');
    Serial.println(a);

    /* ---- 只驱动这一个舵机 ---- */
    int k = servoIndex(s);
    if (k < 0) continue;                       // 不认识的代号
    if (!angleInRange(s, a)) continue;         // 行程越界，跳过

    if (k == 3) {
      servoCmd('c', a, CLAW_STEP_MS);          // 夹爪慢一点
      delay(CLAW_SETTLE_MS);                   // 等它真正夹稳 / 放稳
    } else {
      servoCmd(s, a, MOVE_STEP_MS);            // 单个舵机平滑走，其余三个不动
      delay(STEP_PAUSE);
    }
  }

  /* ---- 数组结束 ---- */
  Serial.print(name);
  Serial.println(F(" done"));
}

int servoIndex(char name){
  if (name == 'b') return 0;                   // base 底盘
  if (name == 'r') return 1;                   // rArm 后臂
  if (name == 'f') return 2;                   // fArm 前臂
  if (name == 'c') return 3;                   // claw 夹爪
  return -1;                                   // 不认识的代号
}

void servoWriteAt(int idx, int angle){
  if      (idx == 0) base.write(angle);
  else if (idx == 1) rArm.write(angle);
  else if (idx == 2) fArm.write(angle);
  else if (idx == 3) claw.write(angle);
}

int servoReadAt(int idx){
  if (idx == 0) return base.read();
  if (idx == 1) return rArm.read();
  if (idx == 2) return fArm.read();
  if (idx == 3) return claw.read();
  return -1;
}

/*  angleInRange() —— 角度是否在该舵机行程内（防止把舵机掰坏） */
bool angleInRange(char name, int v){
  switch (name) {
    case 'b': return (v >= baseMin && v <= baseMax);
    case 'r': return (v >= rArmMin && v <= rArmMax);
    case 'f': return (v >= fArmMin && v <= fArmMax);
    case 'c': return (v >= clawMin && v <= clawMax);
    default:  return false;
  }
}

/* 一度一度地转 */
void servoCmd(char servoName, int toPos, int servoDelay){
  int idx = servoIndex(servoName);             // 先把代号翻译成下标

  if (idx < 0) return;                         // 判无效必须在最前
  if (!angleInRange(servoName, toPos)) return; // 行程检查
  int fromPos = servoReadAt(idx);

  if (fromPos <= toPos) {
    for (int i = fromPos; i <= toPos; i++) { servoWriteAt(idx, i); delay(servoDelay); }
  } else {
    for (int i = fromPos; i >= toPos; i--) { servoWriteAt(idx, i); delay(servoDelay); }
  }
}
