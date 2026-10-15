/* ============================================================================
 *  机械臂考核 —— 任务三 · 按键板【带检查功能版】
 *  文件：uno_keys_check.ino        板子：Arduino Uno（第二块，专职读按键）
 *
 *  ==========================================================================
 *  【这块板子干什么】
 *      正经工作（和精简版一模一样）：
 *          按键 1  →  '1'    循环夹取（A → B → C → A …）
 *          按键 2  →  '2'    录制 开始 / 结束（同一个键切换）
 *          按键 3  →  '3'    回放上一次录制的动作
 *          按键 4  →  '4'    平滑回到初始位置
 *
 *      附加功能（本文件独有）—— 打开这块板自己的串口监视器（9600），
 *      敲一个字母回车就能用：
 *          t  ->  按键接线自检：一步步提示你按每个键，最后给"通过 n / 4"
 *          w  ->  实时电平监视：按住某个键，看它到底接在哪个 D 号上
 *          s  ->  联机测试：往主控发 5 个 '?'，验证板间那条线通不通
 *          i  ->  再看一遍 4 个引脚的静态电平
 *          h  ->  命令列表
 *          q  ->  自检进行中发 q 回车可以中断
 *      实现在【第 6 部分】，那里写了"为什么需要这几个工具"。
 *
 *  ==========================================================================
 *  【★ 接线改了：以前用 D6→D10，现在用硬件串口 RX / TX】
 *  --------------------------------------------------------------------------
 *    上一版：按键板用软串口，把字符从 D6 发给主控的 D10（只发不收）。
 *    这一版：D6 ~ D10 那几根线**全部去掉**，改成两块板 RX / TX 对接 + 共地：
 *
 *        按键板 TX (D1)  ──>  主控 RX (D0)     ← 必须接，按键指令走这根线
 *        按键板 RX (D0)  <──  主控 TX (D1)     ← 可选，见下面 ② 的警告
 *        按键板 GND      ───  主控 GND         ← 必须接，不接必乱码
 *
 *    所以本程序用 Serial（Uno 唯一的硬件串口，引脚就是 D0/D1）来发字符，
 *    不再需要 SoftwareSerial。
 *
 *  ==========================================================================
 *  ⚠⚠ 三条必须知道的"副作用"（硬件串口的固有特点，不是程序 bug）⚠⚠
 *
 *    ① Uno 的 D0/D1 同时也是 USB 口，所以这条线上"三方共用"：
 *         · 你在电脑串口监视器里打的字 → 会**同时**发给主控。
 *           在按键板的监视器里敲 '1'，主控也会真的去循环夹取一次。
 *         · 主控的打印（+key: 1 之类）→ 也会流进按键板的 RX。
 *       结论：**正常演示时，两块板都插 USB 只是为了供电，
 *             只看主控那一路的串口监视器就好。**
 *
 *    ② 烧录前，把"主控 TX → 按键板 RX"这根线**拔掉**（或给主控断电）。
 *       因为主控的 TX 是个强输出，一直把按键板的 RX 顶在高电平，
 *       会和 USB 串口芯片抢同一条线 —— 结果就是**烧录失败 / 校验出错**。
 *       ★ 只留"按键板 TX → 主控 RX + 共地"这两根，功能完全正常，
 *         推荐就只用这 2 根。
 *
 *    ③ 因为主控的打印会流进按键板的 RX，如果照着老程序"收到一个字符
 *       就当命令处理"，主控随便打印一句话就会误触发自检、联机测试，
 *       甚至形成死循环。所以本程序的命令解析做了一个**过滤**：
 *           ★ 只有"整行恰好一个字符"才算命令；
 *             主控的打印都是长句子，会被整行丢掉。
 *       （见【第 5 部分】runCmdLine()，那里写得更细。）
 *
 *  ==========================================================================
 *  【推荐的使用顺序（这样最不容易乱）】
 *      第一步：**先别接主控那两根线**，只插按键板 USB，
 *              烧录本程序 → 发 t 做按键自检 → 全通过再往下走。
 *              （此时 RX 上没有任何干扰，自检提示最干净。）
 *      第二步：接上 TX + GND 两根线，插主控。
 *      第三步：发 s 联机测试。★ 因为反向线也接上了，
 *              "主控那 5 行 +LINK OK"会直接出现在按键板自己的监视器里，
 *              不用来回切窗口 —— 这是硬件串口直连的一个额外好处。
 *      第四步：按真按键，看主控有没有反应。
 * ==========================================================================*/


/* ============================================================================
 *  【1】引脚配置（要改就改这里）
 * --------------------------------------------------------------------------
 *  ★ 唯独 D0 / D1 绝对不能用：那是硬件串口，已经用来跟主控说话了。
 *  ★ D13 上有板载 LED，接按键也能用，但每按一次灯会闪一下，避开就好。
 * ==========================================================================*/
const int KEY_PIN_1 = 2;     // 按键 1：循环夹取   → 发 '1'
const int KEY_PIN_2 = 3;     // 按键 2：录制开/关  → 发 '2'
const int KEY_PIN_3 = 4;     // 按键 3：播放       → 发 '3'
const int KEY_PIN_4 = 5;     // 按键 4：回中       → 发 '4'


/* ============================================================================
 *  【2】参数与开关
 * ==========================================================================*/
const long BAUD = 9600;
/*  ★ 必须和主控那边 Serial.begin(9600) 写的一致，
 *    不一致收到的就是一堆乱码 —— 这是联调最常见的故障。 */


/* ★ 按键时要不要往串口打一行回显？默认关。
 *
 *   【为什么默认关掉？】
 *     现在两块板共用同一条串口线，按键板每打一个字，主控都会当成
 *     "未知按键"再回显一行（+Unknown key: x）。打完一行提示要刷十几行垃圾，
 *     两边监视器都被刷花，反而看不清。
 *
 *   【那怎么知道按键被识别了？】
 *     主控收到 '1' 会打印   +key: 1
 *     —— 这行字会同时出现在**按键板自己的监视器**里（共用一条线嘛），
 *     所以照样有反馈，而且更干净。
 *     另外还可以用 w（实时电平监视）或 t（引导式自检）。
 *
 *   想让本板也回显的话，把下面改成 1 即可。 */
#define ECHO_KEY_TO_MAIN 0


/* ============================================================================
 *  【3】按键状态（做"边沿触发"用）
 * ==========================================================================*/
int last1 = HIGH;    // 上一次读到的电平，HIGH = 松开
int last2 = HIGH;
int last3 = HIGH;
int last4 = HIGH;


/* ---- 诊断工具的状态（实现在【第 6 部分】）----
 *  ★ 这两个变量为什么定义在这里，而不是和函数放在一起？
 *    因为 loop() 里要用到它们。C++ 的规矩是"先声明后使用"，
 *    变量写在函数后面，前面的代码就看不到它。 */
bool watchMode = false;      // w 命令：实时电平监视开关
bool testAbort = false;      // 自检过程中发 q 回车可以中断


/* ============================================================================
 *  【4】命令行的"整行过滤"缓冲
 * --------------------------------------------------------------------------
 *  【为什么不能"收到一个字符就立刻当命令"？】
 *    因为主控的打印会流进这块板子的 RX。它的打印长这样：
 *        +key: 1
 *        +LINK OK (from keypad)
 *        +Key1: cycle pick object #1
 *    这些句子里含有 t / w / s / i / h 这些字母，要是按字符处理，
 *    主控打印一句话就会误触发"自检""联机测试"，联机测试又让主控打印更多，
 *    一来一回就是死循环。
 *
 *  【解法：整行解析 + 只认长度为 1 的行】
 *    1) 收到的字节先攒进缓冲区，遇到换行（\n 或 \r）才算"一行结束"；
 *    2) 一行结束后掐掉首尾空格，**长度必须正好是 1** 才当命令；
 *    3) 主控的所有打印都是长句子（最短的 "+key: 1" 也有 7 个字符），
 *       于是被整行丢掉，一句话都不会误触发。
 *    4) 缓冲区超过 24 个字节还没换行 → 直接清空（防垃圾数据撑爆）。
 *
 *  ★ 顺带好处：有些串口工具不发送换行符，所以再加一条"静默超时"——
 *    60ms 没收到新字节就当成这行结束了，否则敲 't' 不带回车会没反应。
 * ==========================================================================*/
const int CMD_BUF_MAX = 24;
char  cmdBuf[CMD_BUF_MAX];
int   cmdLen = 0;
unsigned long lastCharMs = 0;
const unsigned long LINE_IDLE_MS = 60;


/* ============================================================================
 *  【5】函数原型
 * ==========================================================================*/
void scanOneKey(int pin, char keyChar, int &lastState, const char* label);

void handleSerialInput();
void runCmdLine();
void printDiagHelp();
void reportPin(const __FlashStringHelper* name, int pin);
void printIdleState();
void runSelfTest();
bool testOneButton(int pin, const __FlashStringHelper* name);
bool waitForLevel(int pin, int want, unsigned long timeoutMs);
bool idleWait(unsigned long ms);
void watchInit();
void watchPins();
void reportWatch(const __FlashStringHelper* name, int from, int to);
void sendLinkTest();


/* ============================================================================
 *  setup() —— 开机只跑一次
 * ==========================================================================*/
void setup(){
  /* ---- 1. 四个按键引脚：内部上拉输入 ----
   * ★ 用 INPUT_PULLUP 而不是普通 INPUT，是为了省掉 4 个外部上拉电阻。
   *   打开之后：不按键时引脚被拉到高电平（读到 1），
   *             一按键引脚被直接接到 GND（读到 0）。
   *   面包板上少 4 个电阻 4 根线，好搭很多，也不容易接错。 */
  pinMode(KEY_PIN_1, INPUT_PULLUP);
  pinMode(KEY_PIN_2, INPUT_PULLUP);
  pinMode(KEY_PIN_3, INPUT_PULLUP);
  pinMode(KEY_PIN_4, INPUT_PULLUP);

  /* ---- 2. 打开硬件串口（既通 USB，也通主控）---- */
  Serial.begin(BAUD);
  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("Task3 Uno keypad (check ver.) ready."));
  Serial.println(F("  D2 = key1 cycle    D3 = key2 record"));
  Serial.println(F("  D4 = key3 play     D5 = key4 home"));
  Serial.println(F("  TX(D1) -> main RX(D0)    GND - GND !!!"));
  Serial.println(F("================================"));

  /* ---- 3. 上电自检：读一遍四个按键的电平 ----
   * ★ 为什么要自检？如果某个按键的线接错了（比如接到了 VCC，
   *   或者按键本身就是坏的常闭），上电读到的是 LOW，
   *   程序就会"以为按键一直被按着"，表现是刚开机机械臂自己就动了。
   *   这里一条条打印并标出 OK / 异常。
   * ★ 注意：上电只能查"没按的时候对不对"，
   *   查不了"按下去通不通"——那要用命令 t 做完整自检。
   * ★ 这一段字会同时流进主控，主控会回显一堆 "+Unknown key"，
   *   把主控那边的监视器刷花 —— 看着难受但完全无害，忽略即可。
   *   想让它别刷，把这段 printIdleState() 删掉就行。 */
  printIdleState();
  Serial.println(F("+ 想看诊断命令列表：发 h 回车。"));
}


/* ============================================================================
 *  loop() —— 主循环
 * ==========================================================================*/
void loop(){
  /* ---- ① 先看串口有没有发来诊断命令（t / w / s / i / h）----
   * ★ 收到的可能是"电脑敲的命令"，也可能是"主控的打印"，
   *   靠【第 4 部分】的整行过滤把后者丢掉。 */
  handleSerialInput();

  /* ---- ② 监视模式：只读电平、报变化，**不发指令** ----
   * ★ 为什么这里要单独一条分支、不走下面的 scanOneKey()？
   *   因为扫线的目标是"看清电平"，不是"触发动作"。
   *   如果监视时还照常往主控发指令，你测一次线机械臂就在边上动一次，
   *   既吓人又可能撞到东西。所以监视模式下只读数，不发送。 */
  if (watchMode) {
    watchPins();
    return;
  }

  /* ---- ③ 正常模式：四个按键各扫一遍 ----
   * ★ 本循环里**没有任何 delay()**，一圈跑完只要几十微秒，
   *   所以按键响应永远是"一按就有"，不会漏按。 */
  scanOneKey(KEY_PIN_1, '1', last1, "key1 cycle pick");
  scanOneKey(KEY_PIN_2, '2', last2, "key2 record");
  scanOneKey(KEY_PIN_3, '3', last3, "key3 play");
  scanOneKey(KEY_PIN_4, '4', last4, "key4 home");
}


/* ============================================================================
 *  【第 5 部分】按键扫描与命令接收
 * ==========================================================================*/

/*  scanOneKey() —— 扫描一个按键，检测到"刚按下"就发对应字符
 *
 *    参数 lastState 用了引用（&），这是老写法的便利之处：
 *    函数里改的就是调用方那个变量**本身**。
 *    （对照精简版用数组下标的写法，两种都能用，看你更习惯哪种。）
 *
 *    【★ 边沿触发 vs 电平触发】
 *      本函数是**边沿触发**：条件 = "现在读到 LOW 且 上一次是 HIGH"，
 *      → 只在"从松开变成按下"的那一瞬间触发一次。
 *      如果用电平触发（只要读到 LOW 就触发），按住不放就会一直触发：
 *      按住按键 1 会让机械臂不停地循环夹取，根本没法用。
 */
void scanOneKey(int pin, char keyChar, int &lastState, const char* label){
  int cur = digitalRead(pin);

  /* 两个条件都要满足：
   *   ① 现在读到 LOW     —— 按键被按下
   *   ② 上一次读到 HIGH  —— 才说明是"刚按下"这个瞬间 */
  if (cur == LOW && lastState == HIGH) {

#if ECHO_KEY_TO_MAIN
    /* 这一段默认不编译。打开的话每按一次键会多打一行，
     * 但主控会把这一行的每个字都当"未知按键"回显一遍，屏幕会很花。 */
    Serial.print(F("+ "));
    Serial.print(label);
    Serial.print(F("  ->  send '"));
    Serial.print(keyChar);
    Serial.println(F("'"));
#endif

    Serial.write(keyChar);
  }

  lastState = cur;   // ★ 一定要把本次电平存回去，下次比较用。
                     //   漏了这一步，按键就只会触发一次，之后再也不响应。
}

/*  handleSerialInput() —— 收字节，凑成"行"，再交给 runCmdLine() */
void handleSerialInput(){
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    lastCharMs = millis();

    if (c == '\n' || c == '\r') {          // 行结束
      if (cmdLen > 0) { runCmdLine(); cmdLen = 0; }
      continue;
    }

    if (cmdLen < CMD_BUF_MAX - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdLen = 0;                          // 太长了，肯定不是命令，直接丢
    }
  }

  /* 静默超时：有些串口工具不发换行，靠这个也能把命令"收尾" */
  if (cmdLen > 0 && (millis() - lastCharMs >= LINE_IDLE_MS)) {
    runCmdLine();
    cmdLen = 0;
  }
}

/*  runCmdLine() —— 一行收完了，判断是不是命令
 *
 *  ★ 核心规则：**掐掉首尾空格后，长度必须正好是 1**。
 *    主控的所有打印都是长句子，会在这里被静默丢掉。 */
void runCmdLine(){
  cmdBuf[cmdLen] = '\0';

  int s = 0;
  while (s < cmdLen && (cmdBuf[s] == ' ' || cmdBuf[s] == '\t')) s++;
  int e = cmdLen;
  while (e > s && (cmdBuf[e - 1] == ' ' || cmdBuf[e - 1] == '\t')) e--;

  int len = e - s;
  if (len != 1) return;          // ★ 不是单字符 → 不是命令 → 丢掉

  char c = cmdBuf[s];

  switch (c) {
    case 't': case 'T': runSelfTest(); break;

    case 'w': case 'W':
      watchMode = !watchMode;                  // 再发一次就关掉
      if (watchMode) {
        Serial.println(F(""));
        Serial.println(F("+ WATCH ON —— 现在按下任意一个按键（或用手碰一下杜邦线），"));
        Serial.println(F("+ 看串口报出来的那个 D 号是谁。"));
        Serial.println(F("+ ★ 监视模式下不会往主控发指令，机械臂不会动。"));
        Serial.println(F("+ 再发一次 w 关闭监视。"));
        watchInit();
      } else {
        Serial.println(F("+ WATCH OFF —— 恢复正常按键功能。"));
        /* ★ 退出监视时，把"上一次电平"重新读一遍。
         *   否则监视期间按键状态变了，last1..last4 还停在旧值，
         *   一回到正常模式就可能立刻触发一次莫名其妙的动作。 */
        last1 = digitalRead(KEY_PIN_1);
        last2 = digitalRead(KEY_PIN_2);
        last3 = digitalRead(KEY_PIN_3);
        last4 = digitalRead(KEY_PIN_4);
      }
      break;

    case 's': case 'S': sendLinkTest();   break;
    case 'i': case 'I': printIdleState(); break;
    case 'h': case 'H': printDiagHelp();  break;

    default:
      Serial.print(F("+Unknown cmd: ")); Serial.println(c);
      Serial.println(F("+ 可用命令: t=自检  w=实时监视  s=联机测试  i=电平  h=帮助"));
      break;
  }
}


/* ============================================================================
 *  【第 6 部分】★ 接线自检与诊断工具
 * ----------------------------------------------------------------------------
 *  【为什么需要这一部分？】
 *    "按键按了没反应"这一个现象，背后至少有 5 种完全不同的原因：
 *      ① 按键的另一端忘了接到 GND              ← 最常见，占一半以上
 *      ② 面包板那一列 / 那一排内部不通，或者杜邦线内部断了
 *      ③ 这根线其实插在了别的 D 号上（引脚搞错了）
 *      ④ 板间那根 TX→RX 信号线没接好
 *      ⑤ 两块板忘了共地
 *    没有工具的时候只能"拆了重接一遍再试"，非常低效，而且不知道到底错在哪。
 *    下面几个命令正好把这 5 种原因一一区分开：
 *
 *      t  —— 按键接线自检（引导式）
 *            逐条提示"请按住 key1"，你按对了它报 PASS；6 秒内没读到按下
 *            就报 FAIL，并列出最可能的原因。全部走完给"通过 n / 4"。
 *      w  —— 实时电平监视（开 / 关）
 *            打开后按住任意一个键，串口会打印它到底接在哪个 D 号上：
 *              D2 key1 cycle  : 1 -> 0   [按下]
 *            专门用来抓"我明明接的是 D3，按它怎么没反应"这种情况。
 *            ★ 监视模式下**不会**往主控发指令，机械臂不会动，可以放心测。
 *      s  —— 联机测试
 *            往主控发 5 个测试字符 '?'，主控收到会打印 5 行
 *            "+LINK OK (from keypad)"。
 *            ★ 反向线接着的话，这 5 行会直接出现在本板监视器里；
 *              一行都没有就是板间信号线或共地的问题。
 *      i  —— 再把 4 个引脚的静态电平打一遍
 *      h  —— 命令列表
 *
 *  【★ 实现上要注意的两点】
 *    1) 所有固定的中文提示句都套了 F()，让字符串留在 Flash 里。
 *       AVR 的 RAM 只有 2KB，中文一个字占 3 字节，不套 F() 很容易吃光。
 *    2) 传"引脚名字"给函数时，参数类型写成 const __FlashStringHelper*
 *       （就是 F() 返回的类型），这样调用处可以写
 *           reportPin(F("D2 key1 cycle "), KEY_PIN_1);
 *       名字依然留在 Flash 里，一个字节 RAM 都不占。
 * ==========================================================================*/

/*  printDiagHelp() —— 命令列表 */
void printDiagHelp(){
  Serial.println(F(""));
  Serial.println(F("========== 按键板诊断命令（波特率 9600）=========="));
  Serial.println(F("  t  ->  按键接线自检（引导式，会提示你按每个键）"));
  Serial.println(F("  w  ->  实时电平监视  开 / 关（按一次开，再按一次关）"));
  Serial.println(F("  s  ->  联机测试（往主控发 '?'，看主控那边有没有回应）"));
  Serial.println(F("  i  ->  再看一遍 4 个按键的静态电平"));
  Serial.println(F("  h  ->  这个帮助"));
  Serial.println(F("  q  ->  自检进行中发 q 回车可以中断"));
  Serial.println(F("  ★ 本板打印会流进主控：主控会回显一堆 +Unknown key，属正常。"));
  Serial.println(F("================================================"));
  Serial.println(F(""));
}

/*  reportPin() —— 打印一个引脚的静态电平，并判断是否正常 */
void reportPin(const __FlashStringHelper* name, int pin){
  int v = digitalRead(pin);
  Serial.print(F("+  "));
  Serial.print(name);
  Serial.print(F(" = "));
  Serial.print(v);
  Serial.println(v == HIGH ? F("   OK") : F("   <<== 异常！"));
}

/*  printIdleState() —— 静态电平检查（上电时和发 i 时都会调用） */
void printIdleState(){
  Serial.println(F(""));
  Serial.println(F("+---- 静态电平检查（手不要碰任何按键）----"));
  reportPin(F("D2 key1 cycle "), KEY_PIN_1);
  reportPin(F("D3 key2 record"), KEY_PIN_2);
  reportPin(F("D4 key3 play  "), KEY_PIN_3);
  reportPin(F("D5 key4 home  "), KEY_PIN_4);
  Serial.println(F("+  正常应该全是 1。谁出现 0，就是那个键的线接错了"));
  Serial.println(F("+  （最常见：按键那一脚没经过按键就直接接到了 GND）。"));
  Serial.println(F("+  注意：静态检查只能证明松开时是对的，"));
  Serial.println(F("+        证明不了按下去通不通——要查这个请发 t 做完整自检。"));
  Serial.println(F("+----------------------------------------"));
}

/*  runSelfTest() —— 引导式按键接线自检
 *    这是本文件最有用的一个工具：把"接线对不对"变成一个能自己跑的测试。
 */
void runSelfTest(){
  testAbort = false;

  Serial.println(F(""));
  Serial.println(F("+==================================================+"));
  Serial.println(F("+     按键接线自检  BUTTON WIRING SELF-TEST"));
  Serial.println(F("+  跟着提示按对应的按键就行。想中断：发 q 回车。"));
  Serial.println(F("+==================================================+"));

  printIdleState();

  int pass = 0;

  if (testOneButton(KEY_PIN_1, F("D2   key1 (cycle pick)"))) pass++;
  if (testAbort) { Serial.println(F("+ 自检被中断。")); return; }
  if (testOneButton(KEY_PIN_2, F("D3   key2 (record)")))     pass++;
  if (testAbort) { Serial.println(F("+ 自检被中断。")); return; }
  if (testOneButton(KEY_PIN_3, F("D4   key3 (play)")))       pass++;
  if (testAbort) { Serial.println(F("+ 自检被中断。")); return; }
  if (testOneButton(KEY_PIN_4, F("D5   key4 (home)")))       pass++;
  if (testAbort) { Serial.println(F("+ 自检被中断。")); return; }

  Serial.println(F(""));
  Serial.println(F("+============ SELF-TEST RESULT ============"));
  Serial.print(F("+  通过  "));
  Serial.print(pass);
  Serial.println(F(" / 4 个按键"));
  if (pass == 4) {
    Serial.println(F("+  [OK] 4 个按键的接线全部正确！"));
    Serial.println(F("+  接下来检查两块板之间那 2 根线："));
    Serial.println(F("+     按键板 TX(D1)  ->  主控 RX(D0)   （信号）"));
    Serial.println(F("+     按键板 GND     <-> 主控 GND      （★ 必须共地）"));
    Serial.println(F("+  发 s 回车可以实测这根信号线通不通。"));
  } else {
    Serial.println(F("+  [!!] 有按键没通过，按上面列出的原因逐条排查。"));
    Serial.println(F("+  万能办法：发 w 回车打开实时监视，按住那个不响应的键，"));
    Serial.println(F("+  看串口报的是哪个 D 号 —— 那就是它真正接的引脚。"));
  }
  Serial.println(F("+=========================================="));
  Serial.println(F(""));
}

/*  testOneButton() —— 测一个按键，通过返回 true
 *
 *  ★ 为什么要"真的按一下"才算通过？因为只读静态电平查不出下面这种错误：
 *      按键一端接在 D3，另一端也插在面包板上、但没有接到 GND。
 *      这时松开读到 1（看着正常），按下去还是 1（因为没接地）。
 *      只有要求"按下必须读到 0"，才能证明按键两端都接对了。
 *
 *  ★ 它一共查三件事，正好对应三种典型接错方式：
 *      ① 松手时是 1    → 不是的话，说明引脚被直接短路到 GND 了
 *      ② 按下时是 0    → 不是的话，说明"按下"这个动作没传到引脚上
 *      ③ 松手后又变 1  → 不是的话，说明引脚被常闭短接着（按键坏了）
 */
bool testOneButton(int pin, const __FlashStringHelper* name){
  Serial.println(F(""));
  Serial.print(F("+---- "));
  Serial.print(name);
  Serial.println(F(" ----"));

  /* ---- ① 静态：松手时应该是高电平 ---- */
  int idle = digitalRead(pin);
  Serial.print(F("+  松手时的电平 = "));
  Serial.println(idle);
  if (idle == LOW) {
    Serial.println(F("+  [FAIL] 还没按就已经是 0 了！"));
    Serial.println(F("+    说明这个引脚一直被拉在地线上：要么这根线直接接到了 GND"));
    Serial.println(F("+    （没经过按键），要么按键是常闭型 / 已经坏了。换个按键试试。"));
    return false;
  }

  Serial.println(F("+  请把手放到这个按键上，3 秒后开始检测 ..."));
  if (!idleWait(3000)) return false;

  /* ---- ② 按下：应该变成低电平 ---- */
  Serial.print(F("+  >>> 现在按住它，保持 3 秒 ... "));
  if (!waitForLevel(pin, LOW, 6000)) {
    Serial.println(F(""));
    if (testAbort) return false;
    Serial.println(F("+  [FAIL] 6 秒内一直是 1，没读到过 0。"));
    Serial.println(F("+    按概率排的原因："));
    Serial.println(F("+      1) 这个按键的另一端没有接到 GND     <- 最常见"));
    Serial.println(F("+      2) 面包板那一列是断的，或者杜邦线内部断了"));
    Serial.println(F("+      3) 这根线其实插在了别的 D 号上"));
    Serial.println(F("+    排查：发 w 回车打开实时监视，再按住这个键，"));
    Serial.println(F("+          串口报出来的 D 号就是它真正接的引脚。"));
    return false;
  }
  Serial.println(F("读到 0 了，"));

  /* ---- ③ 松开：应该回到高电平 ---- */
  Serial.print(F("+  >>> 请松开它 ... "));
  if (!waitForLevel(pin, HIGH, 3000)) {
    Serial.println(F(""));
    if (testAbort) return false;
    Serial.println(F("+  [FAIL] 松手之后还是 0 —— 这个引脚被一直拉在地线上。"));
    Serial.println(F("+    常见原因：按键损坏变成常闭，或者这根线短路到 GND 排了。"));
    return false;
  }
  Serial.println(F("回到 1 了。"));
  Serial.println(F("+  [PASS] 1 --按下--> 0 --松开--> 1，这个键完全正常。"));
  return true;
}

/*  waitForLevel() —— 在 timeoutMs 毫秒内等引脚变成 want 电平
 *    ★ 等待期间还在读串口，所以自检走到一半也能发 q 中断。
 *      注意：这段等待里收到的字节**不做整行过滤**（来不及），
 *      直接看有没有 'q'。主控的打印里不会出现 q，所以不会被误中断。 */
bool waitForLevel(int pin, int want, unsigned long timeoutMs){
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (digitalRead(pin) == want) return true;
    if (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == 'q' || c == 'Q') {
        testAbort = true;
        return false;
      }
    }
  }
  return false;
}

/*  idleWait() —— 纯粹等一段时间，但同样认 q 中断 */
bool idleWait(unsigned long ms){
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == 'q' || c == 'Q') {
        testAbort = true;
        return false;
      }
    }
  }
  return true;
}

/* ---- 实时电平监视（w 命令）----
 *  ★ 策略是"电平一变就报一行"，而不是每一轮都报。
 *    每轮都报的话，9600 波特率会被刷屏刷爆，反而什么都看不清。 */
int wPrev1, wPrev2, wPrev3, wPrev4;

void watchInit(){
  wPrev1 = digitalRead(KEY_PIN_1);
  wPrev2 = digitalRead(KEY_PIN_2);
  wPrev3 = digitalRead(KEY_PIN_3);
  wPrev4 = digitalRead(KEY_PIN_4);
  Serial.println(F("+ 已开始监视。按任意键，或者用手碰一下杜邦线试试。"));
}

void reportWatch(const __FlashStringHelper* name, int from, int to){
  Serial.print(F("+  "));
  Serial.print(name);
  Serial.print(F(" : "));
  Serial.print(from);
  Serial.print(F(" -> "));
  Serial.print(to);
  Serial.println(to == LOW ? F("   [按下]") : F("   [松开]"));
}

void watchPins(){
  int cur;

  cur = digitalRead(KEY_PIN_1);
  if (cur != wPrev1) { reportWatch(F("D2 key1 cycle "), wPrev1, cur); wPrev1 = cur; }

  cur = digitalRead(KEY_PIN_2);
  if (cur != wPrev2) { reportWatch(F("D3 key2 record"), wPrev2, cur); wPrev2 = cur; }

  cur = digitalRead(KEY_PIN_3);
  if (cur != wPrev3) { reportWatch(F("D4 key3 play  "), wPrev3, cur); wPrev3 = cur; }

  cur = digitalRead(KEY_PIN_4);
  if (cur != wPrev4) { reportWatch(F("D5 key4 home  "), wPrev4, cur); wPrev4 = cur; }
}

/*  sendLinkTest() —— 联机测试：验证 按键板 -> 主控 那根信号线
 *
 *  ★ 为什么发 '?' 这个字符？
 *    主控的程序里 '1'~'4' 会触发真动作，
 *    只有 '?' 是专门留出来的"我只想确认链路通不通"的测试字符 ——
 *    主控收到 '?' 会打印一行 +LINK OK (from keypad)，但不会动机械臂。
 */
void sendLinkTest(){
  Serial.println(F(""));
  Serial.println(F("+========= LINK TEST  (按键板 TX(D1) -> 主控 RX(D0)) ========="));
  Serial.println(F("+  现在往主控发 5 个测试字符 '?'。"));
  Serial.println(F("+  ★ 反向线接着的话，下面应该直接出现 5 行 +LINK OK"));
  Serial.println(F("+    （主控的回应会顺着 RX 流回本板，共用一条线）。"));
  Serial.println(F("+"));
  Serial.println(F("+  怎么判断："));
  Serial.println(F("+     5 行都有    -> 信号线和共地都正常，链路没问题"));
  Serial.println(F("+     一行都没有  -> TX->RX 这根线没接好，或者两板忘了共地"));
  Serial.println(F("+     只有几行    -> 接触不良（面包板、杜邦线很常见），换一根线"));
  Serial.println(F("+  发送中 ..."));

  for (int i = 0; i < 5; i++) {
    Serial.write('?');
    Serial.print(F("+    send '?'  "));
    Serial.println(i + 1);
    delay(300);
  }

  Serial.println(F("+  发完了。上面看到几行 +LINK OK ？"));
  Serial.println(F("+====================================================="));
  Serial.println(F(""));
}
