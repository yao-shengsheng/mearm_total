#include <SoftwareSerial.h>

/* 软串口：只发送不接收 */
SoftwareSerial mainSerial(8, 7);

/* ---- 按键初始电平 ---- */
int last1 = HIGH;
int last2 = HIGH;
int last3 = HIGH;
int last4 = HIGH;

/* 扫描一个按键：检测到"刚按下"就发对应字符 按住不放只发一次 避免了长按不断触发 */
void scanKey(int pin, char keyChar, int &lastState){
  int cur = digitalRead(pin);
  if (cur == LOW && lastState == HIGH) {
    mainSerial.write(keyChar);
  }
  lastState = cur;
}

void setup(){
  mainSerial.begin(9600);
    
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
}

void loop(){
  scanKey(2, '1', last1);
  scanKey(3, '2', last2);
  scanKey(4, '3', last3);
  scanKey(5, '4', last4);
}
