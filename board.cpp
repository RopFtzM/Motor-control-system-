// ==========================================================================
// 纯执行器版：只负责读取编码器和输出PWM，逻辑由 Python 负责
// ==========================================================================
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define NUM_MOTORS 4

// ================== 1. 硬件定义 (保持不变) ==================
// L293D
#define M1_EN_PIN   11
#define M2_EN_PIN   3
#define M3_EN_PIN   6
#define M4_EN_PIN   5   
// 74HC595
#define M1_IN1_BIT  (1 << 6)
#define M1_IN2_BIT  (1 << 5)
#define M2_IN1_BIT  (1 << 3)
#define M2_IN2_BIT  (1 << 4)
#define M3_IN1_BIT  (1 << 1)
#define M3_IN2_BIT  (1 << 2)
#define M4_IN1_BIT  (1 << 0)
#define M4_IN2_BIT  (1 << 7)

// 编码器
#define ENC1_A_PIN  18
#define ENC1_B_PIN  19
#define ENC2_A_PIN  20
#define ENC2_B_PIN  21

const int SER_PIN   = 8;
const int SRCLK_PIN = 4;
const int RCLK_PIN  = 12;
const int OE_PIN    = 7;

volatile uint8_t latch_state = 0;
struct MotorHW { uint8_t enPin; uint8_t in1Bit; uint8_t in2Bit; };
MotorHW motors[NUM_MOTORS] = {
  { M1_EN_PIN, M1_IN1_BIT, M1_IN2_BIT },
  { M2_EN_PIN, M2_IN1_BIT, M2_IN2_BIT },
  { M3_EN_PIN, M3_IN1_BIT, M3_IN2_BIT },
  { M4_EN_PIN, M4_IN1_BIT, M4_IN2_BIT }
};

// 编码器变量
volatile long encoderValue[NUM_MOTORS] = {0, 0, 0, 0};
volatile uint8_t lastStateEnc[NUM_MOTORS] = {0, 0, 0, 0};
const int8_t quadLookup[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

// 安全看门狗：如果超过 500ms 没收到指令，自动停机
unsigned long lastCmdTime = 0;
const unsigned long SAFETY_TIMEOUT = 500; 

// ================== 2. 硬件驱动 ==================
void write595() {
  digitalWrite(RCLK_PIN, LOW);
  shiftOut(SER_PIN, SRCLK_PIN, LSBFIRST, latch_state);
  digitalWrite(RCLK_PIN, HIGH);
}

void setMotorRaw(int idx, int pwm) {
  if (idx < 0 || idx >= NUM_MOTORS) return;
  MotorHW &m = motors[idx];

  // PWM 限幅 -255 ~ 255
  pwm = constrain(pwm, -255, 255);
  int absPWM = abs(pwm);

  // 死区处理（可选，防止电机嗡嗡响但不转）
  if (absPWM < 10 && absPWM > 0) absPWM = 0; 
  if (absPWM == 0) {
    latch_state &= ~(m.in1Bit | m.in2Bit);
    write595();
    analogWrite(m.enPin, 0);
    return;
  }

  // 方向处理
  if (pwm > 0) {
    latch_state |=  m.in1Bit;
    latch_state &= ~m.in2Bit;
  } else { 
    latch_state &= ~m.in1Bit;
    latch_state |=  m.in2Bit;
  }
  write595();
  analogWrite(m.enPin, absPWM);
}

// ================== 3. 中断 (保持不变) ==================
void readEncoder1() {
  uint8_t a = digitalRead(ENC1_A_PIN); uint8_t b = digitalRead(ENC1_B_PIN);
  uint8_t state = (a << 1) | b;
  uint8_t combined = (lastStateEnc[0] << 2) | state;
  encoderValue[0] += quadLookup[combined & 0x0F]; lastStateEnc[0] = state;
}
void readEncoder2() {
  uint8_t a = digitalRead(ENC2_A_PIN); uint8_t b = digitalRead(ENC2_B_PIN);
  uint8_t state = (a << 1) | b;
  uint8_t combined = (lastStateEnc[1] << 2) | state;
  encoderValue[1] += quadLookup[combined & 0x0F]; lastStateEnc[1] = state;
}
ISR(PCINT1_vect) {
  uint8_t pinj = PINJ;
  uint8_t a3 = (pinj >> 3) & 0x01; uint8_t b3 = (pinj >> 4) & 0x01;
  uint8_t state3 = (a3 << 1) | b3;
  uint8_t combined3 = (lastStateEnc[2] << 2) | state3;
  encoderValue[2] += quadLookup[combined3 & 0x0F]; lastStateEnc[2] = state3;

  uint8_t a4 = (pinj >> 5) & 0x01; uint8_t b4 = (pinj >> 6) & 0x01;
  uint8_t state4 = (a4 << 1) | b4;
  uint8_t combined4 = (lastStateEnc[3] << 2) | state4;
  encoderValue[3] += quadLookup[combined4 & 0x0F]; lastStateEnc[3] = state4;
}

// ================== 4. Setup & Loop ==================
void setup() {
  MCUSR = 0; wdt_disable();
  Serial.begin(115200); // 必须是 115200 或更高，减少延迟
  Serial.setTimeout(5); // 缩短串口超时时间

  pinMode(SER_PIN, OUTPUT); pinMode(SRCLK_PIN, OUTPUT);
  pinMode(RCLK_PIN, OUTPUT); pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, LOW); write595();

  for (int i = 0; i < NUM_MOTORS; i++) { pinMode(motors[i].enPin, OUTPUT); analogWrite(motors[i].enPin, 0); }

  pinMode(ENC1_A_PIN, INPUT_PULLUP); pinMode(ENC1_B_PIN, INPUT_PULLUP);
  pinMode(ENC2_A_PIN, INPUT_PULLUP); pinMode(ENC2_B_PIN, INPUT_PULLUP);
  lastStateEnc[0] = (digitalRead(ENC1_A_PIN) << 1) | digitalRead(ENC1_B_PIN);
  lastStateEnc[1] = (digitalRead(ENC2_A_PIN) << 1) | digitalRead(ENC2_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B_PIN), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), readEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B_PIN), readEncoder2, CHANGE);

  DDRJ &= ~((1 << 3) | (1 << 4) | (1 << 5) | (1 << 6));
  PORTJ |= ((1 << 3) | (1 << 4) | (1 << 5) | (1 << 6));
  uint8_t pinj = PINJ;
  lastStateEnc[2] = ((pinj >> 3) & 0x01) << 1 | ((pinj >> 4) & 0x01);
  lastStateEnc[3] = ((pinj >> 5) & 0x01) << 1 | ((pinj >> 6) & 0x01);
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT12) | (1 << PCINT13) | (1 << PCINT14) | (1 << PCINT15);
  interrupts();
}

void loop() {
  // 1. 接收指令: "C:pwm1,pwm2,pwm3,pwm4"
  // 例如: "C:100,-100,0,50" -> 批量控制
  // 或者 "R" -> 复位
  if (Serial.available()) {
    char cmd = Serial.read();
    
    if (cmd == 'C') { // Control Command
       // 读取后续的 4 个整数
       int p1 = Serial.parseInt();
       int p2 = Serial.parseInt();
       int p3 = Serial.parseInt();
       int p4 = Serial.parseInt();
       
       setMotorRaw(0, p1);
       setMotorRaw(1, p2);
       setMotorRaw(2, p3);
       setMotorRaw(3, p4);
       
       lastCmdTime = millis(); // 更新心跳时间
    }
    else if (cmd == 'R') { // Reset Encoder
       noInterrupts();
       for(int i=0; i<NUM_MOTORS; i++) encoderValue[i] = 0;
       interrupts();
       for(int i=0; i<NUM_MOTORS; i++) setMotorRaw(i, 0);
    }
    
    // 清空缓冲区剩余内容（如换行符）
    while(Serial.available() && Serial.peek() < ' ') Serial.read();
  }

  // 2. 安全检查：如果超过500ms没收到指令，强制停机
  if (millis() - lastCmdTime > SAFETY_TIMEOUT) {
    for(int i=0; i<NUM_MOTORS; i++) setMotorRaw(i, 0);
  }

  // 3. 高频发送编码器位置
  // 格式: P:pos1,pos2,pos3,pos4
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 30) { // 30ms 汇报一次 (约 33Hz)
    lastReport = millis();
    long p[4];
    noInterrupts();
    for(int i=0; i<NUM_MOTORS; i++) p[i] = encoderValue[i];
    interrupts();
    
    Serial.print("P:");
    Serial.print(p[0]); Serial.print(",");
    Serial.print(p[1]); Serial.print(",");
    Serial.print(p[2]); Serial.print(",");
    Serial.println(p[3]); // 换行
  }
}
