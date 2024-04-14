#include <M5Core2.h>
#include <MadgwickAHRS.h>
Madgwick madgwic;
Madgwick MadgwickFilter;

#define MOTOR_PIN_F        33 // to DC Motor Driver FIN
#define MOTOR_PIN_R        32  // to DC Motor Driver RIN
#define MOTOR_PWM_F        0  // PWM CHANNEL
#define MOTOR_PWM_R        1  // PWM CHANNEL

float Kp = 700;//
float Ki =200;//4Kd
float Kd = 52;//
float target = 66.5;//67-64.5
int Motor_offset = 27;

float P, I, D, preP;
float roll, pitch, yaw;
float now;
float dt, preTime;
double power = 0;
int Duty = 0;

void drawScreen() {
    M5.Lcd.setCursor(0, 17);
    M5.Lcd.printf("\nStay:%6.1f\n",target);
    M5.Lcd.printf("Roll:%6.1f\n",roll);
    M5.Lcd.printf("Dev:%7.1f\n", target - roll);
    M5.Lcd.printf("\nDuty:%5.1d\n\n\n", Duty);
    M5.Lcd.printf("Kp:%8.1f\n", Kp);
    M5.Lcd.printf("Ki:%8.1f\n", Ki);
    M5.Lcd.printf("Kd:%8.3f\n", Kd);
}

void setup() {
  Serial.begin(151200);
  //MadgwickFilter.begin(100);

  M5.begin(true, true, true, false);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.IMU.Init();

  pinMode(MOTOR_PIN_F, OUTPUT);
  pinMode(MOTOR_PIN_R, OUTPUT);
  ledcSetup(MOTOR_PWM_F, 490, 8); //CHANNEL, FREQ, BIT
  ledcSetup(MOTOR_PWM_R, 490, 8);
  ledcAttachPin(MOTOR_PIN_F, MOTOR_PWM_F);
  ledcAttachPin(MOTOR_PIN_R, MOTOR_PWM_R);
  drawScreen();
}

void loop() {
  preTime = micros();
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  //Serial.printf("%8.1f\n",roll);
  drawScreen();
  delay(2);
  
  now = target - roll;
  dt = (micros() - preTime) / 1000000; // 処理時間を求める
  preTime = micros(); // 処理時間を記録

// 目標角度から現在の角度を引いて偏差を求める
// -90~90を取るので90で割って-1.0~1.0にする
  P  = (target - roll) / 90;
// 偏差を積分する
  I += P * dt;
// 偏差を微分する
  D  = (P - preP) / dt;
// 偏差を記録する
  preP = P;
// 積分部分が大きくなりすぎると出力が飽和するので大きくなり過ぎたら0に戻す(アンチワインドアップ)
  if (100 < abs(I * Ki)) {
    I = 0;
  }

  power = (int)(Kp * P + Ki * I + Kd * D);
  Duty = (int)(constrain((int)abs(power)+Motor_offset, 0, 255)); //255に制限　飽和する

  // +-30度を越えたら倒れたとみなす
  if (-30 < now && now < 30) {
    ledcWrite( MOTOR_PWM_F,(power < 0 ?    0 : Duty) );  
    ledcWrite( MOTOR_PWM_R,(power < 0 ? Duty :    0) ); 
  }
  else { 
    ledcWrite(MOTOR_PWM_F, 0);
    ledcWrite(MOTOR_PWM_R, 0);
    power = 0;
    I = 0;
  }
}  