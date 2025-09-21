#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <math.h>
MPU6050 mpu(Wire);
Servo myservo;      // كائن السيرفو
int servoPin = 32;



  
// ======= Pins =======
const int IN1_FL = 22, IN2_FL = 23, ENA_FL = 2;
const int IN3_FR = 24, IN4_FR = 25, ENB_FR = 3;
const int IN1_AL = 26, IN2_AL = 27, ENA_AL = 4;
const int IN3_AR = 28, IN4_AR = 29, ENB_AR = 5;
const int RELAY_BRUSH = 30;
const int RELAY_PUMP = 31;
// Ultrasonic
const int TRIG_FRONT = 46, ECHO_FRONT = 47;
const int TRIG_DOWN  = 52, ECHO_DOWN  = 53;
const int TRIG_RIGHT = 48, ECHO_RIGHT = 49;
const int TRIG_LEFT  = 50, ECHO_LEFT = 51;
bool movingForwardManual = false;
// Speeds
const int BASE_SPEED_FWD   = 200;
const int BASE_SPEED_BACK  = 100;
const int BASE_SPEED_TURN  = 200;

const int TRIM_LEFT  = 0;
const int TRIM_RIGHT = -15;

// Times
 int BACK_MS        = 1500;
 int FORWARD_STEPMS = 700;

// Ultrasonic thresholds
const int FRONT_STOP_CM  = 15;
const int EDGE_CM        = 15;

// ======= Servo control =======
bool movingForward = false;
unsigned long forwardStartTime = 0;
bool servoAtZero = true;
//PUMP
unsigned long lastPumpToggle = 0;
bool pumpState = false;

bool lastRun = false;


String command = "";
bool autoMode = false;
bool turnRightNext = true;
float lastFront = 100.0, lastDown = 10.0;
bool firstSideCheck = true;
bool servoIsUp = false;
const int BATTERY_PIN = A0;     // وصل مقسم الجهد هنا
const float AREF_VOLT = 5.0;    // مرجع الأنالوج (DEFAULT = 5V)

const float R1 = 30000.0;       // مقاومة أعلى (للبطارية) بالأوم
const float R2 = 7500.0;        // مقاومة أسفل (للأرضي) بالأوم
// عدّل R1/R2 حسب مقسم الجهد الحقيقي عندك

const float VBAT_FULL  = 12.4;  // فول فولت بطارية
const float VBAT_EMPTY = 9;  // حد أدنى (عدّل حسب بطاريتك)

unsigned long lastSend = 0;

// ===== متغيرات الميلان =====
float lastTiltAngle = 0;       // آخر زاوية مسجلة
float currentTiltAngle = 0;    // الزاوية الحالية
const float ANGLE_THRESHOLD = 3.0; // فرق لازم يتجاوز (3 درجات مثلاً) حتى نخزن

void updateStepTimes() {
  if (fabs(currentTiltAngle) <= 8.0) {
    BACK_MS        = 1700;
    FORWARD_STEPMS = 600;
  } else {
    BACK_MS        = 2300;
    FORWARD_STEPMS = 600;
  }
}


// دالة لتحديث قراءة الميلان
void updateTiltAngle() {
  mpu.update();   // تحديث الحساس

  float newX = mpu.getAngleX();   // ميلان حول محور X
  float newY = mpu.getAngleY();   // ميلان حول محور Y

  // نختار القيمة الأكبر بالمطلق
  float newAngle;
  if (fabs(newX) >= fabs(newY)) {
    newAngle = newX;
  } else {
    newAngle = newY;
  }

  // تحقق إذا التغير كبير
  if (fabs(newAngle - lastTiltAngle) >= ANGLE_THRESHOLD) {
    lastTiltAngle = newAngle;
    currentTiltAngle = newAngle;

    
  }
}

float readBatteryVoltage() {
  const int N = 20;
  long sum = 0;
  for (int i = 0; i < N; i++) { sum += analogRead(BATTERY_PIN); delay(2); }
  float raw   = sum / (float)N;
  float v_pin = (raw / 1023.0) * AREF_VOLT;          // جهد على البن بعد المقسم
  float v_bat = v_pin * (R1 + R2) / R2;              // رجاع جهد البطارية قبل المقسم
  return v_bat;
}

float readBatteryPercent() {
  float v = readBatteryVoltage();
  float p = (v - VBAT_EMPTY) / (VBAT_FULL - VBAT_EMPTY) * 100.0;
  if (p < 0) p = 0; if (p > 100) p = 100;
  
  return p;
}


// ======= Utility =======
int clampPWM(int v){ if (v<0) return 0; if (v>255) return 255; return v; }

// ======= Motor functions =======
void setLeftForward(int spd){
  digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW);  analogWrite(ENA_FL, clampPWM(spd + TRIM_LEFT));
  digitalWrite(IN1_AL, HIGH); digitalWrite(IN2_AL, LOW);  analogWrite(ENA_AL, clampPWM(spd + TRIM_LEFT));
}
void setRightForward(int spd){
  digitalWrite(IN3_FR, HIGH); digitalWrite(IN4_FR, LOW);  analogWrite(ENB_FR, clampPWM(spd + TRIM_RIGHT));
  digitalWrite(IN3_AR, HIGH); digitalWrite(IN4_AR, LOW);  analogWrite(ENB_AR, clampPWM(spd + TRIM_RIGHT));
}
void setLeftBackward(int spd){
  digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, HIGH);  analogWrite(ENA_FL, clampPWM(spd + TRIM_LEFT));
  digitalWrite(IN1_AL, LOW); digitalWrite(IN2_AL, HIGH);  analogWrite(ENA_AL, clampPWM(spd + TRIM_LEFT));
}
void setRightBackward(int spd){
  digitalWrite(IN3_FR, LOW); digitalWrite(IN4_FR, HIGH);  analogWrite(ENB_FR, clampPWM(spd + TRIM_RIGHT));
  digitalWrite(IN3_AR, LOW); digitalWrite(IN4_AR, HIGH);  analogWrite(ENB_AR, clampPWM(spd + TRIM_RIGHT));
}

void stopMove(){
  analogWrite(ENA_FL,0); digitalWrite(IN1_FL,LOW); digitalWrite(IN2_FL,LOW);
  analogWrite(ENB_FR,0); digitalWrite(IN3_FR,LOW); digitalWrite(IN4_FR,LOW);
  analogWrite(ENA_AL,0); digitalWrite(IN1_AL,LOW); digitalWrite(IN2_AL,LOW);
  analogWrite(ENB_AR,0); digitalWrite(IN3_AR,LOW); digitalWrite(IN4_AR,LOW);

  digitalWrite(RELAY_BRUSH, HIGH);

  movingForward = false;
   movingForwardManual = false;
 
  if (!servoAtZero) {
    int an = map(20,-60,60,0,180);
    myservo.write(an);
    servoAtZero = true;
  }
}

void moveForwardStraight(){ 
  setLeftForward(BASE_SPEED_FWD); setRightForward(BASE_SPEED_FWD); 
  digitalWrite(RELAY_BRUSH, LOW);

  if(!movingForward){
    movingForward = true;
    forwardStartTime = millis();
  }

 

}


void moveBackward(){ 
  setLeftBackward(BASE_SPEED_BACK); setRightBackward(BASE_SPEED_BACK); 
  digitalWrite(RELAY_BRUSH, LOW);

  movingForward = false;
  if (!servoAtZero) {
    int an = map(20,-60,60,0,180);
    myservo.write(an);
    servoAtZero = true;
  }
}

void turnRightSpin() {
  if (fabs(currentTiltAngle) <= 8.0) {
    // ميلان قليل
    setLeftForward(200);
    setRightBackward(200);
  } else {
    // ميلان أكبر
    setLeftForward(200);
    setRightBackward(50);
  }
  digitalWrite(RELAY_BRUSH, HIGH);
}

void turnLeftSpin() {
  if (fabs(currentTiltAngle) <= 8.0) {
    // ميلان قليل
    setRightForward(200);
    setLeftBackward(200);
  } else {
    // ميلان أكبر
    setRightForward(200);
    setLeftBackward(50);
  }
  digitalWrite(RELAY_BRUSH, HIGH);
}

// ======= Ultrasonic helper =======
float singlePing(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 25000);
  if (dur == 0) return 999;
  return dur * 0.0343 / 2.0;
}
float filteredDistance(int trigPin, int echoPin) {
  float a = singlePing(trigPin, echoPin);
  delay(40);
  float b = singlePing(trigPin, echoPin);
  return min(a, b);
}


// ======= Turn with gyro =======
void turnRight90(){
  mpu.update();
  float startAngle = mpu.getAngleZ();

  turnRightSpin();
  while (fabs(mpu.getAngleZ() - startAngle) < 87 ) {
    mpu.update();
  }
  stopMove();
}

void turnLeft90(){
  mpu.update();
  float startAngle = mpu.getAngleZ();
  turnLeftSpin();
  while (fabs(mpu.getAngleZ() - startAngle) < 87 ) {
    mpu.update();
  }
  stopMove();
}

// ======= Patterns =======
void safeForward(unsigned long duration) {
  unsigned long start = millis();
  moveForwardStraight();

  while (millis() - start < duration) {
    float distFront = filteredDistance(TRIG_FRONT, ECHO_FRONT);
    float distDown  = filteredDistance(TRIG_DOWN,  ECHO_DOWN);

    if (distFront < FRONT_STOP_CM) {
      Serial.println("⚠ Obstacle in front!");
      stopMove();
      moveBackward();
      delay(500);
      stopMove();
      lastRun = true; 
      return;
    }
    if (distDown > EDGE_CM) {
      Serial.println("⚠ Edge detected!");
      stopMove();
      moveBackward();
      delay(500);
      stopMove();
      lastRun = true; 
      return;
    }
    delay(50); // تحديث كل 50ms
  }

  stopMove();
}



void cornerPatternRight(){
    updateStepTimes();
  stopMove(); delay(50);
  moveBackward(); delay(BACK_MS);
  stopMove(); delay(50);
  turnRight90(); 
  safeForward(FORWARD_STEPMS);  // بدل delay
  turnRight90(); 
}

void cornerPatternLeft(){
    updateStepTimes();
  stopMove(); delay(50);
  moveBackward(); delay(BACK_MS);
  stopMove(); delay(50);
  turnLeft90(); 
  safeForward(FORWARD_STEPMS);  // بدل delay
  turnLeft90(); 
}

void pumpOn() {
  digitalWrite(RELAY_PUMP, LOW);  // أو HIGH حسب الريلاي عندك
}
void pumpOff() {
  digitalWrite(RELAY_PUMP, HIGH);
}

void brushDown() {
            // إذا مش نازلة أصلاً
    myservo.write(map(-2,-60,60,0,180)); // أنزل السيرفو
    servoIsUp = false;
  
}

void brushUp() {
          // إذا مش مرفوعة أصلاً
    myservo.write(map(20,-60,60,0,180)); // ارفع السيرفو
    servoIsUp = true;
  
}

void setup(){
  Serial.begin(9600);
  Serial1.begin(115200);
  Wire.begin();
  mpu.begin();
  Serial.println("Calibrating gyro...");
  delay(1000);
  mpu.calcOffsets();
  pinMode(BATTERY_PIN, INPUT);
  myservo.attach(servoPin);
  myservo.write(map(20,-60,60,0,180));

  pinMode(IN1_FL,OUTPUT); pinMode(IN2_FL,OUTPUT); pinMode(ENA_FL,OUTPUT);
  pinMode(IN3_FR,OUTPUT); pinMode(IN4_FR,OUTPUT); pinMode(ENB_FR,OUTPUT);
  pinMode(IN1_AL,OUTPUT); pinMode(IN2_AL,OUTPUT); pinMode(ENA_AL,OUTPUT);
  pinMode(IN3_AR,OUTPUT); pinMode(IN4_AR,OUTPUT); pinMode(ENB_AR,OUTPUT);

  pinMode(RELAY_BRUSH, OUTPUT); digitalWrite(RELAY_BRUSH, LOW);
  
  pinMode(RELAY_PUMP, OUTPUT); digitalWrite(RELAY_PUMP, HIGH);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_DOWN,  OUTPUT); pinMode(ECHO_DOWN,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT, INPUT);

  stopMove();
}


void loop() {

 float battPct = readBatteryPercent();

  if (battPct < 25.0) {
    stopMove();       // يوقف كل المحركات والمضخة
    pumpOff();
    brushUp();        // رفع السيرفو لو نازل
    autoMode = false; // إيقاف المنطق التلقائي
    Serial1.println("warning:low_battery");
    delay(500);       // تعطيل لفترة قصيرة لتجنب تكرار الرسائل بسرعة
    return;           // يمنع باقي الكود من التنفيذ
  }
    
 // =====Serial1 =====
    
if (Serial1.available() > 0) {

String cmd = Serial1.readStringUntil('\n');

 cmd.trim();
 Serial.println(cmd);
if (cmd == "forward") {
    float distFront = filteredDistance(TRIG_FRONT, ECHO_FRONT);
    float distDown = filteredDistance(TRIG_DOWN, ECHO_DOWN);

    if (distFront < FRONT_STOP_CM) {
    Serial1.println("warning:front_obstacle");
      stopMove();
    } else if (distDown > EDGE_CM) {
    Serial1.println("warning:edge_detected");
      stopMove();
  } else {
      moveForwardStraight();
        movingForwardManual = true; // NEW: Set the flag to enable continuous check
    }
  } 

   else if (cmd == "backward") {

 moveBackward();
} 

else if (cmd == "left") {

 turnLeft90();

} 

else if (cmd == "right") {

 turnRight90();

}

else if (cmd == "stop") {

stopMove();

} 

else if (cmd == "pump_on") {

 pumpOn();

 } 
 
 else if (cmd == "pump_off") {

 pumpOff();

 } 
 
 else if (cmd == "brush_up") {

 brushUp();

 } 
 
 else if (cmd == "brush_down") {

 brushDown();

 } 
 
 else if (cmd == "auto_start") {

 

 autoMode = true;

 } 
 
 else if (cmd == "auto_stop") {


 autoMode = false;

 stopMove();

 } 
 
 else {

Serial.println("Unknown command.");

 }

 }

    if (movingForwardManual) {
float distFront = filteredDistance(TRIG_FRONT, ECHO_FRONT);
float distDown = filteredDistance(TRIG_DOWN, ECHO_DOWN);

if (distFront < FRONT_STOP_CM) {
Serial1.println("warning:front_obstacle");
stopMove();
} else if (distDown > EDGE_CM) {
Serial1.println("warning:edge_detected");
stopMove();
}
}

 if (millis() - lastSend >= 5000UL) {

 float battPct = readBatteryPercent(); 




 Serial1.print("battery:");

 Serial1.print(battPct, 1);





 lastSend = millis();

 }


     updateTiltAngle();
 // ===== المنطق التلقائي (AUTO MODE) =====

 if (!autoMode) {

turnRightNext = true;

 return;}



 float distFront = filteredDistance(TRIG_FRONT, ECHO_FRONT);

 float distDown = filteredDistance(TRIG_DOWN, ECHO_DOWN);

 float distRight = filteredDistance(TRIG_RIGHT,ECHO_RIGHT);

 float distLeft = filteredDistance(TRIG_LEFT, ECHO_LEFT);



 if (distFront <= 600 && distFront > 0) lastFront = distFront;

 if (distDown <= 600 && distDown > 0) lastDown = distDown;



if (lastRun) {
  moveForwardStraight();

  if (distDown > EDGE_CM) {
    pumpOff();
    stopMove();
    autoMode = false;
    lastRun = false;
    Serial1.println("done:last_run_finished");
  }
  return; 
}

if (lastFront < FRONT_STOP_CM) {

 stopMove();
  Serial1.println("done:front_blocked");

 } 

else if (lastDown > EDGE_CM) {

 pumpOff();

if (turnRightNext) { cornerPatternRight(); turnRightNext = false; }

 else { cornerPatternLeft(); turnRightNext = true; }

 firstSideCheck = true;

 } else {

 // ===== منطق المضخة في AUTO MODE =====

 unsigned long now = millis();

if (pumpState) {



 if (now - lastPumpToggle >= 500) {

 pumpOff();

 pumpState = false;

lastPumpToggle = now;

 }

} else {



 if (now - lastPumpToggle >= 1500) {

pumpOn();

 pumpState = true;

 lastPumpToggle = now;

 }

}

        if (servoAtZero) {
    int an = map(-5,-60,60,0,180);
    myservo.write(an);
    servoAtZero = false;
  }

 moveForwardStraight();
 
  firstSideCheck = true;

 }


}
