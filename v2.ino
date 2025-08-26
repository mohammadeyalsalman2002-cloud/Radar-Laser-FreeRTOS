#include <Arduino_FreeRTOS.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>


const uint8_t trigPin         = 9;
const uint8_t echoPin         = 2;   
const uint8_t scannerPin      = 6;   
const uint8_t laserServoPin   = 7;   
const uint8_t buzzerPin       = 11;  
const uint8_t laserPin        = 4;


volatile float currentAngle      = 0.0; 
volatile int distance            = -1;
volatile bool hit= false;


volatile int firstAngle          = -1;
volatile int lastAngle           = -1;
volatile int objectTargetAngle   = -1; 
volatile bool objectDetected     = false;
volatile bool laserOn            = false;

volatile unsigned long laserStartTime = 0;
volatile bool laserActive = false;

const int MAX_DIST_CM = 100;
const long MAX_TIMEOUT = MAX_DIST_CM * 58;

const int offsetAngle = -3;

#define T2_PRESC_128   0x05
#define T2_TICK_US     8
#define FRAME_US       20000UL
#define FRAME_TICKS    (FRAME_US / T2_TICK_US)  // 2500

#define SERVO1_PORT PORTD
#define SERVO1_DDR  DDRD
#define SERVO1_BIT  6
#define SERVO2_PORT PORTD
#define SERVO2_DDR  DDRD
#define SERVO2_BIT  7

const int SCAN_MIN_US = 1000, SCAN_MAX_US = 2000;
const int LASR_MIN_US = 1000, LASR_MAX_US = 2000;

// قيم نبض (بوحدة 8µs)
volatile uint16_t s1_ticks = 1500 / T2_TICK_US; // ≈188
volatile uint16_t s2_ticks = 1500 / T2_TICK_US; // ≈188

enum { S_START=0, S_END_FIRST=1, S_END_SECOND=2, S_GAP=3 };
volatile uint8_t  s_state = S_START;
volatile uint8_t  s_firstIsS1 = 1;
volatile uint16_t s_minTicks = 125, s_maxTicks = 250, s_gapRemain = 0;

static inline uint16_t us_to_ticks(int us){
  long t = (us + (T2_TICK_US/2)) / T2_TICK_US;
  if (t < 1) t = 1;
  if (t > 2500) t = 2500;
  return (uint16_t)t;
}

static inline int degToUsClamp(float deg, int usMin, int usMax){
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return (int)(usMin + (usMax - usMin) * (deg / 180.0f));
}

static inline void t2_schedule_next(uint8_t delta){
  OCR2A = (uint8_t)(TCNT2 + delta);
}

void servo_set_deg(uint8_t which, float deg){
  int us = (which==0)
             ? degToUsClamp(deg, SCAN_MIN_US, SCAN_MAX_US)
             : degToUsClamp(deg, LASR_MIN_US,  LASR_MAX_US);
  uint16_t ticks = us_to_ticks(us);
  uint8_t oldSREG = SREG; cli();
  if (which==0) s1_ticks = ticks; else s2_ticks = ticks;
  SREG = oldSREG;
}

ISR(TIMER2_COMPA_vect){
  switch (s_state){
    case S_START: {
      uint16_t a = s1_ticks;
      uint16_t b = s2_ticks;
      SERVO1_PORT |= _BV(SERVO1_BIT);
      SERVO2_PORT |= _BV(SERVO2_BIT);
      if (a <= b){ s_firstIsS1=1; s_minTicks=a; s_maxTicks=b; }
      else       { s_firstIsS1=0; s_minTicks=b; s_maxTicks=a; }
      t2_schedule_next((uint8_t)s_minTicks);
      s_state = S_END_FIRST;
    } break;

    case S_END_FIRST: {
      if (s_firstIsS1) SERVO1_PORT &= ~_BV(SERVO1_BIT);
      else             SERVO2_PORT &= ~_BV(SERVO2_BIT);
      uint16_t diff = (uint16_t)(s_maxTicks - s_minTicks);
      t2_schedule_next(diff==0 ? 1 : (uint8_t)diff);
      s_state = S_END_SECOND;
    } break;

    case S_END_SECOND: {
      if (s_firstIsS1) SERVO2_PORT &= ~_BV(SERVO2_BIT);
      else             SERVO1_PORT &= ~_BV(SERVO1_BIT);
      uint16_t used = s_maxTicks;
      s_gapRemain = (uint16_t)(FRAME_TICKS - used);
      if (s_gapRemain <= 255){ t2_schedule_next((uint8_t)s_gapRemain); s_state=S_START; }
      else { t2_schedule_next(255); s_gapRemain-=255; s_state=S_GAP; }
    } break;

    case S_GAP: {
      if (s_gapRemain <= 255){ t2_schedule_next((uint8_t)s_gapRemain); s_state=S_START; }
      else { t2_schedule_next(255); s_gapRemain -= 255; }
    } break;
  }
}

void servo_driver_init(){
  SERVO1_DDR |= _BV(SERVO1_BIT);
  SERVO2_DDR |= _BV(SERVO2_BIT);
  SERVO1_PORT &= ~_BV(SERVO1_BIT);
  SERVO2_PORT &= ~_BV(SERVO2_BIT);

  TCCR2A = (1<<WGM21);
  TCCR2B = T2_PRESC_128;
  TCNT2  = 0;
  OCR2A  = 10;
  TIMSK2 = (1<<OCIE2A);
  s_state = S_START;
}


long readEchoUS() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, MAX_TIMEOUT);
}

int readDistanceCM() {
  const int N = 5;
  long readings[N];
  for(int i=0;i<N;i++){
    long duration = readEchoUS();
    readings[i] = (duration==0)? MAX_TIMEOUT : duration;
    vTaskDelay(2/portTICK_PERIOD_MS);
  }
  for(int i=0;i<N-1;i++){
    for(int j=i+1;j<N;j++){
      if(readings[j]<readings[i]){
        long t=readings[i]; readings[i]=readings[j]; readings[j]=t;
      }
    }
  }
  long median=readings[N/2];
  if(median==MAX_TIMEOUT) return -1;
  return (int)(median*0.0343/2.0);
}


void TaskScan(void *pvParameters){
  bool forward=true;
  for(;;){

    servo_set_deg(0, currentAngle);
    vTaskDelay(17/portTICK_PERIOD_MS);

    int d = readDistanceCM();
    if(d>0){
      distance=d;
      hit=(d<=50);
    }else{
      distance=-1;
      hit=false;
    }

    if(hit){
      if(firstAngle==-1) firstAngle=(int)currentAngle;
      lastAngle=(int)currentAngle;
      objectDetected=true;
    }else{
      if(firstAngle!=-1){
        objectTargetAngle = firstAngle;
        int objectWidth = lastAngle - firstAngle + 1;

        Serial.print(F("OBJECT,"));
        Serial.print(objectTargetAngle);
        Serial.print(F(","));
        Serial.println(objectWidth);

        firstAngle=-1;
        lastAngle=-1;
        objectDetected=false;
      }
    }


    float speed = 1.0;
    currentAngle += forward ? speed : -speed;
    if(currentAngle>=180) forward=false;
    if(currentAngle<=0) forward=true;
  }
}


void TaskLaser(void *pvParameters){
  static int laserAngleDeg = 90;
  const int maxStep = 8;

  for(;;){
    if(objectTargetAngle>=0){
      int targetAngle = 180 - objectTargetAngle + offsetAngle;
      int diff = targetAngle - laserAngleDeg;
      if(diff!=0){
        int step = min(abs(diff), maxStep);
        laserAngleDeg += (diff>0 ? step : -step);

        servo_set_deg(1, (float)laserAngleDeg);
      }

      if(abs(laserAngleDeg-targetAngle)<=1){
        if(!laserActive){
          laserActive=true;
          laserStartTime=millis();
        }
      }else{
        laserActive=false;
      }
    }

    laserOn = laserActive && (millis()-laserStartTime<=2000);
    digitalWrite(laserPin, laserOn?HIGH:LOW);

    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}


void TaskBuzzerSerial(void *pvParameters){
  for(;;){
    if(hit){
      digitalWrite(buzzerPin,HIGH);
      vTaskDelay(30/portTICK_PERIOD_MS);
      digitalWrite(buzzerPin,LOW);
    }

    Serial.print((int)currentAngle);
    Serial.print(F(","));
    Serial.print(distance);
    Serial.print(F(","));
    Serial.println(hit?1:0);

    vTaskDelay(30/portTICK_PERIOD_MS);
  }
}


void setup(){
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(buzzerPin,OUTPUT);
  digitalWrite(buzzerPin,LOW);

  pinMode(laserPin,OUTPUT);
  digitalWrite(laserPin,LOW);


  servo_driver_init();

  Serial.begin(115200);
  Serial.println(F("BOOT"));


  xTaskCreate(TaskScan,"Scan", 140, NULL, 3, NULL);
  xTaskCreate(TaskLaser,"Laser",120, NULL, 2, NULL);
  xTaskCreate(TaskBuzzerSerial,"Buzzer",110, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop(){
 
}
