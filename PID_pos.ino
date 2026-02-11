
#include "util/atomic.h"


#define dcin1 4
#define dcin2 7
#define dcen 5
#define encA 2
#define encB 3
#define zeroTol 0.1
#define minpwmBKWD 59
#define minpwmFWD 54

// 2x---->4x
const float encres = 9600.0;
const float dt = 0.01;

// PID
float Kp = 15.0;
float Ki = 0.5;
float Kd = 0.5;

volatile long encoder_pos = 0;
volatile float current_pos = 0.0;
volatile long encoder_pos_old = 0;
volatile float target_pos = 6.0;
volatile float current_speed = 0.0;
volatile float currentSpeedCopy = 0.0;
float target_speed = 6.0;
volatile uint8_t prevState = 0;

volatile int newdata = 0;
volatile int newdataflag = 0;
volatile long prevTime = 0.0;  
volatile long currentTime = 0.0;
int counter = 0;
volatile long time = 0.0;

// PID memory
float u = 0.0, u_old = 0.0;
float error = 0.0, error_prev = 0.0, error_prev2 = 0.0;
float a, b, c;

// Deadzone
int minpwm = 35;

float curSamplePos = 0.0;
 volatile long current_sampledpos = 0.0;

void setup() {
  Serial.begin(115200);

  pinMode(dcin1, OUTPUT);
  pinMode(dcin2, OUTPUT);
  pinMode(dcen, OUTPUT);
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  uint8_t A = digitalRead(encA);
  uint8_t B = digitalRead(encB);
  prevState = (A << 1) | B;

  attachInterrupt(digitalPinToInterrupt(encA), readEncoder4x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), readEncoder4x, CHANGE);


  a = Kp + (Ki * dt / 2.0) + (Kd / dt);
  b = -Kp + (Ki * dt / 2.0) - (2.0 * Kd / dt);
  c = Kd / dt;
  setup_timer();
}
//Timer
void setup_timer() {

  cli();

  TCCR1A = 0;

  TCCR1B = 0;

  TCNT1 = 0;

  OCR1A = 19999;

  TCCR1B |= (1 << WGM12);

  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);

  TIMSK1 |= (1 << OCIE1A);

  sei();  //inttrrupt accept

  Serial.println("(Sistem 9600 4x)");
}

void loop() {

//prevent from data loss
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    newdataflag = newdata;
    newdata = 0;
    currentTime = counter;
    currentSpeedCopy = current_speed;
    curSamplePos = current_pos;
  }

  if (newdataflag == 1) {
    Serial.print(currentTime * dt);
    Serial.print("\t");
    Serial.print(curSamplePos);
    Serial.print("\t");
    Serial.println(current_speed);
  }
}

void setMotor(float pwm_val) {
  int power = (int)abs(pwm_val);



  if (pwm_val > zeroTol) {

    power = minpwmFWD + power;
    power = constrain(power, 0, 255);
   
    digitalWrite(dcin1, LOW);
    digitalWrite(dcin2, HIGH);
    analogWrite(dcen, power);
  } else if (pwm_val < -zeroTol) {  

    power = minpwmBKWD + power;
    power = constrain(power, 0, 255);
 
    digitalWrite(dcin1, HIGH);
    digitalWrite(dcin2, LOW);
    analogWrite(dcen, power);
  } else {  
    digitalWrite(dcin1, LOW);
    digitalWrite(dcin2, LOW);
    analogWrite(dcen, 0);
  }
}
//interrupt service routine
ISR(TIMER1_COMPA_vect) {
 
  counter++;
  newdata = 1;
  time = currentTime * dt;
  current_sampledpos = encoder_pos;

  current_speed = 2 * PI * ((float)(current_sampledpos - encoder_pos_old) / encres) / dt;

  encoder_pos_old = current_sampledpos;

  current_pos = 2 * PI * ((float)current_sampledpos / encres);
 
  //error = target_speed - current_speed;
  error = target_pos - current_pos;


  u = u_old + a * error + b * error_prev + c * error_prev2;

  u_old = u;
 error_prev2 = error_prev;
 error_prev = error;

 setMotor(u);
}

void readEncoder4x() {
  uint8_t A = digitalRead(encA);
  uint8_t B = digitalRead(encB);
  uint8_t currState = (A << 1) | B;

  // 4x State Machine

  if ((prevState == 0 && currState == 1) || (prevState == 1 && currState == 3) || (prevState == 3 && currState == 2) || (prevState == 2 && currState == 0)) {
    encoder_pos++;
  } else if ((prevState == 0 && currState == 2) || (prevState == 2 && currState == 3) || (prevState == 3 && currState == 1) || (prevState == 1 && currState == 0)) {
    encoder_pos--;
  }

  prevState = currState;
}

