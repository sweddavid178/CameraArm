


// Select the timers you're using, here ITimer1
#define USE_TIMER_1     false
#define USE_TIMER_2     true
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include <TimerInterrupt.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates

#define NUM_SERVOS    3
// our servo # counter
uint8_t servonum = 0;
uint16_t targetServoPositions[NUM_SERVOS] = {150,150,150};
float currentServoPositions[NUM_SERVOS] = {150,150,150};
float currentServoVelocities[NUM_SERVOS] = {0.0,0.0,0.0};
unsigned long lastUpdateTime[NUM_SERVOS] = {0.0,0.0,0.0};
#define SERVO_ACCL  1.0
#define MAX_VELOCITY 5.0 
#define SERVO_ERROR_RANGE 2


unsigned long lastTime = 0;

/********** STEPPER SECTION ****************/
#define NUM_STEPPERS  3
#define FREQ          10000
 
// defines pins numbers
const int dirPin1 = 2;
const int stepPin1 = 3; 
const int dirPin2 = 4;
const int stepPin2 = 5; 
const int dirPin3 = 6;
const int stepPin3 = 7; 
 
const int btn1 = 8;
const int btn2 = 9;
const int btn3 = 10;
const int btn4 = 11;
const int btn5 = 12;
const int btn6 = 13;

volatile bool step1State = false;
volatile bool step2State = false;
volatile bool step3State = false;


#define STEPPER_ACCL  0.05
#define MAX_STEPPER_VELOCITY  0.5

//#define MANUAL_MODE 0
//#define MANUAL_SERVO 0


#define NUM_STEPS 5
long path[NUM_STEPS][6] = {
  {-1672,3331,-15030, 362,197,479},
  {-1672,1791,-11247, 429,184,479},
  {-2793,1394,-10278, 564,192,247},
  {-3830,257,-10278, 411,192,150},
  {-3830,1340,-15857, 376,185,128}
};
float pathTime[NUM_STEPS] = {
  3.0,
  1.0,
  1.0,
  1.0,
  1.0
};
int pathStep = 0;

volatile long positions[3] = {0,0,0};
volatile float velocities[3] = {0.0,0.0,0.0};
float targetVelocity[3]  = {0.0,0.0,0.0};
float accls[3] = {0.01,0.05,0.05};
volatile int targetPositions[3] = {0.0,0.0,0.0};
volatile int lastPositions[3] = {0.0,0.0,0.0};
volatile unsigned long counts[3] = {0,0,0};
volatile unsigned long count = 0;
int upBtns[3] = {btn1, btn3, btn5};
int downBtns[3] = {btn2, btn4, btn6};
int stepPins[3] = {stepPin1, stepPin2, stepPin3};
int dirPins[3] = {dirPin1, dirPin2, dirPin3};
volatile bool states[3] = {false, false, false};


void TimerHandler() {
#if !defined(MANUAL_MODE) && !defined(MANUAL_SERVO)
  count++;
  updateStepper(0);
  updateStepper(1);
  updateStepper(2);
#endif
}
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin1,OUTPUT); 
  pinMode(dirPin1,OUTPUT);
  pinMode(stepPin2,OUTPUT); 
  pinMode(dirPin2,OUTPUT);
  pinMode(stepPin3,OUTPUT); 
  pinMode(dirPin3,OUTPUT);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(btn5, INPUT_PULLUP);
  pinMode(btn6, INPUT_PULLUP);

  Serial.begin(115200);

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates
  

  delay(3000);
  lastTime = millis();

  // Init timer ITimer1
  ITimer2.init();
  // Frequency in float Hz
  if (ITimer2.attachInterrupt(FREQ, TimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");
}

void incrementPath(void) {
  if (pathStep >= NUM_STEPS) {
    return;
  }
  targetPositions[0] = path[pathStep][0];
  targetPositions[1] = path[pathStep][1];
  targetPositions[2] = path[pathStep][2];
  targetVelocity[0] = calculateVelocity(positions[0], targetPositions[0], pathTime[pathStep]); 
  targetVelocity[1] = calculateVelocity(positions[1], targetPositions[1], pathTime[pathStep]); 
  targetVelocity[2] = calculateVelocity(positions[2], targetPositions[2], pathTime[pathStep]); 
  targetServoPositions[0] = path[pathStep][3];
  targetServoPositions[1] = path[pathStep][4];
  targetServoPositions[2] = path[pathStep][5];
  pathStep++;
  
}

float calculateVelocity(long x, long y, float timeS) {
  long delta = y - x;
  float v = delta / (timeS * FREQ);
  Serial.println(v);
  return fabs(v);
}

bool checkIfStepFinished() {
  for (int num = 0; num < NUM_STEPPERS; num++) {
    int err = targetPositions[num] - positions[num];
    if (abs(err) > 10) {
      return false; 
    }
  }
  return true;
  
}

float absLimit(float in, float maximum) {
  if (fabs(in) > maximum) {
    if (in < 0) {
      return -maximum;
    } else {
      return maximum;
    }
  }
  return in;
}

void updateStepper(uint8_t num) {
  if (num > NUM_STEPPERS) {
    return;
  }
  int err = targetPositions[num] - positions[num];

  if (abs(err) < 2) {
    counts[num] = count;
    return; 
  }

  float delta = count - counts[num];

  if (delta * velocities[num] > 1) {
    digitalWrite(dirPins[num],HIGH);
    states[num] = !states[num];
    digitalWrite(stepPins[num], states[num]);
    positions[num]++;
    counts[num] += 1/fabs(velocities[num]);
  } else if (delta * velocities[num] < -1) {
    digitalWrite(dirPins[num],LOW);
    states[num] = !states[num];
    positions[num]--;
    digitalWrite(stepPins[num], states[num]);
    counts[num] += 1/fabs(velocities[num]);
  }
}

void updateStepperSpeed(uint8_t num) {
  if (num > NUM_STEPPERS) {
    return;
  }

  int err = targetPositions[num] - positions[num];
  float nv = absLimit(float(err) / 5000.0, targetVelocity[num]);
  float vError = nv - velocities[num];
  /*if (fabs(nv) < 0.001) {
    velocities[num] = 0.0;
  }*/
   if (vError > accls[num]) {
    velocities[num] += accls[num];
   } else if (vError < -accls[num]){
    velocities[num] -= accls[num];
    
   }
}

bool stepperUpdate(int stepper) {
  if (digitalRead(upBtns[stepper]) == 0) {
    digitalWrite(dirPins[stepper],HIGH);
    states[stepper] = !states[stepper];
    digitalWrite(stepPins[stepper], states[stepper]);
    positions[stepper]++;
  } else if (digitalRead(downBtns[stepper]) == 0) {
    digitalWrite(dirPins[stepper],LOW);
    states[stepper] = !states[stepper];
    positions[stepper]--;
    digitalWrite(stepPins[stepper], states[stepper]);
  }
}

void updateServo(uint8_t num) {
  if (num > NUM_SERVOS) {
    return;
  }

  float err = targetServoPositions[num] - currentServoPositions[num];
  unsigned long nTime = millis();
  unsigned long delta = nTime - lastUpdateTime[num];
  lastUpdateTime[num] = nTime;

  if (err == 0) {
    //nothing to do 
    return;
  }

  if (abs(err) <= SERVO_ERROR_RANGE) {
    //go ahead and force it to position
    currentServoPositions[num] = targetServoPositions[num];
    pwm.setPWM(num, 0, currentServoPositions[num]);
    
    return;
  }

  float nv = currentServoVelocities[num];
  if (err > 0) {
    nv += SERVO_ACCL * (delta);
  } else {
    nv += -SERVO_ACCL * (delta);
  }

  currentServoVelocities[num] = absLimit(nv, MAX_VELOCITY);
  float pos = currentServoPositions[num] + currentServoVelocities[num];
  currentServoPositions[num] = (uint16_t)round(pos);
  pwm.setPWM(num, 0, (uint16_t) round(currentServoPositions[num]));
}

void manualServo(int num) {
  if (digitalRead(upBtns[num]) == 0) {
    currentServoPositions[num]++;
  } else if (digitalRead(downBtns[num]) == 0) {
    currentServoPositions[num]--;
  }
  pwm.setPWM(num, 0, (uint16_t) round(currentServoPositions[num]));
}



void loop() {
#if defined(MANUAL_SERVO)
  manualServo(0);
  manualServo(1);
  manualServo(2);
  Serial.print(currentServoPositions[0]);
  Serial.print(":");
  Serial.print(currentServoPositions[1]);
  Serial.print(":");
  Serial.print(currentServoPositions[2]);
  Serial.println(":");
  delay(5);
#elif defined( MANUAL_MODE)
  stepperUpdate(0);
  stepperUpdate(1);
  stepperUpdate(2);
  Serial.print(positions[0]);
  Serial.print(":");
  Serial.print(positions[1]);
  Serial.print(":");
  Serial.println(positions[2]);
#else
  int err = targetPositions[1] - positions[1];
  if (!checkIfStepFinished()) {
    updateStepperSpeed(0);
    updateStepperSpeed(1);
    updateStepperSpeed(2);
    updateServo(0);
    updateServo(1);
    updateServo(2);
    Serial.print(pathStep);
    Serial.print(":");
    Serial.print(positions[0]);
    Serial.print(":");
    Serial.print(targetPositions[0]);
    Serial.print(":");
    Serial.print(positions[1]);
    Serial.print(":");
    Serial.print(targetPositions[1]);
    Serial.print(":");
    Serial.print(positions[2]);
    Serial.print(":");
    Serial.println(targetPositions[2]);
  } else {
    incrementPath();
  }
  
  delay(20);
#endif
  
}
