// Library used with the ultrasonic distance sensor to interpret the signals
#include <NewPing.h>
#include <Servo.h>

Servo myservo1;  // gripping servo
Servo myservo2;  // lifting servo

//Pin definitions for the necessary inputs and outputs
#define GRIP_SERVO_OUTPUT A3 
#define LIFT_SERVO_OUTPUT A4
#define FORCE_SENSOR A5

#define ECHO_PIN A0
#define TRIGGER_PIN A1
#define MAX_DISTANCE 150 //Max sensing distance for the distance sensor library

//Pin associations that the distance sensor library will communicate with
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

//Default pin connections and binary movement settings from the LAFVIN motor control shield code
const int PWM2A = 11;
const int PWM2B = 3;
const int PWM0A = 6;
const int PWM0B = 5;
const int DIR_CLK = 4;
const int DIR_EN = 7;
const int DATA = 8;
const int DIR_LATCH = 12;

//All of these movement numbers can be composed into a sequence of binary values that corresponds to HIGH and LOW movements
const int Move_Forward = 39;
const int Move_Backward = 216;
const int Left_Move = 116;
const int Right_Move = 139;
const int Right_Rotate = 149;
const int Left_Rotate = 106;
const int Stop = 0;
const int Upper_Left_Move = 36;
const int Upper_Right_Move = 3;
const int Lower_Left_Move = 80;
const int Lower_Right_Move = 136;
const int Drift_Left = 20;
const int Drift_Right = 10;

//Movement settings
int Speed1 = 255;
int Speed2 = 255;
int Speed3 = 255;
int Speed4 = 255;

//Servo incremental movement amount
int movement_amount = 5;

const int OBJECT_DETECTION_DISTANCE = 50; //Max detection range, different from library max distance above
const int GRIP_DISTANCE = 10; //When the object is at or within this range, the grip function will be called
const int FORCE_THRESHOLD = 400; //Analog sensor threshold for the force sensor

//Servo start and max positions for the grip and lifting servos
const int GRIP_START_POS = 180;
const int GRIP_MIN_POS = 90;

const int LIFT_START_POS = 160;
const int LIFT_UP_POS = 80;

bool taskComplete = false; //Boolean constant to be referenced by later if statements 

void setup() {
  Serial.begin(9600);
  //From provided LAFVIN code, specifies type that the pin connection will be
  pinMode(DIR_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(DIR_EN, OUTPUT);
  pinMode(DIR_LATCH, OUTPUT);
  pinMode(PWM0B, OUTPUT);
  pinMode(PWM0A, OUTPUT);
  pinMode(PWM2A, OUTPUT);
  pinMode(PWM2B, OUTPUT);
  //Voltage from the force sensor will be interpreted by the Arduino
  pinMode(FORCE_SENSOR, INPUT);

  //Communication with the library so that it knows what pins to send the servo signals to
  //Two different prefixes are used, allowing for the servos to be communicated with separately
  myservo1.attach(GRIP_SERVO_OUTPUT);
  myservo2.attach(LIFT_SERVO_OUTPUT);
  //Writes the starting position
  myservo1.write(GRIP_START_POS);
  myservo2.write(LIFT_START_POS);
  //Moves to a set initial location (Kind of a relic of the former code)
  // Motor(Move_Forward, 255, 255, 255, 255);
  // delay(1300);
  // Motor(Stop, 0, 0, 0, 0);
}

void loop() {
  if (taskComplete == false) {
    collisionDetection(30000);
  } else {
    Motor(Stop, 0, 0, 0, 0);
  }
}

//Provided LAFVIN motor movement function. Writes the binary direction consisting of HIGH and LOW signals and then the speed for each motor
void Motor(int Dir, int Speed1, int Speed2, int Speed3, int Speed4) {
  analogWrite(PWM2A, Speed1);
  analogWrite(PWM2B, Speed2);
  analogWrite(PWM0A, Speed3);
  analogWrite(PWM0B, Speed4);

  digitalWrite(DIR_LATCH, LOW);
  shiftOut(DATA, DIR_CLK, MSBFIRST, Dir);
  digitalWrite(DIR_LATCH, HIGH);
}

//Function created for the BOOP project that looks for an object, detects it, and moves toward it
//Ranges have been edited to be condusive with grip function
//Only argument is the time that it will run for in the loop
void collisionDetection(unsigned long duration_ms) {
  unsigned long startTime = millis(); //Millis counter that will run in the background to prevent code bocking

  //While loop that runs while the time is less than the specified time and the task has not been completed
  while (millis() - startTime < duration_ms && taskComplete == false) {
    unsigned long elapsed = millis() - startTime; //Difference integer for accurate time tracking
    int distance = sonar.ping_cm(); //Employs the sonar library to take measurements of the distance and then associates with variable 
                                    //To be compared to in later if statements

    Serial.print("Distance: "); //Prints distance for accuracy testing purposes
    Serial.println(distance);
    

    // If close enough, stop and pick up object
    if (distance > 0 && distance <= GRIP_DISTANCE) {
      Motor(Stop, 0, 0, 0, 0);
      delay(200);

      if (GripFunction()) { //If grip function has been completed, lift the object some amount
        LiftFunction();
        taskComplete = true; //Exits loop and stops car moving as in the main loop, the car only moves if this variable is "false"
      }

      return;
    }

    //If object has been detected but is outside of the grip distance, it will move toward it 
    else if (distance > GRIP_DISTANCE && distance < OBJECT_DETECTION_DISTANCE) {
      Motor(Move_Forward, 120, 120, 120, 120);
    }

    // Default search behavior: rotate while detecting
    else {
      Motor(Right_Rotate, 100, 100, 100, 100);
    }

    delay(30); //Delay because alternating readings of 0 and accurate values were occurring, causing the car to malfunction
  }
  //If the loop isn't running, meaning function has been achieved, car wont move
  Motor(Stop, 0, 0, 0, 0);
}

//Function that will run when grip proximity has been achieved
//Claws will close on object by using the associated servo motor until force threshold has been reached
bool GripFunction() {
  int servoPos = GRIP_START_POS; //Initial servo position from setup code section
  int force_reading = analogRead(FORCE_SENSOR); 

  Serial.println("Starting grip"); //For troubleshooting

  //Analog reading of force sensor increases as force increases so when it is less, sufficient grip has not been achieved
  while (force_reading < FORCE_THRESHOLD) {
    servoPos -= movement_amount; //Incremental count for motor movement
    servoPos = constrain(servoPos, GRIP_MIN_POS, GRIP_START_POS); //Limits the movement of the servo to the min and max domain
    
    //Library moves the motor
    myservo1.write(servoPos);
    delay(80); 
    //Take another reading to check if motion should continue
    force_reading = analogRead(FORCE_SENSOR);

    Serial.print("Force reading: ");
    Serial.print(force_reading);
    Serial.print(" | Grip servo position: ");
    Serial.println(servoPos);
    //If outside of range, stop moving
    if (servoPos <= GRIP_MIN_POS) {
      break;
    }
  }
  //Stops function from running after the threshold has been exceeding, indicating that a sufficient hold has been achieved
  if (force_reading >= FORCE_THRESHOLD) {
    Serial.println("Grip successful");
    return true;
  } else {
    Serial.println("Grip failed");
    LiftFunction();
    //Set movement amount to move object to other location
    Motor(Move_Backward, 120, 120, 120, 120);
    delay(1500);
    Motor(Stop, 0, 0, 0, 0);
    //Writes to start position, dropping object
    myservo1.write(GRIP_START_POS);
    myservo2.write(LIFT_START_POS);

    return false;
  }
}
//Short lift function that will pick up object after grip has been achieved, to be called in collision detection function
void LiftFunction() {
  Serial.println("Lifting object");
  myservo2.write(LIFT_UP_POS);
  //Incrementally lifts object until threshold has been achieved
  // for (int pos = LIFT_START_POS; pos <= LIFT_UP_POS; pos += 2) {
  //   myservo2.write(pos);
  //   delay(30);
  // }
  
  Motor(Stop, 0, 0, 0, 0);
}