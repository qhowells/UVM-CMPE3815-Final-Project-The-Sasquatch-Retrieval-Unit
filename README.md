# UVM-CMPE3815-Final-Project-The-Sasquatch-Retrieval-Unit
The final project for CMPE3815 used a Mecanum wheeled car with an ultrasonic sensor on it to detect objects in a vicinity.

**Hardware Requirements**
* Arduino Uno
* LAFVIN Mecanum wheeled car kit
* Ultrasonic Sensor (HC-SR04)
* 2 servo motors
* Force sensitive resistor (FSR)
* Provided claw and hinge system

**Libraries and Initialization**

The code makes use of the following libraries and initializes them using the necessary code. 

```
  #include <NewPing.h> \\For the ultrasonic sensor
  #include <servo.h>   \\For the servo motors
```

All three components can be connected to any of the six analog pins on the motor control shield. This leaves one pin for the FSR to be connected to. Links to the respective libraries are provided below. 
* Servo.h: https://github.com/arduino-libraries/Servo
* NewPing.h: https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

**How it works**

*1) Search*: Rotates in place, looking for an object in the detection range of the distance sensor

*2) Approach*: Drives toward the object of interest

*3) Clamp*: When in the claw vicinity, close the claws until force threshold is reached

*4) Lift*: Raise object a specified distance

*5) Move*: Move backward some amount, dropping the object in a new location

**Necessary Functions**

These functions are of great importance to the code and they will be provided as examples in the repository 

```
collisionDetection(duration_ms)
```

Controls robots autonomous motion and executes other functions depending on scenario. 

```
GripFunction()
```

Closes claw according to force sensor feedback. 

```
LiftFunction()
```

Raises object using lift servo. 


```
Motor(Dir, Speed1, Speed2, Speed3, Speed4)
```

Function provided by LAFVIN car kit guide that controls the motion type and the speed at which each of the wheels move
