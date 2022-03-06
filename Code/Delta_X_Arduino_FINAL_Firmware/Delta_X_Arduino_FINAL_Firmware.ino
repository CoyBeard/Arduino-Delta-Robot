#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>


//program Selection:
int RunKinematicDemmo = false;
int RunPicAndPlaceDemmo = false;
int RunProgram1 = true;
int RunProgram2 = false;
int RunProgram3 = false;


// defines pins numbers
const int stepX = 2;
const int dirX  = 5;

const int stepY = 3;
const int dirY  = 6;

const int stepZ = 4;
const int dirZ  = 7;

const int enPin = 8;

const int SOneLimit = 9;
const int STwoLimit = 10;
const int SThreeLimit = 11;

int SOneLimitState = 0;
int STwoLimitState = 0;
int SThreeLimitState = 0;

int SOneLimitReached = false;
int STwoLimitReached = false;
int SThreeLimitReached = false;

int RobotHomed = false;
int S1Homed = false;
int S2Homed = false;
int S3Homed = false;

double S1Position = 0;
double S2Position = 0;
double S3Position = 0;

double S1PreviousPosition = 0;
double S2PreviousPosition = 0;
double S3PreviousPosition = 0;

//Kinamatics

//unchanging 
double S1xSub1 = -2.759;
double S1ySub1 = 0;
double S1zSub1 = -1.063;

double S2xSub1 = 1.38;
double S2ySub1 = 2.389;
double S2zSub1 = -1.063;

double S3xSub1 = 1.38;
double S3ySub1 = -2.389;
double S3zSub1 = -1.063;

double ArmSegmentOneLength = 3.5; //in
double ArmSegmentTwoLength = 7; //in
double J1XOfset = -2.759; //in 
double J1ZOfset = 1.063; //in
double CairageXOfset = 0.962; //in
double CairageZOfset = 0.295; //in
double CairageM2XOfset = 0.481; //in
double CairageM3XOfset = -0.481; //in
double CairageM2YOfset = 0.833; //in
double CairageM3YOfset = -0.833; //in
double M2XOfset = 1.38; //in
double M3XOfset = 1.38; //in
double M2YOfset = 2.389; //in
double M3YOfset = -2.389; //in
double pi = atan(1)*4;


//changed
double S1ImaginaryDistance = 0;
double S2ImaginaryDistance = 0;
double S3ImaginaryDistance = 0;

double S1SideLengthA = 0;
double S2SideLengthA = 0;
double S3SideLengthA = 0;

double S1SideLengthB = 0;
double S2SideLengthB = 0;
double S3SideLengthB = 0;

double S1JLength = 0;
double S2JLength = 0;
double S3JLength = 0;

double S2N = 0;
double S3N = 0;

double S2P = 0;
double S3P = 0;

double S1AngleA = 0;
double S2AngleA = 0;
double S3AngleA = 0;

double S1AngleC = 0;
double S2AngleC = 0;
double S3AngleC = 0;

double S1Angle = 0;
double S2Angle = 0;
double S3Angle = 0;

double S2X2N = 0;
double S2Y2N = 0;
double S3X2N = 0;
double S3Y2N = 0;

double S2X2P = 0;
double S2Y2P = 0;
double S3X2P = 0;
double S3Y2P = 0;

double M2PLO = 3.902; //Parallel Line Ofset
double M3PLO = 3.902; //Parallel Line Ofset

double S1xSub2 = 0;
double S1ySub2 = 0;
double S1zSub2 = 0;

double S2xSub2 = 0;
double S2ySub2 = 0;
double S2zSub2 = 0;

double S3xSub2 = 0;
double S3ySub2 = 0;
double S3zSub2 = 0;

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, 2, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

AccelStepper stepper2(AccelStepper::DRIVER, 3, 6); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

AccelStepper stepper3(AccelStepper::DRIVER, 4, 7); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

Servo Gripper;

Servo Wrist;

const byte enablePin = 8;  // ***** pin 8 is the enable pin

//testing 

int RobotTestPhase = 0;

void setup() {
  // Sets the pins
  pinMode(enablePin, OUTPUT); // **** set the enable pin to output
  digitalWrite(enablePin, LOW); // *** set the enable pin low

  pinMode(stepX,OUTPUT);
  pinMode(dirX,OUTPUT);

  pinMode(stepY,OUTPUT);
  pinMode(dirY,OUTPUT);

  pinMode(stepZ,OUTPUT);
  pinMode(dirZ,OUTPUT);

  Gripper.attach(12);
  Wrist.attach(13);

  pinMode(SOneLimit,INPUT);
  pinMode(STwoLimit,INPUT);
  pinMode(SThreeLimit,INPUT);

  pinMode(enPin,OUTPUT);
  
  digitalWrite(enPin,LOW);

  digitalWrite(dirX,HIGH);

  digitalWrite(dirY,LOW);

  digitalWrite(dirZ,HIGH);

  stepper1.setMaxSpeed(10000000.0);
  stepper1.setAcceleration(10000.0);
    
  stepper2.setMaxSpeed(10000000.0);
  stepper2.setAcceleration(10000.0);
  
  stepper3.setMaxSpeed(10000000.0);
  stepper3.setAcceleration(10000.0);

  //Setup terminal
  Serial.begin(9600);
}

/**NOTES 
 * 
 * 
 * 
 * 1,600 steps per revolution 
 * 
 * (350 / 79) Pully ME
 * 
 * (40 / 9) Microstepping Compensation 
 * 
 * 1 real degree = (14000 / 711) input degrees about 19.5
 * 
 * **/

void CalcMotorAngles (double x, double y, double z) {

  //Motor 1
  //Cairrage offsets
  S1xSub2 = x - 0.962;
  S1ySub2 = y + 0;
  S1zSub2 = z - 0.295;
  
  //A
  S1SideLengthA = sqrt((sq(S1xSub2 - S1xSub1) + sq(S1ySub2 - S1ySub1) + sq(S1zSub1 - S1zSub1)));

  //Debugging
  Serial.print("S1SideLengthA:  ");
  Serial.print(S1SideLengthA);
  Serial.print("\n");
  
  //B
  S1SideLengthB = sqrt((sq(S1xSub2 - S1xSub2) + sq(S1ySub2 - S1ySub1) + sq(S1zSub2 - S1zSub1)));

  //Debugging
  Serial.print("S1SideLengthB:  ");
  Serial.print(S1SideLengthB);
  Serial.print("\n");
  
  //I
  S1ImaginaryDistance = sqrt((sq(S1SideLengthA) + sq(S1SideLengthB)));

  //Debugging
  Serial.print("S1ImaginaryDistance:  ");
  Serial.print(S1ImaginaryDistance);
  Serial.print("\n");

  //C
  S1AngleC = (acos(((sq(S1SideLengthA) + sq(S1ImaginaryDistance) - sq(S1SideLengthB)) / (2 * S1SideLengthA * S1ImaginaryDistance)))) * (180 / pi); 

  //Debugging
  Serial.print("S1AngleC:  ");
  Serial.print(S1AngleC);
  Serial.print("\n");

  //A
  S1AngleA = (acos(((sq(ArmSegmentOneLength) + sq(S1ImaginaryDistance) - sq(ArmSegmentTwoLength)) / (2 * ArmSegmentOneLength * S1ImaginaryDistance)))) * (180 / pi);

  //Debugging
  Serial.print("S1AngleA:  ");
  Serial.print(S1AngleA);
  Serial.print("\n");

  //Final Motor Angle
  S1Angle = ((S1AngleA + S1AngleC + 90 + 36.79) - 360) * -1;

  //Debugging
  Serial.print("S1 Final Motor Angle:  ");
  Serial.print(S1Angle);
  Serial.print("\n");


  //Motor 2   
  //Cairrage offsets
  S2xSub2 = x + 0.481;
  S2ySub2 = y + 0.833;
  S2zSub2 = z - 0.295;
   
  //A
  S2SideLengthA = sqrt((sq(S2xSub2 - S2xSub1) + sq(S2ySub2 - S2ySub1) + sq(S2zSub1 - S2zSub1)));

  //Debugging
  Serial.print("S2SideLengthA:  ");
  Serial.print(S2SideLengthA);
  Serial.print("\n");
  
  //B
  S2SideLengthB = sqrt((sq(S2xSub2 - S2xSub2) + sq(S2ySub2 - S2ySub1) + sq(S2zSub2 - S2zSub1)));

  //Debugging
  Serial.print("S2SideLengthB:  ");
  Serial.print(S2SideLengthB);
  Serial.print("\n");
  
  //I
  S2ImaginaryDistance = sqrt((sq(S2SideLengthA) + sq(S2SideLengthB)));

  //Debugging
  Serial.print("S2ImaginaryDistance:  ");
  Serial.print(S2ImaginaryDistance);
  Serial.print("\n");

  //C
  S2AngleC = (acos(((sq(S2SideLengthA) + sq(S2ImaginaryDistance) - sq(S2SideLengthB)) / (2 * S2SideLengthA * S2ImaginaryDistance)))) * (180 / pi);

  //Debugging
  Serial.print("S2AngleC:  ");
  Serial.print(S2AngleC);
  Serial.print("\n");

  //A
  S2AngleA = (acos(((sq(ArmSegmentOneLength) + sq(S2ImaginaryDistance) - sq(ArmSegmentTwoLength)) / (2 * ArmSegmentOneLength * S2ImaginaryDistance)))) * (180 / pi);

  //Debugging
  Serial.print("S2AngleA:  ");
  Serial.print(S2AngleA);
  Serial.print("\n");

  //Final Motor Angle
  S2Angle = ((S2AngleA + S2AngleC + 90 + 36.79) - 360) * -1;

  //Debugging
  Serial.print("S2 Final Motor Angle:  ");
  Serial.print(S2Angle);
  Serial.print("\n");


  //Motor 3    
  //Cairrage offsets
  S3xSub2 = x + 0.481;
  S3ySub2 = y - 0.833;
  S3zSub2 = z - 0.295;
              
  //A
  S3SideLengthA = sqrt((sq(S3xSub2 - S3xSub1) + sq(S3ySub2 - S3ySub1) + sq(S3zSub1 - S3zSub1)));

  //Debugging
  Serial.print("S3SideLengthA:  ");
  Serial.print(S3SideLengthA);
  Serial.print("\n");
  
  //B
  S3SideLengthB = sqrt((sq(S3xSub2 - S3xSub2) + sq(S3ySub2 - S3ySub1) + sq(S3zSub2 - S3zSub1)));

  //Debugging
  Serial.print("S3SideLengthB:  ");
  Serial.print(S3SideLengthB);
  Serial.print("\n");
  
  //I
  S3ImaginaryDistance = sqrt((sq(S3SideLengthA) + sq(S3SideLengthB)));

  //Debugging
  Serial.print("S3ImaginaryDistance:  ");
  Serial.print(S3ImaginaryDistance);
  Serial.print("\n");

  //C
  S3AngleC = (acos(((sq(S3SideLengthA) + sq(S3ImaginaryDistance) - sq(S3SideLengthB)) / (2 * S3SideLengthA * S3ImaginaryDistance)))) * (180 / pi);

  //Debugging
  Serial.print("S3AngleC:  ");
  Serial.print(S3AngleC);
  Serial.print("\n");

  //A
  S3AngleA = (acos(((sq(ArmSegmentOneLength) + sq(S3ImaginaryDistance) - sq(ArmSegmentTwoLength)) / (2 * ArmSegmentOneLength * S3ImaginaryDistance)))) * (180 / pi);

  //Debugging
  Serial.print("S3AngleA:  ");
  Serial.print(S3AngleA);
  Serial.print("\n");

  //Final Motor Angle
  S3Angle = ((S3AngleA + S3AngleC + 90 + 36.79) - 360) * -1;

  //Debugging
  Serial.print("S3 Final Motor Angle:  ");
  Serial.print(S3Angle);
  Serial.print("\n");
}

void StepperOneMove (int Degrees, int Direction, int SSpeed) {
  //convert steps to degrees 
  Degrees = Degrees * (14000 / 711);
  
  //Sets Direction
  if (Direction == 0) {
    digitalWrite(dirX,LOW);
  }else if (Direction == 1) {
    digitalWrite(dirX,HIGH);
  }

  //Enables the motor to move in a particular direction
  //Makes 200 pulses for making one full cycle rotation
  
  for(int x = 0; x < Degrees; x++) {
    digitalWrite(stepX,HIGH);

    delayMicroseconds(pow(1.06,((SSpeed * -1) + 139.5)));//800 home speed, 10 == 100%, 100 = 50%, 3000 = 1%

    digitalWrite(stepX,LOW);

    delayMicroseconds(pow(1.06,((SSpeed * -1) + 139.5)));//1000

    if (x >= Degrees - 1) {
      if (S1Homed == true) {
        Serial.print("S1 HOMED");
        Serial.print("\n");
        if (Direction == 0) {
          Degrees = (Degrees/(14000 / 711)) * -1;
        } else if (Direction == 1) {
          Degrees = (Degrees/(14000 / 711));
        }
        
        S1Position += Degrees;

        //Debugging
        Serial.print("S1Position:  ");
        Serial.print(S1Position);
        Serial.print("\n");
      }
    }
  }
}

void StepperTwoMove (int Degrees, int Direction, int SSpeed) {
  //convert steps to degrees 
  Degrees = Degrees * (14000 / 711);
  
  //Sets Direction
  if (Direction == 0) {
    digitalWrite(dirY,LOW);
  }else if (Direction == 1) {
    digitalWrite(dirY,HIGH);
  }

  //Enables the motor to move in a particular direction
  //Makes 200 pulses for making one full cycle rotation
  
  for(int x = 0; x < Degrees; x++) {
    digitalWrite(stepY,HIGH);

    delayMicroseconds(pow(1.06,((SSpeed * -1) + 139.5)));//800 home speed, 10 == 100%, 100 = 50%, 3000 = 1%

    digitalWrite(stepY,LOW);

    delayMicroseconds(pow(1.06,((SSpeed * -1) + 139.5)));//1000

    if (x >= Degrees - 1) {
      if (S2Homed == true) {
        Serial.print("S2 HOMED");
        Serial.print("\n");
        if (Direction == 0) {
          Degrees = (Degrees/(14000 / 711)) * -1;
        } else if (Direction == 1) {
          Degrees = (Degrees/(14000 / 711));
        }
        
        S2Position += Degrees;

        //Debugging
        Serial.print("S2Position:  ");
        Serial.print(S2Position);
        Serial.print("\n");
      }
    }
  }
}

void StepperThreeMove (int Degrees, int Direction, int SSpeed) {
  //convert steps to degrees 
  Degrees = Degrees * (14000 / 711);
  
  //Sets Direction
  if (Direction == 0) {
    digitalWrite(dirZ,LOW);
  }else if (Direction == 1) {
    digitalWrite(dirZ,HIGH);
  }

  //Enables the motor to move in a particular direction
  //Makes 200 pulses for making one full cycle rotation
  
  for(int x = 0; x < Degrees; x++) {
    digitalWrite(stepZ,HIGH);

    delayMicroseconds(pow(1.06,((SSpeed * -1) + 139.5)));//800 home speed, 10 == 100%, 100 = 50%, 3000 = 1%

    digitalWrite(stepZ,LOW);

    delayMicroseconds(pow(1.06,((SSpeed * -1) + 139.5)));//1000

    if (x >= Degrees - 1) {
      if (S3Homed == true) {
        Serial.print("S3 HOMED");
        Serial.print("\n");
        if (Direction == 0) {
          Degrees = (Degrees/(14000 / 711)) * -1;
        } else if (Direction == 1) {
          Degrees = (Degrees/(14000 / 711));
        }
        
        S3Position += Degrees;

        //Debugging
        Serial.print("S3Position:  ");
        Serial.print(S3Position);
        Serial.print("\n");
      }
    }
  }
}

void Home () {
  //read S1 limit switch 
  SOneLimitState = digitalRead(SOneLimit);

  //find M1 limit 
  if (SOneLimitState == HIGH && SOneLimitReached == false && STwoLimitReached == false && SThreeLimitReached == false) {
    //move 1 degree at a time 
    StepperOneMove(1, 0, 30);
  }else if (SOneLimitState == LOW) {
    //S1 Homed 
    S1Homed = true;
    S1Position = 0;
    stepper1.setCurrentPosition (0);
    stepper1.moveTo(45 * (14000 / 711));
    while (stepper1.distanceToGo() > 0) {
      stepper1.run();
    }
    
    //toggle var
    if (stepper1.distanceToGo() <= 0) {
      SOneLimitReached = true;
      Serial.print("S1 HOMED");
      Serial.print("\n");
      
      delay(300);
    }
  }
  
  //read S1 limit switch 
  STwoLimitState = digitalRead(STwoLimit);

    //find M2 limit 
  if (STwoLimitState == HIGH && SOneLimitReached == true && STwoLimitReached == false && SThreeLimitReached == false) {
    //move 1 degree at a time 
    StepperTwoMove(1, 0, 30);
  }else if (STwoLimitState == LOW) {
    //S1 Homed 
    S2Homed = true;
    S2Position = 0;
    stepper2.setCurrentPosition (0);
    stepper2.moveTo(45 * (14000 / 711));
    while (stepper2.distanceToGo() > 0) {
      stepper2.run();
    }
    
    //toggle var
    if (stepper2.distanceToGo() <= 0) {
      STwoLimitReached = true;
      Serial.print("S2 HOMED");
      Serial.print("\n");
      
      delay(300);
    }
  }

  //read S1 limit switch 
  SThreeLimitState = digitalRead(SThreeLimit);

    //find M3 limit 
  if (SThreeLimitState == HIGH && SOneLimitReached == true && STwoLimitReached == true && SThreeLimitReached == false) {
    //move 1 degree at a time 
    StepperThreeMove(1, 0, 30);
  }else if (SThreeLimitState == LOW) {
    //S1 Homed 
    S3Homed = true;
    S3Position = 0;
    stepper3.setCurrentPosition (0);
    stepper3.moveTo(45 * (14000 / 711));
    while (stepper3.distanceToGo() > 0) {
      stepper3.run();
    }
    
    //toggle var
    if (stepper3.distanceToGo() <= 0) {
      SThreeLimitReached = true;
      Serial.print("S3 HOMED");
      Serial.print("\n");
        
      delay(300);

      //toggle var
      SThreeLimitReached = true;
      RobotHomed = true;
      S1Homed = false;
      S2Homed = false;
      S3Homed = false;
  
      RobotTestPhase += 1;
  
      delay(2000);
    }
  }
}

void loop() {
  
  if (RobotHomed == false) {
    Home();
  }

  if (RobotHomed == true) {

    if (RunProgram1 == true) {

      Wrist.write(0);

      Gripper.write(0);

      delay(3000);
      
      Wrist.write(180);

      Gripper.write(180);

      delay(3000);
    }

    if (RunProgram2 == true) {
      
    }

    if (RunProgram3 == true) {
      
    }

    if (RunPicAndPlaceDemmo == true) {
      //Move to Point
      CalcMotorAngles(-1, 0, 3);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() !=  0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, 0");
        Serial.print("\n");
        //delay(3000);
      }

      //Move to Point
      CalcMotorAngles(2, 0, 2);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("2, 0");
        Serial.print("\n");
        //delay(3000);
      }

      //Move to Point
      CalcMotorAngles(2, 0, 5);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, 0");
        Serial.print("\n");
        //delay(3000);
      }

      //Move to Point
      CalcMotorAngles(2, 0, 2);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("2, 0");
        Serial.print("\n");
        //delay(3000);
      }

      //Move to Point
      CalcMotorAngles(-1, 0, 3);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() !=  0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, 0");
        Serial.print("\n");
        //delay(3000);
      }

      //Move to Point
      CalcMotorAngles(-1, 0, 5);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() !=  0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, 0");
        Serial.print("\n");
        //delay(3000);
      }
      
    }
    
    if (RunKinematicDemmo == true) {
      //Move to Point
      CalcMotorAngles(0, 0, 4);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, 0");
        Serial.print("\n");
        //delay(3000);
      }

      //Move to Point
      CalcMotorAngles(0, 0, 6);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, 0");
        Serial.print("\n");
        //delay(3000);
      }

      //Move to Point
      CalcMotorAngles(0, 0, 4);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, 0");
        Serial.print("\n");
        //delay(3000);
      }
          
      //Move to Point
      CalcMotorAngles(2, 0, 4);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("2, 0");
        Serial.print("\n");
        //delay(3000);
      }
  
      //Move to Point 
      CalcMotorAngles(0, -2, 4);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, -2");
        Serial.print("\n");
        //delay(3000);
      }
  
      //Move to Point 
      CalcMotorAngles(-2, 0, 4);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("-2, 0");
        Serial.print("\n");
        //delay(3000);
      }
  
      //Move to Point 
      CalcMotorAngles(0, 2, 4);
      stepper1.moveTo(S1Angle * (14000 / 711));
      stepper2.moveTo(S2Angle * (14000 / 711));
      stepper3.moveTo(S3Angle * (14000 / 711));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      } if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
        Serial.print("0, 2");
        Serial.print("\n");
        delay(2000);
      }
    }

  }

}
