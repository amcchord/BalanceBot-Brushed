#include <PulsePosition.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

//Trim for RC Inputs
const int rcMin = 1100;
const int rcMax = 1920;
int rcScale = rcMax - rcMin;

int count=0;
//Channels from the transmitter!
double rc1 = 0;
double rc2 = 0;
double rc3 = 0;
double rc4 = 0;
double rc5 = 0;
double rc6 = 0;
PulsePositionInput myIn;


//Motor Driver Outputs
#define MOTLF 3
#define MOTLR 4
#define MOTRF 5
#define MOTRR 6

//AHRS Vars
#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);
bool inverted = 0;
double targetHeading = 0;
double targetBalance = 5;

//Define Variables we'll be connecting to
double consKp=11, consKi=60, consKd=0.20;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//Define the aggressive and conservative Tuning Parameters


double dKp=1.0, dKi=0.05, dKd=.17;
double driveGoal, driveIn, driveOut;
PID drivePID(&driveIn, &driveOut, &driveGoal, dKp, dKi, dKd, DIRECT);

void setup(){
  Serial.begin(115200);
  Serial.println("Servos: GOOD");
  Setpoint = -0.0;



  myIn.begin(23);

  myPID.SetOutputLimits(-245, 245);
  drivePID.SetOutputLimits(-15,15);
  driveGoal = 0;

  pinMode(13,OUTPUT);
  pinMode(MOTLF, OUTPUT);
  pinMode(MOTLR, OUTPUT);
  pinMode(MOTRF, OUTPUT);
  pinMode(MOTRR, OUTPUT);
  Wire.begin();
  if(!bno.begin())
  {
    Serial.println("AHRS: ERROR");
  }
  Serial.println("AHRS: Calibrating");

  digitalWrite(13, HIGH);
  delay(5000);
  digitalWrite(13, LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  drivePID.SetMode(AUTOMATIC);
  drivePID.SetSampleTime(10);

}

void loop(){

  updateChannels();

  pidDrive();
  pidBalance();
  delay(10);
//  handlePosition();
}

void pidDrive(){
  driveGoal = driveGoal + rc2/10;
  if (driveGoal < -25){
    driveGoal = -25;
  }
  else if (driveGoal > 25){
    driveGoal = 25;
  }

  if (driveIn < -25){
    driveIn = -25;
  }
  else if (driveIn > 25){
    driveIn = 25;
  }
  if (driveIn > 20 && driveGoal > 20){
    driveIn = driveIn - 20;
    driveGoal = driveGoal - 20;
  }
  if (driveIn < -20 && driveGoal < -20){
    driveIn = driveIn + 20;
    driveGoal = driveGoal + 20;
  }

  drivePID.Compute();
  Setpoint = driveOut;

}


void pidBalance(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double currentAngle = euler.z();
  Input = currentAngle;
  myPID.Compute();

  int correctedOut = 0;
  if (Output > 0){
    correctedOut = Output + 4;
  }
  else if (Output < 0){
    correctedOut = Output - 4;
  }
  else {
    correctedOut = Output;
  }


  Serial.print(" trgt: ");
  Serial.print(Setpoint);
  Serial.print(" cur: ");
  Serial.print(currentAngle);
  Serial.print(" resp: ");
  Serial.print(correctedOut);

  Serial.print(" dG: ");
  Serial.print(driveGoal);
  Serial.print(" dI: ");
  Serial.print(driveIn);

  Serial.print(" RC State: ");
  Serial.print(rc1);
  Serial.print(" ");
  Serial.print(rc2);
  Serial.print(" ");
  Serial.print(rc3);
  Serial.print(" ");
  Serial.print(rc4);
  Serial.print(" ");
  Serial.print(rc5);
  Serial.print(" ");
  Serial.print(rc6);
  Serial.println(" ");



  if (currentAngle > 60 || currentAngle < -60){
    simpleDrive(0,0);
    driveGoal = 0.0;
    driveIn = 0.0; //Reset drive delta;

  }
  else {
    if (rc1 < 5 && rc1 > -5){ //Don't apply drive corrections during steering
      driveIn = driveIn + ((correctedOut * -1) / 1000.0);
    } else {
      driveIn = driveIn + (((correctedOut * -1) / 1000.0) / (rc1 / 2)); //Apply them slightly less depending on steering strength.
    }
    simpleDrive((correctedOut * -1), rc1 * -1);
  }

}


void simpleDrive(double thrust, double turn){
  int left = 0;
  int right = 0;

  left = thrust + turn;
  right = thrust - turn;

  if (left > 255){
    left = 255;
  }
  else if (left < -255){
    left = -255;
  }

  if (left > 0){
    // analogWrite(MOTLF, left);
    // digitalWrite(MOTLR, 0);

    analogWrite(MOTLF, left);
    analogWrite(MOTLR, 0);

  } else {
    analogWrite(MOTLF, left * -1);
    digitalWrite(MOTLR, 1);

    // analogWrite(MOTLF, 0);
    // analogWrite(MOTLR, left * -1);
  }

  if (right > 255){
    right = 255;
  }
  else if (right < -255){
    right = -255;
  }

  if (right > 0){
    analogWrite(MOTRF, right);
    digitalWrite(MOTRR, 0);

    // analogWrite(MOTRF, right);
    // analogWrite(MOTRR, 0);

  } else {
    analogWrite(MOTRF, right * -1);
    digitalWrite(MOTRR, 1);

    // analogWrite(MOTRF, 0);
    // analogWrite(MOTRR, right * -1);
  }
}


void handlePosition(){
  /* Display the floating point data */
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print(" ");
  if (abs(euler.z()) > 100 ){
      inverted = 0;
  } else {
      inverted = 1;
  }

}

void updateChannels(){
  int num = myIn.available();
  if (num > 0) {
    rc1 = round(((myIn.read(1) - rcMin)/rcScale) * 150) - 75; //Cast to -250-0-250
    rc2 = round(((myIn.read(2) - rcMin)/rcScale) * 4) - 2; //Cast to -250-0-250
//  rc1 = myIn.read(1);
//  rc2 = myIn.read(2);

    rc3 = myIn.read(3);
    rc4 = myIn.read(4);
    rc5 = myIn.read(5);
    rc6 = 15 - myIn.read(6)/100.0 ;

    if (rc3 > 1800){
      rc1 = 0;
      rc2 = 0;
      rc3 = 0;
      rc4 = 0;
      rc5 = 0;
      rc6 = 0;

    }
  }

}
