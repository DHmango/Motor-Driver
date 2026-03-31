#include <ESP32Encoder.h>
#include <Arduino.h> // I made this in arduino ide because its an ide, and I didn't know how to use platformIO

//TODO implement analog feedback
//                             0   1   2   3
const int EncoderAnalog[4] = { 36, 35, 25, 14 };  //Encoder analog position
const int EncoderA[4]      = { 39, 32, 26, 12 };  //Encoder A channel
const int EncoderB[4]      = { 34, 33, 27, 13 };  //Encoder A channel
const int motorA[4]        = { 23, 19, 5 , 16 };  //Motor A channels
const int motorB[4]        = { 22, 18, 17, 4  };  //Motor B channels

bool ready = false; //for the initialization phase

double velocity[4] = {0};
double position[4] = {0}; // these get frequently updated based on encoder feedback
double integral[4] = {0}; // Even though it represents something different for each, I think I can use this for both pos and vel
double lastError[4] = {0}; // used for derivative component

int mode[4];
double kP[4];
double kI[4];
double kD[4];
double target[4];

const int windowUpdates = 20; // how far back should the velocity look in the array of past encoder positions
const int velocityFreq = 1000; // how many microseconds should come between each check. this*windowupdates is how far back in time it looks. Don't set it too low, or it might get behind
const int refreshrate = (windowUpdates*velocityFreq)/1000000; // ticks/second
int64_t howManyVelocities = 0; // how many total times has the velocity updated
long pastValues[4][windowUpdates] = {0}; // has all the past __ tick values for all 4 encoders
int pastIndex = 0; // the array is like a circle, and we have to move to the next value an loop

long lastLoop = 0; // when, in microseconds since boot, was the last loop
long loopTime; // how many microseconds since the last loop... don't want /0

struct PosAndVel {
  long Position;
  double velocity;
};
ESP32Encoder encoder[4];

HardwareSerial control(0); //change to control(2)
void VelocityPID(int channel, double targeVelocity, double kP, double kI, double kD);
void PositionPID(int channel, long targePosition, double kP, double kI, double kD);
void ManualControl(int channel, double power);

void setup() {
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  for (int i=0; i<4; i++){
    encoder[i].attachFullQuad(EncoderA[i],EncoderB[i]);
    encoder[i].clearCount();
  }
  control.begin(115200); //serial monitor channel. I tried 921600 but it was sometimes returning junk
  //control.begin(115200, SERIAL_8N1, 2, 0) CHANGE THIS ONE TO BE ACTIVE ON ACTUAL HARDWARE
  control.println("Ready for data"); 
  while (!ready) { // you're stuck here until data is here
    delay(10);//prevents the esp from getting mad
    if (control.available()) { // data should come as INIT:[mode,p,i,d] x4 where mode 1=manual,2=position,3=velocity (PID must be passed no matter what)
      String initData = control.readStringUntil('\n');
      if (initData.startsWith("INIT:")) {
        sscanf(initData.c_str(),"INIT:%d,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f",&mode[0],&kP[0],&kI[0],&kD[0],&mode[1],&kP[1],&kI[1],&kD[1],&mode[2],&kP[2],&kI[2],&kD[2],&mode[3],&kP[3],&kI[3],&kD[3]);
      }
    }
  }
}

void loop() {
  int64_t now = esp_timer_get_time();
  loopTime = lastLoop - now;
  lastLoop = now;
  if (control.available()) {
    control.write(control.read());
  }
  for (int i=0; i<4; i++){ //updates every encoder position always
    position[i] = encoder[i].getCount();
  } 
  if (now >= howManyVelocities*velocityFreq) { // update encoder position as frequently as specified
    howManyVelocities++;
    pastIndex = (pastIndex+1) % windowUpdates;
    for (int motor=0; motor<4; motor++) { 
      long oldestVal=pastValues[motor][pastIndex];
      pastValues[motor][pastIndex] = position[motor];
      velocity[motor] = (pastValues[motor][pastIndex]-oldestVal)/refreshrate;
    }
  }
  for (int motor=0; motor<4; motor++) { 
    if (mode[motor]=1){
      ManualControl(motor,target[motor]);
    } else if (mode[motor=2]){
      PositionPID(motor,static_cast<int>(target[motor]),kP[motor],kI[motor],kD[motor]);
    } else {
      VelocityPID(motor,target[motor],kP[motor],kI[motor],kD[motor]);
    }
  }
  if (control.available()>0) { // data should come as target[0],target[1],target[2],target[3] (duh)
    String initData = control.readStringUntil('\n'); //TODO: make this not use string (something something buffer)
    if (initData.startsWith(":")) {
      sscanf(initData.c_str(),"%f,%f,%f,%f",&target[0],&target[1],&target[2],&target[3]);
    }
  }
}
// usually, these constants probably have to be really low because 1 is full power 
void VelocityPID(int channel, double targetVelocity, double kP, double kI, double kD){
  double error = targetVelocity-velocity[channel];
  integral[channel]+=error*loopTime; // I was going to add double I = integral[channel], but idk why I would
  ManualControl(channel, kP*error + kI*integral[channel] + kD*(lastError[channel]-error)/loopTime);
  lastError[channel] = error;
}
void PositionPID(int channel, long targetPosition, double kP, double kI, double kD){ //As far as I know, this can just be the same as the velocity but with position
  double error = targetPosition-position[channel];
  integral[channel]+=error*loopTime;
  ManualControl(channel, kP*error + kI*integral[channel] + kD*(lastError[channel]-error)/loopTime);
  lastError[channel] = error;
}
void ManualControl(int channel, double power) {
  if (power > 0) {
    if (power > 1){ // the range is -1 / 1, but i dont want problems
      power=1;
    }
    analogWrite(motorA[channel],0);
    analogWrite(motorB[channel],round(power*255));
  } else {
    if (power < -1){
      power=-1;
    }
    analogWrite(motorB[channel],0);
    analogWrite(motorA[channel],round(power*-255));
  }
}
