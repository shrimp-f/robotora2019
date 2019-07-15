#include <MsTimer2.h>
#include "Wire.h"
#include "sensorbar.h"
#include <Adafruit_MotorShield.h>

#define CBUFFER_SIZE 100

#define Kp 0.01
#define Ki 0
#define Kd 0.001
#define PID_SCALE 40

//ライントレース用係数
#define L_Kp 1.5 //1.5
#define L_Ki 0
#define L_Kd 0.50 // 0.5
#define L_PID_SCALE 0.1

#define TARGET_MOTOR_VEL_AVERAGE 70.

#define SERIAL_
#define LOOP_DELAY 10

//直角曲がる用フラッグ
int angle_flag = 0; // 0:直線 1:左周りする　-1:右回りする
int angle_count = 10; // 復帰用。ループを繰り返す用

const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)


volatile int motor_Lvel = 0;//motor rotation
volatile int pre_motor_Lvel = 0;//
volatile float wheel_Lvel = 0;
volatile int motor_Rvel = 0;//motor rotation
volatile int pre_motor_Rvel = 0;//
volatile float wheel_Rvel = 0;

// PID
float duty_R = 100.;
float duty_L = 100.;
float dt, preTime;
float P_R, I_R, D_R, preP_R;
float P_L, I_L, D_L, preP_L;
// Line
float diff = 0.;
float L_dt, L_preTime;
float L_P, L_I, L_D, preL_P;

const float reduction_ratio = 280;//減速比
const float wheel_circumference = 1;//mm 円周
const int reset_msec = 100;//msec,timer割り込みの周期

//motor sheild
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);

//line follower array
SensorBar mySensorBar(SX1509_ADDRESS);
CircularBuffer positionHistory(CBUFFER_SIZE);


void Rcount(){
  motor_Rvel++;
}
void Lcount(){
  motor_Lvel++;
}

void reset() {
//  wheel_Rvel = vel_mag * float(motor_Rvel) / reduction_ratio *wheel_circumference;
//  wheel_Lvel = vel_mag * float(motor_Lvel) / reduction_ratio *wheel_circumference;
  pre_motor_Rvel = motor_Rvel;
  pre_motor_Lvel = motor_Lvel;
  motor_Rvel = 0;
  motor_Lvel = 0;
}


void setup() {
  pinMode(13, OUTPUT);

  pinMode(44, OUTPUT);//R motor 
  pinMode(45, OUTPUT);//R
  pinMode(7, OUTPUT);//L
  pinMode(8, OUTPUT);//L


  MsTimer2::set(reset_msec, reset);
  MsTimer2::start();

  attachInterrupt(4, Rcount, FALLING);//19pin
  attachInterrupt(5, Lcount, FALLING);//18pin


  Serial.begin(115200);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  
  AFMS.begin();  // create with the default frequency 1.6KHz


  //For this demo, the IR will only be turned on during reads.
  mySensorBar.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default dark on light
  mySensorBar.clearInvertBits();
  //Other option: light line on dark
  //mySensorBar.setInvertBits();
  
  uint8_t returnStatus = mySensorBar.begin();
  if(returnStatus)
  {
	  Serial.println("sx1509 IC communication OK");
  }
  else
  {
	  Serial.println("sx1509 IC communication FAILED!");
	  while(1);
  }
  Serial.println();


  preTime = micros();
  L_preTime = micros();
  duty_R = 100.0;
  duty_L = 100.0;
}


void rawDrive(int left,int right){

  rightMotor->setSpeed(right);
  rightMotor->run(FORWARD);
//  rightMotor->run(RELEASE);
  // turn on motor
  leftMotor->setSpeed(left);
  leftMotor->run(FORWARD);
}




void loop() {

/****** ラインセンサ読み取り*****/
  //Get the data from the bar and save it to the circular buffer positionHistory.
  int temp = mySensorBar.getDensity();
  if( (temp < 4)&&(temp > 0) )
  {
    positionHistory.pushElement( mySensorBar.getPosition());
  }

    int16_t avePos = positionHistory.averageLast( 10 );
    Serial.print(" avePos = ");
    Serial.print(avePos);
/************/

//  float target_wheel_Lvel = 1500.;
//  float target_wheel_Rvel = 1500.;

  ///// Line trace //////

  // 直角曲がる用
  // ラインセンサの淵に行ったらcount=11にして、count減るまで同じ復帰動作をさせる
/*  float angle_threshold = 60.;
  if(angle_count <= 0){
    if(avePos > angle_threshold ){//右にラインが出たとき
      angle_flag = -1;
      angle_count = 11;
    }else if(avePos < -angle_threshold){
      angle_flag = 1;
      angle_count = 11;
    }else if(angle_count <= 0){
      angle_flag = 0;
    }
  }

  if(angle_count > 0){
    --angle_count;
  }
  Serial.print(" flag = ");
  Serial.print(angle_flag);
  Serial.print(" ");
  Serial.print(" count= ");
  Serial.print(angle_count);
  Serial.print(" ");
*/

  L_dt = (micros() - L_preTime) / 1000000;
  L_preTime = micros();
  L_P  = avePos;

  L_D  = (L_P - preL_P) / L_dt;
  preL_P = L_P;

  diff = L_PID_SCALE* (L_Kp * L_P + L_Kd * L_D);

  Serial.print("diff:");
  Serial.print(diff);

//  float target_motor_Rvel = TARGET_MOTOR_VEL_AVERAGE - diff;
//  float target_motor_Lvel = TARGET_MOTOR_VEL_AVERAGE + diff;
  float target_motor_Rvel = TARGET_MOTOR_VEL_AVERAGE - diff;
  float target_motor_Lvel = TARGET_MOTOR_VEL_AVERAGE + diff;

  Serial.print("target_motor_vel:");
  Serial.print(target_motor_Lvel);
  Serial.print(",");
  Serial.print(target_motor_Rvel);

  /*
  target_motor_Rvel = 60.;
  target_motor_Lvel = 60.;
  */

  ///// Line trace end//////



  ////// PID //////
  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  P_R  = target_motor_Rvel - pre_motor_Rvel;
  P_L  = target_motor_Lvel - pre_motor_Lvel;
//  Serial.print("P ");
//  Serial.println(P);
  
//  I += P * dt;
  D_R  = (P_R - preP_R) / dt;
  D_L  = (P_L - preP_L) / dt;
  preP_R = P_R;
  preP_L = P_L;

//  duty += Kp * P + Ki * I + Kd * D;

  duty_R += PID_SCALE* (Kp * P_R + Kd * D_R);
//  duty_R += PID_SCALE* (Kp * P_R);
  duty_R = constrain(duty_R,0,240);
  duty_L += PID_SCALE* (Kp * P_L + Kd * D_L);
//  duty_L += PID_SCALE* (Kp * P_L);
  duty_L = constrain(duty_L,0,240);
  
  /////////////////


  rawDrive(duty_L, duty_R);

/*/
  if(angle_flag == 0){
    rawDrive(duty_L, duty_R);
  }else if(angle_flag == -1){//右まわり
    rawDrive(duty_L, 0);
  }else if(angle_flag == 1){//右まわり
    rawDrive(0, duty_R);
  }
*/


  /*
  #ifdef SERIAL
    Serial.print("target_motor_vel:");
    Serial.print(target_motor_Lvel);
    Serial.print(",");
    Serial.print(target_motor_Rvel);
    Serial.print(" duty:");
    Serial.print(duty_L);
    Serial.print(",");
    Serial.print(duty_R);
    Serial.print(" motor_vel:");
//    Serial.print(pre_motor_Lvel);
//    Serial.print(" ");
    Serial.println(pre_motor_Lvel);
    Serial.print(",");
    Serial.println(pre_motor_Rvel);

//    SerialMonitor();
  #endif
  */
  Serial.println("");
  delay(LOOP_DELAY);
}


inline void SerialMonitor(){
  
/*  
  Serial.print("wheel_vel: ");
  Serial.print(wheel_Lvel);
  Serial.print(" ");
  Serial.println(wheel_Rvel);
*/
}
