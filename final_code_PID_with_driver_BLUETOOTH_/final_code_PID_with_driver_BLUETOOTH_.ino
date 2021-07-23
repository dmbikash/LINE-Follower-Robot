//////////////////////////////////////////////////////////////////////////////////////////////////////////////// SECtion 1



int command = 0;
int c = 0;
int speed = 250;

int forward = 0;
int backward = 0;
int left = 0;
int right = 0;

int result;
int t = 0;
boolean switch_ = LOW;
int d =0;
const int aaa = 7;
const int bbb = 6 ;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////// SECtion 2

#include <QTRSensors.h> 
#define KP .015 
#define KD 25
#define M1_minimum_speed 70  
#define M2_minimum_speed 70 
#define M1_maximum_speed 130 
#define M2_maximum_speed 120 
#define MIDDLE_SENSOR 4       
#define NUM_SENSORS 5         
#define TIMEOUT 2500          
#define EMITTER_PIN 2
         
//#define DEBUG 1

int lastError = 0;
int last_proportional = 0;
int integral = 0;

QTRSensorsRC qtrrc((unsigned char[]) { A4,A3,A2,A1,A0} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);
  
unsigned int sensorValues[NUM_SENSORS];


int leftMotorBackward = 5;     //in 1  
int rightMotorForward = 3;   //  in3
int leftMotorForward = 6;    // in2
int rightMotorBackward = 4;  // in4

int left_motor_speed  =130; 
int right_motor_speed = 130;

int rightMotorENB = 13;       //enable 1
int leftMotorENB  = 12;       // enable 2int leftMotorBackward = 5;     //in 1  


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////   SECtion 3

void setup() {
  
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT); 
  pinMode(leftMotorBackward, OUTPUT);  
  pinMode(rightMotorBackward, OUTPUT);
  pinMode(leftMotorENB, OUTPUT); 
  pinMode(rightMotorENB, OUTPUT);

  pinMode(aaa,OUTPUT);
  pinMode(bbb,INPUT);
  pinMode(aaa,HIGH);

  for(int i=0; i<6; i++){
    pinMode(aaa,HIGH);
  result= digitalRead(bbb);
  }
 
  
}

void loop() {
//Serial.print("result : ");    
//Serial.println(result);

  if (result == 1)
  {
    Serial.println("Welcome to Android mode");
    android_mode();
  }
  else
  {
    //pid
    Serial.println("Welcome to PID mode ");

    if(d<1){
       manual_calibration();
       analogWrite(leftMotorENB,0);
       analogWrite(rightMotorENB,0);
      d++;
    }
    
    unsigned int sensors[5];
    int position = qtrrc.readLine(sensors); 
    int error = position - 2000;
      
    int motorSpeed = KP * error + KD * (error - lastError);
    lastError = error;
      
    int leftMotorSpeed = M1_minumum_speed + motorSpeed;
    int rightMotorSpeed = M2_minumum_speed - motorSpeed;
    
    set_motors(leftMotorSpeed, rightMotorSpeed);  
    
    digitalWrite(leftMotorForward,HIGH);
    digitalWrite(rightMotorForward,HIGH);
  }




  }
/////////////////////////////////////////////////////////////////////////////////////////////   SECtion 4

  void set_motors(int motor1speed, int motor2speed)
    {
    if (motor1speed > M1_maksimum_speed ) motor1speed = M1_maksimum_speed;
    if (motor2speed > M2_maksimum_speed ) motor2speed = M2_maksimum_speed;
    if (motor1speed < 0) motor1speed = 0; 
    if (motor2speed < 0) motor2speed = 0; 
    analogWrite(leftMotorENB,motor1speed);//pwm
    analogWrite(rightMotorENB,motor2speed);//pwm
  }
  
 ///////////////////////////////////////////////////////////////////////////////////////////////   SECtion 5

  
 ///////////////////////////////////////////////////////////////////////////////////////////////////   SECtion 6
void manual_calibration() {
  
int i;
for (i = 0; i < 250; i++)
  {
  qtrrc.calibrate(QTR_EMITTERS_ON);
  delay(20);
  }

}


 /////////////////////////////////////////////////////////////////////////////////////////////// **************

  
 /////////////////////////////////////////////////////////////////////////////////////////////////// SECtion 7


  void android_mode(){
     //Serial.println("Operating in android mode");
     //command = Serial.read();
    // Serial.println(command);
     
  
  if (Serial.available()>0 )
  {
    command = Serial.read();
    Serial.println(command );
    c=c+1;
    Serial.println(c);
    Serial.println("hello...we are getting data");
    
  }
/////////////// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Section 8
  if (command == 1 && forward ==0 )
  {
    digitalWrite(leftMotorForward,HIGH);
    digitalWrite(rightMotorForward,HIGH);
    analogWrite(leftMotorENB,speed);
    analogWrite(rightMotorENB,speed);
     delay(500);
     command=0;
     forward=1;
    
     Serial.print(command);
    
  }

 if(  command == 1 && forward ==1              )
 {
    digitalWrite(leftMotorForward,LOW);
    digitalWrite(rightMotorForward,LOW);
    forward=0;
     command=0;
    
  
  }


     
  /////////////// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SECtion 9


  
  if (command == 2 && right ==0 ) ///right
  {
    digitalWrite(leftMotorForward,HIGH);
    digitalWrite(rightMotorForward,LOW);
    analogWrite(rightMotorENB,speed);
    // motor2.run(FORWARD);
     delay(500);
     command=0;
     right=1;
    
     Serial.print(command);
    
  }

 if(  command == 2 && right ==1              )
 {
    digitalWrite(leftMotorForward,LOW);
    digitalWrite(rightMotorForward,LOW);
    right=0;
    command=0;
    
  
  }
////////////////// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SECtion 10

  if (command == 3 && backward ==0 ) ///down
  {
    digitalWrite(leftMotorBackward,HIGH);
    digitalWrite(rightMotorBackward,HIGH);
    analogWrite(leftMotorENB,speed);
    analogWrite(rightMotorENB,speed);
     delay(500);
     command=0;
     backward=1;
    
     Serial.print(command);
    
  }

 if(  command == 3 && backward ==1              )
 {
   digitalWrite(leftMotorBackward,LOW);
    digitalWrite(rightMotorBackward,LOW);
    backward=0;
    command=0;
    
  
  }
////////////////// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SECtion 1SECtion 11

if (command ==4 && left ==0 ) ///left
  {
    digitalWrite(leftMotorForward,LOW);
    digitalWrite(rightMotorForward,HIGH);
    analogWrite(leftMotorENB,speed);
     delay(500);
     command=0;
     left=1;
    
     Serial.print(command);
    
  }

 if(  command == 4 && left ==1              )
 {
    digitalWrite(leftMotorForward,LOW);
    digitalWrite(rightMotorForward,LOW);
    left=0;
    command=0;
    
  
  }
    
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SECtion 12
  }
