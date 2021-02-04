
#include <Servo.h>        // Include Servo Library
#include <NewPing.h>      // Include Newping Library
#define s1 2
#define s2 3
#define s3 4
#define s4 5
#define s5 6
#define TRIGGER_PIN  A1  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A2 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 250 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 250cm.

Servo servo_motor;  // Servo's name
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

boolean goesForward = false;
int distance = 100;

void setup() 
{
 pinMode(26,OUTPUT);   //LEFT_MOTOR
 pinMode(28,OUTPUT);   //LEFT_MOTOR
 pinMode(22,OUTPUT);   //RIGHT_MOTOR
 pinMode(24,OUTPUT);   //RIGHT_MOTOR
 pinMode(11,OUTPUT);  //LEFT_MOTOR_EN1 
 pinMode(10,OUTPUT);  //RIGHT_MOTOR_EN2
 pinMode(s1,INPUT);    //LEFT_SENSE_1
 pinMode(s2,INPUT);    //LEFT_SENSE_2
 pinMode(s3,INPUT);    //LEFT_SENSE_3
 pinMode(s4,INPUT);    //LEFT_SENSE_4 
 pinMode(s5,INPUT);   //LEFT_SENSE_5
 Serial.begin(9600);
   servo_motor.attach(9);   // Attachs the servo on pin 9 to servo object.
  servo_motor.write(115);   // Set at 115 degrees. 
  delay(2000);              // Wait for 2s.
  distance = readPing();    // Get Ping Distance.
  delay(100);               // Wait for 100ms.
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}
void loop() 
{  
     int distanceRight = 0;
  int distanceLeft = 0;
 while(1)
 {
  boolean sv1=digitalRead(s1);
  boolean sv2=digitalRead(s2);
  boolean sv3=digitalRead(s3);
  boolean sv4=digitalRead(s4);
  boolean sv5=digitalRead(s5);
  //OBSTACLE AVOIDING
   if((sv1==LOW)&&(sv2==LOW)&&(sv3==LOW)&&(sv4==LOW)&&(sv5==LOW))      //END
 {
   distanceRight = 0;
   distanceLeft = 0;
  delay(50);

  if (distance <= 10)
  {
    analogWrite(10,160);
    analogWrite(11,160);
    moveStop();
    delay(300);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distanceRight >= distanceLeft)
    {
      analogWrite(10,180 );
      analogWrite(11,180);
      turnRight();
      delay(300);
      moveStop();
    }
    else
    {
      analogWrite(10,180);
      analogWrite(11,180);
      turnLeft();
      delay(300);
      moveStop();
    }
  
  }
  else
  {
        analogWrite(10,160);
    analogWrite(11,160);
    moveForward(); 
  }

    distance = readPing();
 }
 //LINE FOLLOWING
 else if((sv1==LOW)&&(sv2==HIGH)&&(sv3==HIGH)&&(sv4==HIGH)&&(sv5==LOW))   //BLACK_WHITE_BLACK
 {
  analogWrite(11,180);
  analogWrite(10,180);
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
  digitalWrite(26,HIGH);
  digitalWrite(28,LOW);
 } 
  if((sv1==HIGH)&&(sv2==HIGH)&&(sv3==HIGH)&&(sv4==HIGH)&&(sv5==HIGH))   //BLACK_WHITE_BLACK
 {
  analogWrite(11,100);
  analogWrite(10,100);
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
  digitalWrite(26,HIGH);
  digitalWrite(28,LOW);
 } 
 

 else if((sv1==LOW)&&(sv2==HIGH)&&(sv3==HIGH)&&(sv4==HIGH)&(sv5==HIGH))    //SHARP_RIGHT_TURN_BLACK_WHITE_BLACK
 {
  analogWrite(11,50);
  analogWrite(10,180);
  digitalWrite(22,LOW);
  digitalWrite(24,HIGH);
  digitalWrite(26,HIGH);
  digitalWrite(28,LOW);
 }
  else if((sv1==LOW)&&(sv2==LOW)&&(sv3==HIGH)&&(sv4==HIGH)&(sv5==HIGH))    //SHARP_RIGHT_TURN_BLACK_WHITE_BLACK
 {
  analogWrite(11,50);
  analogWrite(10,180);
  digitalWrite(26,HIGH);
  digitalWrite(28,LOW);
  digitalWrite(22,LOW);
  digitalWrite(24,HIGH);
 }
 else if((sv1==HIGH)&&(sv2==HIGH)&&(sv3==HIGH)&&(sv4==HIGH)&&(sv5==LOW))  //SHARP_LEFT_TURN_BLACK_WHITE_BLACK
 { 
  analogWrite(11,180);
  analogWrite(10,50);
  digitalWrite(26,LOW);
  digitalWrite(28,HIGH);
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
 }
  else if((sv1==HIGH)&&(sv2==HIGH)&&(sv3==HIGH)&&(sv4==LOW)&&(sv5==LOW))  //SHARP_LEFT_TURN_BLACK_WHITE_BLACK
 { 
  analogWrite(11,180);
  analogWrite(10,50);
  digitalWrite(26,LOW);
  digitalWrite(28,HIGH);
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
 }
  else if((sv1==HIGH)&&(sv2==LOW)&&(sv3==LOW)&&(sv4==LOW)&&(sv5==LOW))  //LEFT_TURN_BLACK_WHITE_BLACK
 { 
  analogWrite(11,160);
  analogWrite(10,80);
  digitalWrite(26,LOW);
  digitalWrite(28,HIGH);
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
 }
   else if((sv1==LOW)&&(sv2==LOW)&&(sv3==LOW)&&(sv4==LOW)&&(sv5==HIGH))  //RIGHT_TURN_BLACK_WHITE_BLACK
 { 
  analogWrite(11,80);
  analogWrite(10,160);
  digitalWrite(26,HIGH);
  digitalWrite(28,LOW);
  digitalWrite(22,LOW);
  digitalWrite(24,HIGH);
 }
    else if((sv1==LOW)&&(sv2==LOW)&&(sv3==HIGH)&&(sv4==HIGH)&&(sv5==LOW))  //SLIGHT_RIGHT_TURN_BLACK_WHITE_BLACK
    
 { 
  analogWrite(11,180);
  analogWrite(10,120);
  digitalWrite(26,HIGH);
  digitalWrite(28,LOW);
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
 }
     else if((sv1==LOW)&&(sv2==HIGH)&&(sv3==HIGH)&&(sv4==LOW)&&(sv5==LOW))  //SLIGHT_RIGHT_TURN_BLACK_WHITE_BLACK
 { 
  analogWrite(11,120);
  analogWrite(10,180);
  digitalWrite(26,HIGH);
  digitalWrite(28,LOW);
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
 }
 }
}

//USER DEFINED FUNCTIONS

int lookRight()     // Look Right Function for Servo Motor
{  
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft()      // Look Left Function for Servo Motor 
{
  servo_motor.write(180);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int readPing()      // Read Ping Function for Ultrasonic Sensor.
{
  delay(100);                 // Wait 100ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int cm = sonar.ping_cm();   //Send ping, get ping distance in centimeters (cm).
  if (cm==0)
  {
    cm=250;
  }
  return cm;
}

void moveStop()       // Move Stop Function for Motor Driver.
{
  digitalWrite(22, LOW);
  digitalWrite(24, LOW);
  digitalWrite(26, LOW);
  digitalWrite(28, LOW);
}

void moveForward()    // Move Forward Function for Motor Driver.
{
    digitalWrite(22, HIGH);
    digitalWrite(24, LOW);
    digitalWrite(26, HIGH);
    digitalWrite(28, LOW);
}

void moveBackward()   // Move Backward Function for Motor Driver.
{
  digitalWrite(22, LOW);
  digitalWrite(24, HIGH);
  digitalWrite(26, LOW);
  digitalWrite(28, HIGH);
}

void turnRight()      // Turn Right Function for Motor Driver.
{
  digitalWrite(22, LOW);
  digitalWrite(24, HIGH);
  digitalWrite(26, HIGH);
  digitalWrite(28, LOW);
}

void turnLeft()       // Turn Left Function for Motor Driver.
{
  digitalWrite(22, HIGH);
  digitalWrite(24, LOW);
  digitalWrite(26, LOW);
  digitalWrite(28, HIGH);
}
