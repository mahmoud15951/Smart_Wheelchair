#include <Mindwave.h> // Mindwave library to handle the data coming from the Headset\ #include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

Mindwave mindwave;
VR myVR(2, 3);  

uint8_t records[7]; // save record
uint8_t buf[64];

//LED indicator pins
#define QLED A0     //LED indicates Headset data quality
#define GLED A1      //LED indicates moving forward (focus level reached) it turns ON otherwise it turns OFF indicating stop(focus is below desired level)
#define YLED A2      //LED indicates first blink and rotating to the left
#define RLED A3      //LED indicates double blink and rotating to the right

//motor driver pins
#define enA 8//Enable1 L298 Pin enA 
#define in1 4 //Motor1  L298 Pin in1 
#define in2 5 //Motor1  L298 Pin in2 
#define in3 6 //Motor2  L298 Pin in3 
#define in4 7 //Motor2  L298 Pin in4 
#define enB 9 //Enable2 L298 Pin enB 

#define pointA    (0)
#define pointB   (1)
#define returnA    (2)
#define returnB   (3)
#define left    (4)
#define right   (5)
#define forward    (6)
#define backward   (7)


byte trig = 2;                                    //Assign the ultrasonic sensor pins
byte echo = 13;
byte maxDist = 150;                               //Maximum sensing distance (Objects further than this distance are ignored)
byte stopDist = 50;                               //Minimum distance from an object to stop in cm
float timeOut = 2*(maxDist+10)/100/340*1000000;   //Maximum time to wait for a return signal


int gosignal = 0;     // indicator for go or stop
int blinksignal = 0;  // counter for signals after first blink
int blinks = 0;       // number of blinks in a group

int mSpeed = 130;     // variable for Motor speed controlling

unsigned long time;   // variable for elapsed time
unsigned long time0;  // variable for time of first blink
unsigned long time1;  // variable for time from 1st to 2nd blink
void printSignature(uint8_t *buf, int len)
{
  int i;
  for (i = 0; i < len; i++) {
    if (buf[i] > 0x19 && buf[i] < 0x7F) {
      Serial.write(buf[i]);
    }
    else {
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}


void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if (buf[0] == 0xFF) {
    Serial.print("NONE");
  }
  else if (buf[0] & 0x80) {
    Serial.print("UG ");
    Serial.print(buf[0] & (~0x80), DEC);
  }
  else {
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if (buf[3] > 0) {
    printSignature(buf + 4, buf[3]);
  }
  else {
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}



void setup() {
  Serial.begin(MINDWAVE_BAUDRATE);
  myVR.begin(9600);
  Serial.begin(115200);
  Serial.println("Voice Control car");

  pinMode(trig,OUTPUT);                           //Assign ultrasonic sensor pin modes
  pinMode(echo,INPUT);
  
  pinMode(QLED, OUTPUT);   // initialize the LED pins as output
  pinMode(GLED, OUTPUT);
  pinMode(YLED, OUTPUT);
  pinMode(RLED, OUTPUT);

  pinMode(enA, OUTPUT); // declare as output for L298 Pin enA
  pinMode(in1, OUTPUT); // declare as output for L298 Pin in1
  pinMode(in2, OUTPUT); // declare as output for L298 Pin in2
  pinMode(in3, OUTPUT); // declare as output for L298 Pin in3
  pinMode(in4, OUTPUT); // declare as output for L298 Pin in4
  pinMode(enB, OUTPUT); // declare as output for L298 Pin enB
if (myVR.clear() == 0) {
    Serial.println("Recognizer cleared.");
  } else {
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while (1);
  }

  if (myVR.load((uint8_t)pointA) >= 0) {
    Serial.println("pointA loaded");
  }

  if (myVR.load((uint8_t)pointB) >= 0) {
    Serial.println("pointB loaded");
  }
  if (myVR.load((uint8_t)returnA) >= 0) {
    Serial.println("returnA loaded");
  }

  if (myVR.load((uint8_t)returnB) >= 0) {
    Serial.println("returnB loaded");
  }

  if (myVR.load((uint8_t)left) >= 0) {
    Serial.println("left loaded");
  }

  if (myVR.load((uint8_t)right) >= 0) {
    Serial.println("right loaded");
  }

  if (myVR.load((uint8_t)forward) >= 0) {
    Serial.println("forward loaded");
  }

  if (myVR.load((uint8_t)backward) >= 0) {
    Serial.println("backward loaded");
  }
  
}

void onMindwaveData() {
int ret;  
ret = myVR.recognize(buf, 50);
  if (mindwave.quality() < 90) {  // Function for Quality indicating LED, ON if the quality was poor
    digitalWrite(QLED, HIGH);
  } else {
    digitalWrite(QLED, LOW);
  }
  mSpeed = map(mindwave.attention(), 0, 100, 0, 254); // map attention level which is from 0-100 to pwm motor speed output from 0-255
  analogWrite(enA, mSpeed); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed
  analogWrite(enB, mSpeed); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed
  Serial.print("\tspeed: ");
  Serial.print(mSpeed);
  Serial.println();
  // Check Attention Level
  if ((mindwave.attention() > 50) && (gosignal == 0) && (ret = 0) ) {
    Serial.print("Go, ");
    Serial.print("\tattention: ");
    Serial.print(mindwave.attention());
    time = millis();
    Serial.print("\ttime: ");
    Serial.print(time);
    Serial.println();
    digitalWrite(GLED, HIGH);  // green LED on
    gosignal = 1;              // indicates going
    MotorForward();
    delay(10);

  }
  if ((mindwave.attention() < 50) && (gosignal == 1) && (ret = 0)) {
    Serial.print("Stop, ");
    Serial.print("\tattention: ");
    Serial.print(mindwave.attention());
    time = millis();
    Serial.print("\ttime: ");
    Serial.print(time);
    Serial.println();
    digitalWrite(GLED, LOW);  // green LED off
    gosignal = 0;             // indicates stopped going
    MotorStop();
    delay(15);
  }
  // First blink
  if ((mindwave.quality() < 95) && (blinksignal == 0) && (ret = 0)) {
    Serial.print("First blink ");
    Serial.print("\tquality: ");
    Serial.print(mindwave.quality());
    time = millis();
    Serial.print("\ttime: ");
    Serial.print(time);
    Serial.print("\tgosignal: ");
    Serial.print(gosignal);
    Serial.print("\tblinksignal: ");
    Serial.print(blinksignal);
    Serial.println();

    time0 = millis();
    blinks = 1;
    blinksignal = 1;            // first blink
    digitalWrite(YLED, HIGH);   // yellow LED on
  }
  // Display quality right after the blink, & increase the counter
  // for signals from the headset
  if ((blinksignal > 0) && (blinksignal < 4) && (ret = 0)) {
    Serial.print("\tquality: ");
    Serial.print(mindwave.quality());
    time = millis();
    Serial.print("\ttime: ");
    Serial.print(time);
    Serial.print("\tgosignal: ");
    Serial.print(gosignal);
    Serial.print("\tblinksignal: ");
    Serial.print(blinksignal);
    Serial.println();
    if (blinksignal > 0) {
      blinksignal = 1 + blinksignal; // increase counter for signals
    }
    time1 = millis() - time0;
  }
  // Double-blink?
  if ((mindwave.quality() < 95) && (time1 > 500) && (ret = 0)) {
    Serial.print("\tquality: ");
    Serial.print(mindwave.quality());
    time = millis();
    Serial.print("\ttime: ");
    Serial.print(time);
    Serial.print("\tgosignal: ");
    Serial.print(gosignal);
    Serial.print("\tblinksignal: ");
    Serial.print(blinksignal);
    Serial.print("\ttime1: ");
    Serial.print(time1);
    Serial.print("\tblinks now = 2");
    Serial.println();
    blinks = 2;
  }
  // Single blink to turn right
  if ((blinks == 1) && (blinksignal > 3) && (ret = 0)) {
    Serial.print("right, ");
    Serial.print("\tSingle blink ");
    time = millis();
    Serial.print("\ttime: ");
    Serial.print(time);
    Serial.print("\tgosignal: ");
    Serial.print(gosignal);
    Serial.print("\tblinksignal: ");
    Serial.print(blinksignal);
    Serial.println();

    blinksignal = 0;            // reset for next blink(s)
    //   code for robot to turn wheels right for 0.5 seconds
    TurnRight();
    delay(500);
    MotorStop();
    digitalWrite(YLED, LOW);   // yellow LED off
  }
  // Double blink to turn left
  if ((blinks == 2) && (blinksignal > 3) && (ret = 0)) {
    Serial.print("left, ");
    Serial.print("\tDouble blink ");
    time = millis();
    Serial.print("\ttime: ");
    Serial.print(time);
    Serial.print("\tgosignal: ");
    Serial.print(gosignal);
    Serial.print("\tblinksignal: ");
    Serial.print(blinksignal);
    Serial.println();

    digitalWrite(RLED, HIGH);   // red LED on if right turn
    digitalWrite(YLED, LOW);    // yellow LED off
    blinksignal = 0;            // reset for next blink(s)
    //   code for robot to turn wheels left for 0.5 seconds
    TurnLeft();
    delay(500);
    MotorStop();
    digitalWrite(RLED, LOW);    // red LED off
  }
}
void loop() {
  mindwave.update(Serial, onMindwaveData);
  int ret;
  ret = myVR.recognize(buf, 50);
  int distance = getDistance();                   //Check that there are no objects ahead
  if (distance >= stopDist && ret > 0)                      //If there are no objects within the stopping distance, move forward
  {
    switch (buf[1]) {
      case pointA:
        TurnLeft();
        delay(600); 
        MotorForward();
        delay(3000); 
        MotorStop();
        while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  MotorStop();
        break;
        
      case pointB:
        TurnRight();
        delay(600); 
        MotorForward();
        delay(3000);
        MotorStop();
        while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  MotorStop();
        break;

      case returnA:  
        MotorBackward();
        delay(3000); 
        TurnRight();
        delay(600); 
        MotorStop(); 
while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  MotorStop();               
        break;
        
      case returnB:
        MotorForward();
        delay(3000); 
        TurnRight();
        delay(600); 
        MotorStop();
        while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  MotorStop();
        break;

      case left:
        TurnLeft();
        delay(600);
        MotorStop();
        while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  MotorStop();
        break;
        
      case right:
        TurnRight();
        delay(600);
        MotorStop();
        while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  MotorStop();
        break;

      case forward:
        MotorForward();
        delay(500);
        MotorStop();
        while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  MotorStop();
        break;
        
      case backward:
        MotorBackward();
        delay(500);
        MotorStop();
        while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  MotorStop();
        break;        
        
      default:
        Serial.println("Record function undefined");
        break;
    }
  }
}

/* FORWARD */
void MotorForward() {

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW);
}

/* BACKWARD */
void MotorBackward() {

    digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
   digitalWrite(in4, HIGH);}

/* TURN RIGHT */
void TurnRight() {
    digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW);}

/* TURN LEFT */
void TurnLeft() {

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
   digitalWrite(in4, HIGH);}

/* STOP */
void MotorStop() {

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

int getDistance()                                   //Measure the distance to an object
{
  unsigned long pulseTime;                          //Create a variable to store the pulse travel time
  int distance;                                     //Create a variable to store the calculated distance
  digitalWrite(trig, HIGH);                         //Generate a 10 microsecond pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  pulseTime = pulseIn(echo, HIGH, timeOut);         //Measure the time for the pulse to return
  distance = (float)pulseTime * 340 / 2 / 10000;    //Calculate the object distance based on the pulse time
  return distance;
}