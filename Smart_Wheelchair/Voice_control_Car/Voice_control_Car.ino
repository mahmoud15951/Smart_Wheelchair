#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"
VR myVR(2, 3);  
uint8_t records[7]; // save record
uint8_t buf[64];

/* driver pins */
int in1 = 4;    
int in2 = 5;   
int in3 = 6;     
int in4 = 7;    

#define pointA    (0)
#define pointB   (1)
#define returnA    (2)
#define returnB   (3)
#define left    (4)
#define right   (5)
#define forward    (6)
#define backward   (7)

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

void setup()
{
  /** initialize */
  myVR.begin(9600);

  Serial.begin(115200);
  Serial.println("Voice Control car");

  /* initialize motor control pins as output */
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

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

void loop()
{
  int ret;
  ret = myVR.recognize(buf, 50);
  if (ret > 0) {
    switch (buf[1]) {
      case pointA:
        TurnLeft();
        delay(600); 
        MotorForward();
        delay(3000); 
        MotorStop();
        break;
        
      case pointB:
        TurnRight();
        delay(600); 
        MotorForward();
        delay(3000);
        MotorStop();
        break;

      case returnA:  
        MotorBackward();
        delay(3000); 
        TurnRight();
        delay(600); 
        MotorStop();        
        break;
        
      case returnB:
        MotorForward();
        delay(3000); 
        TurnRight();
        delay(600); 
        MotorStop();
        break;

      case left:
        TurnLeft();
        delay(600);
        MotorStop();
        break;
        
      case right:
        TurnRight();
         delay(600);
        MotorStop();
        break;

      case forward:
        MotorForward();
         delay(500);
        MotorStop();
        break;
        
      case backward:
        MotorBackward();
         delay(500);
        MotorStop();
        break;        
        
      default:
        Serial.println("Record function undefined");
        break;
    }
    /** voice recognized */
    printVR(buf);
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
