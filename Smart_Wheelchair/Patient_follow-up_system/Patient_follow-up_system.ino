#include <Wire.h>
#include "U8g2lib.h"
#include <MAX30105.h>
#include <heartRate.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

MAX30105 particleSensor;
int32_t lastBeatTime = 0;
float beatsPerMinute;
int beatAvg;

void setup() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_profont15_tf);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Initializing...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 3;
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  Serial.println("Initialization complete!");
}


void loop() {
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 20);
    u8g2.print("SPO2: ");
    u8g2.print(particleSensor.getINT2());
    u8g2.print(" %");
    u8g2.setCursor(0, 40);
    u8g2.print("Heart Rate: ");
    if (checkForBeat(particleSensor.getIR())) {
      int delta = millis() - lastBeatTime;
      lastBeatTime = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        beatAvg = (beatAvg + beatsPerMinute) / 2;
      } 
    }
    u8g2.print(beatAvg);
    u8g2.print(" BPM");
  } while (u8g2.nextPage());
}
