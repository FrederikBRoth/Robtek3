#include "digital_compass.h"

#define buttonPin 8 //
int buttonState = 0;
bool pressed = false;
void setup()
{
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  startMPU();
}
void loop()
{
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW)
  {
    float yaw, pitch, roll;
    getRotationContinuous(&yaw, &pitch, &roll);
    delay(100);
    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.print(" |");
    Serial.print(" Pitch: ");
    Serial.print(pitch, 2);
    Serial.print(" |");
    Serial.print(" Roll: ");
    Serial.println(roll, 2);
    pressed = true;
  } 
  // else if(buttonState == HIGH && pressed) {
  //   pressed = false;
  // }
}