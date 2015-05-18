
/* ultrasonic test
 */

#include <Arduino.h>


#ifndef __AVR__
  #define Serial SerialUSB
#endif


#define pinTrigger 5
#define pinEcho    6

#define MAX_ECHO_TIME 17400 // About 300cm range


// HC-SR04 ultrasonic sensor driver
unsigned int readHCSR04(int triggerPin, int echoPin)
{
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  unsigned int uS = (unsigned int)pulseIn(echoPin, HIGH, MAX_ECHO_TIME);
  return uS;
}


void setup(void)
{
  pinMode(pinTrigger , OUTPUT);
  digitalWrite(pinTrigger, LOW); 
  pinMode(pinEcho , INPUT);  

  Serial.begin(19200);
  while (!Serial) ; // required if using Due native port
  Serial.println("START");
}


void loop(void)
{
  unsigned int duration = readHCSR04(pinTrigger, pinEcho);

  // Convert the duration time into a distance
  // No temperature compensation
  unsigned int cm = duration / 58; // At about 22 deg C

  Serial.print(duration);
  Serial.print("us, ");
  Serial.print(cm);
  Serial.println("cm");
  delay(100);
}

