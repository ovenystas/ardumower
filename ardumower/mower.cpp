/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2015 by Alexander Grau
 Copyright (c) 2013-2015 by Sven Gennat
 Copyright (c) 2014 by Maxime Carpentieri
 Copyright (c) 2014-2015 by Stefan Manteuffel
 Copyright (c) 2015 by Uwe Zimprich

 Private-use only! (you need to ask for a commercial-use)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Private-use only! (you need to ask for a commercial-use)
 */

/*
 Ardumower Chassis Kit 1.0 - robot configuration (Ardumower electronics, Arduino Mega)
 http://wiki.ardumower.de/index.php?title=Ardumower_chassis

 Requires: Ardumower PCB v0.5  ( https://www.marotronics.de/Ardumower-Board-Prototyp )
 */

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// NOTE: Verify in config.h that you have enabled 'USE_MOWER' !
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#include "config.h"
#ifdef USE_MOWER

#include <Arduino.h>
#include "mower.h"

// ------ pins---------------------------------------
#define PIN_MOTOR_ENABLE  37          // EN motors enable

#define PIN_MOTOR_LEFT_PWM 3          // M1_IN1 left motor PWM pin
#define PIN_MOTOR_LEFT_DIR 12         // M1_IN2 left motor Dir pin
#define PIN_MOTOR_LEFT_SENSE A1       // M1_FB  left motor current sense
#define PIN_MOTOR_LEFT_FAULT 25       // M1_SF  left motor fault

#define PIN_MOTOR_RIGHT_PWM  11       // M2_IN1 right motor PWM pin
#define PIN_MOTOR_RIGHT_DIR 13        // M2_IN2 right motor Dir pin
#define PIN_MOTOR_RIGHT_SENSE A0      // M2_FB  right motor current sense
#define PIN_MOTOR_RIGHT_FAULT 27      // M2_SF  right motor fault

#define PIN_MOTOR_MOW_PWM 2           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
#define PIN_MOTOR_MOW_DIR 29          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
#define PIN_MOTOR_MOW_SENSE A3        // M1_FB  mower motor current sense
#define PIN_MOTOR_MOW_FAULT 26        // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
#define PIN_MOTOR_MOW_ENABLE 28       // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
#define PIN_MOTOR_MOW_RPM A11

#define PIN_BUMBER_LEFT 39            // bumper pins
#define PIN_BUMBER_RIGHT 38

#define PIN_DROP_LEFT 45              // drop pins - Dropsensor - Absturzsensor
#define PIN_DROP_RIGHT 23             // drop pins - Dropsensor - Absturzsensor

#define PIN_SONAR_CENTER_TRIGGER 24   // ultrasonic sensor pins
#define PIN_SONAR_CENTER_ECHO 22
#define PIN_SONAR_RIGHT_TRIGGER 30
#define PIN_SONAR_RIGHT_ECHO 32
#define PIN_SONAR_LEFT_TRIGGER 34
#define PIN_SONAR_LEFT_ECHO 36

#define PIN_PERIMETER_RIGHT A4        // perimeter
#define PIN_PERIMETER_LEFT A5

#define PIN_LED 13                    // LED
#define PIN_BUZZER 53                 // Buzzer
#define PIN_TILT 35                   // Tilt sensor (required for TC-G158 board)
#define PIN_BUTTON 51                 // digital ON/OFF button
#define PIN_BATTERY_VOLTAGE A2        // battery voltage sensor
#define PIN_BATTERY_SWITCH 4          // battery-OFF switch
#define PIN_CHARGE_VOLTAGE A9         // charging voltage sensor
#define PIN_CHARGE_CURRENT A8         // charge current sensor
#define PIN_CHARGE_RELAY 50           // charge relay
#define PIN_VOLTAGE_MEASUREMENT A7    // test pin for your own voltage measurements
#define PIN_RAIN 44                   // rain sensor

#define PIN_REMOTE_MOW 12             // remote control mower motor
#define PIN_REMOTE_STEER 11           // remote control steering
#define PIN_REMOTE_SPEED 10           // remote control speed
#define PIN_REMOTE_SWITCH 52          // remote control switch


#define PIN_ODOMETRY_LEFT A12         // left odometry sensor
#define PIN_ODOMETRY_LEFT_2 A13       // left odometry sensor (optional two-wire)
#define PIN_ODOMETRY_RIGHT A14        // right odometry sensor
#define PIN_ODOMETRY_RIGHT_2 A15      // right odometry sensor (optional two-wire)

#define PIN_LAWN_FRONT_RECV 40        // lawn sensor front receive
#define PIN_LAWN_FRONT_SEND 41        // lawn sensor front sender
#define PIN_LAWN_BACK_RECV 42         // lawn sensor back receive
#define PIN_LAWN_BACK_SEND 43         // lawn sensor back sender

#define PIN_USER_SWITCH_1 46          // user-defined switch 1
#define PIN_USER_SWITCH_2 47          // user-defined switch 2
#define PIN_USER_SWITCH_3 48          // user-defined switch 3

// IMU (compass/gyro/accel): I2C  (SCL, SDA)
// Bluetooth: Serial2 (TX2, RX2)
// GPS: Serial3 (TX3, RX3)

// ------- baudrates---------------------------------
#define BAUDRATE 115200               // serial output baud rate
#define PFOD_BAUDRATE 19200           // pfod app serial output baud rate
#define PFOD_PIN 1234                 // Bluetooth pin

//#define USE_DEVELOPER_TEST     1      // uncomment for new perimeter signal test (developers)

Mower robot;

Mower::Mower()
{
  name = "Ardumower";

  // ------- wheel motors -----------------------------
  motorAccel = 1000;            // motor wheel acceleration - only functional when odometry is not in use (warning: do not set too low)
  motorSpeedMaxRpm = 25;        // motor wheel max RPM (WARNING: do not set too high, so there's still speed control when battery is low!)
  motorSpeedMaxPwm = 255;       // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
  motorPowerMax = 75;           // motor wheel max power (Watt)
  motorSenseScale[RIGHT] = 15.3;  // motor right sense scale (mA=(ADC-zero)/scale)
  motorSenseScale[LEFT] = 15.3;   // motor left sense scale  (mA=(ADC-zero)/scale)
  motorPowerIgnoreTime = 2000;  // time to ignore motor power (ms)
  motorZeroSettleTime = 3000;   // how long (ms) to wait for motors to settle at zero speed
  motorRollTimeMax = 1500;      // max. roll time (ms)
  motorRollTimeMin = 750;       // min. roll time (ms) should be smaller than motorRollTimeMax
  motorReverseTime = 1200;      // max. reverse time (ms)
  motorForwTimeMax = 80000;     // max. forward time (ms) / timeout
  motorBiDirSpeedRatio1 = 0.3;  // bidir mow pattern speed ratio 1
  motorBiDirSpeedRatio2 = 0.92; // bidir mow pattern speed ratio 2

  // ---- normal control ---
  motorPID[LEFT].Kp = 1.5;        // motor wheel PID controller
  motorPID[LEFT].Ki = 0.29;
  motorPID[LEFT].Kd = 0.25;
  motorPID[RIGHT].Kp = 1.5;        // motor wheel PID controller
  motorPID[RIGHT].Ki = 0.29;
  motorPID[RIGHT].Kd = 0.25;

  // ---- fast control ---
  /*
  motorPID[LEFT].Kp = 1.76;    // motor wheel PID controller
  motorPID[LEFT].Ki = 0.87;
  motorPID[LEFT].Kd = 0.4;
  motorPID[RIGHT].Kp = 1.76;    // motor wheel PID controller
  motorPID[RIGHT].Ki = 0.87;
  motorPID[RIGHT].Kd = 0.4;*/

  motorSwapDir[RIGHT] = 0; // inverse right motor direction?
  motorSwapDir[LEFT] = 0;  // inverse left motor direction?

  // ------ mower motor -------------------------------
  motorMowAccel = 2000;      // motor mower acceleration (warning: do not set too low) 2000 seems to fit best considerating start time and power consumption
  motorMowSpeedMaxPwm = 255; // motor mower max PWM
  motorMowPowerMax = 75.0;   // motor mower max power (Watt)
  motorMowModulate = 0;      // motor mower cutter modulation?
  motorMowRPMSet = 3300;     // motor mower RPM (only for cutter modulation)
  motorSenseScale[MOW] = 15.3; // motor mower sense scale (mA=(ADC-zero)/scale)
  motorPID[MOW].Kp = 0.005;  // motor mower RPM PID controller
  motorPID[MOW].Ki = 0.01;
  motorPID[MOW].Kd = 0.01;

  //  ------ bumper -----------------------------------
  bumperUse = 0; // has bumpers?

  //  ------ drop -----------------------------------
  dropUse = 0;     // has drops? - Dropsensor - Absturzsensor vorhanden ?

  // ------ rain ------------------------------------
  rainUse = 0; // use rain sensor?

  // ------ sonar ------------------------------------
  sonarUse = 0;             // use ultra sonic sensor? (WARNING: robot will slow down, if enabled but not connected!)
  sonarUseArr[Sonar::LEFT] = 1;
  sonarUseArr[Sonar::RIGHT] = 1;
  sonarUseArr[Sonar::CENTER] = 0;
  sonarTriggerBelow = 1050; // ultrasonic sensor trigger distance

  // ------ perimeter ---------------------------------
  perimeterUse = 0;               // use perimeter?
  perimeterTriggerTimeout = 0;    // perimeter trigger timeout when escaping from inside (ms)
  perimeterOutRollTimeMax = 2000; // roll time max after perimeter out (ms)
  perimeterOutRollTimeMin = 750;  // roll time min after perimeter out (ms)
  perimeterOutRevTime = 2200;     // reverse time after perimeter out (ms)
  perimeterTrackRollTime = 1500;  // roll time during perimeter tracking
  perimeterTrackRevTime = 2200;   // reverse time during perimeter tracking
  perimeterPID.Kp = 51.0;         // perimeter PID controller
  perimeterPID.Ki = 12.5;
  perimeterPID.Kd = 0.8;
  trackingPerimeterTransitionTimeOut = 2000;
  trackingErrorTimeOut = 10000;
  trackingBlockInnerWheelWhilePerimeterStruggling = 1;

  // ------ lawn sensor --------------------------------
  lawnSensorUse = 0; // use capacitive Sensor

  // ------  IMU (compass/accel/gyro) ----------------------
  imuUse = 0;          // use IMU?
  imuCorrectDir = 0;   // correct direction by compass?
  imuDirPID.Kp = 5.0;  // direction PID controller
  imuDirPID.Ki = 1.0;
  imuDirPID.Kd = 1.0;
  imuRollPID.Kp = 0.8; // roll PID controller
  imuRollPID.Ki = 21;
  imuRollPID.Kd = 0;

  // ------ model R/C ------------------------------------
  remoteUse = 0; // use model remote control (R/C)?

  // ------ battery -------------------------------------
  batMonitor = 0;              // monitor battery and charge voltage?
  batGoHomeIfBelow = 23.7;     // drive home voltage (Volt)
  batSwitchOffIfBelow = 21.7;  // switch off battery if below voltage (Volt)
  batSwitchOffIfIdle = 1;      // switch off battery if idle (minutes)
  batFactor = 0.495;           // battery conversion factor  / 10 due to arduremote bug, can be removed after fixing (look in robot.cpp)
  batChgFactor = 0.495;        // battery conversion factor  / 10 due to arduremote bug, can be removed after fixing (look in robot.cpp)
  batFull = 29.4;              // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
  batChargingCurrentMax = 1.6; // maximum current your charger can devliver
  batFullCurrent = 0.3;        // current flowing when battery is fully charged
  startChargingIfBelow = 27.0; // start charging if battery Voltage is below
  chargingTimeout = 12600000;  // safety timer for charging (ms) 12600000 = 3.5hrs
  // Sensorausgabe Konsole      (chgSelection = 0)
  // Einstellungen ACS712 5A    (chgSelection = 1 / chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull  = 2)
  // Einstellungen INA169 board (chgSelection = 2)
  chgSelection = 2;
  chgSenseZero = 511;          // charge current sense zero point
  chgFactor = 39;              // charge current conversion factor - Empfindlichkeit nimmt mit ca. 39/V Vcc ab
  chgSense = 185.0;            // mV/A empfindlichkeit des Ladestromsensors in mV/A (Für ACS712 5A = 185)
  chgChange = 0;               // Messwertumkehr von - nach +         1 oder 0
  chgNull = 2;                 // Nullduchgang abziehen (1 oder 2)

  // ------  charging station ---------------------------
  stationRevTime = 1800;   // charge station reverse time (ms)
  stationRollTime = 1000;  // charge station roll time (ms)
  stationForwTime = 1500;  // charge station forward time (ms)
  stationCheckTime = 1700; // charge station reverse check time (ms)

  // ------ odometry ------------------------------------
  odometryUse = 0;                   // use odometry?
  twoWayOdometrySensorUse = 0;       // use optional two-wire odometry sensor?
  odometryTicksPerRevolution = 1060; // encoder ticks per one full resolution
  odometryTicksPerCm = 13.49;        // encoder ticks per cm
  odometryWheelBaseCm = 36;          // wheel-to-wheel distance (cm)
  odometryRightSwapDir = 0;          // inverse right encoder direction?
  odometryLeftSwapDir = 1;           // inverse left encoder direction?

  // ----- GPS -------------------------------------------
  gpsUse = 0;                   // use GPS?
  stuckedIfGpsSpeedBelow = 0.2; // if Gps speed is below given value the mower is stuck
  gpsSpeedIgnoreTime = 5000;    // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)

  // ----- other -----------------------------------------
  buttonUse = 0;            // has digital ON/OFF button?

  // ----- user-defined switch ---------------------------
  userSwitch1 = 0; // user-defined switch 1 (default value)
  userSwitch2 = 0; // user-defined switch 2 (default value)
  userSwitch3 = 0; // user-defined switch 3 (default value)

  // ----- timer -----------------------------------------
  timerUse = 0; // use RTC and timer?

  // ------ mower stats-------------------------------------------
  statsOverride = false; // if set to true mower stats are overwritten with the values below - be careful
  statsMowTimeMinutesTotal = 300;
  statsBatteryChargingCounterTotal = 11;
  statsBatteryChargingCapacityTotal = 30000;

  // -----------configuration end-------------------------------------
}

// remote control (RC) ppm signal change interrupt
ISR(PCINT0_vect)
{
  unsigned long timeMicros = micros();
  boolean remoteSpeedState = digitalRead(PIN_REMOTE_SPEED);
  boolean remoteSteerState = digitalRead(PIN_REMOTE_STEER);
  boolean remoteMowState = digitalRead(PIN_REMOTE_MOW);
  boolean remoteSwitchState = digitalRead(PIN_REMOTE_SWITCH);
  robot.setRemotePPMState(timeMicros, remoteSpeedState, remoteSteerState,
                          remoteMowState, remoteSwitchState);
}

// odometry signal change interrupt
// mower motor speed sensor interrupt
// NOTE: when choosing a higher perimeter sample rate (38 kHz) and using odometry interrupts,
// the Arduino Mega cannot handle all ADC interrupts anymore - the result will be a 'noisy'
// perimeter filter output (mag value) which disappears when disabling odometry interrupts.
// SOLUTION: allow odometry interrupt handler nesting (see odometry interrupt function)
// http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
ISR(PCINT2_vect, ISR_NOBLOCK)
{
  unsigned long timeMicros = micros();
  boolean odometryLeftState = digitalRead(PIN_ODOMETRY_LEFT);
  boolean odometryLeftState2 = digitalRead(PIN_ODOMETRY_LEFT_2);
  boolean odometryRightState = digitalRead(PIN_ODOMETRY_RIGHT);
  boolean odometryRightState2 = digitalRead(PIN_ODOMETRY_RIGHT_2);
  boolean motorMowRpmState = digitalRead(PIN_MOTOR_MOW_RPM);
  robot.setOdometryState(timeMicros, odometryLeftState, odometryRightState,
                         odometryLeftState2, odometryRightState2);
  robot.setMotorMowRPMState(motorMowRpmState);
}

// mower motor speed sensor interrupt
//void rpm_interrupt(){
//}

void Mower::setup()
{
  Wire.begin();
  Console.begin(BAUDRATE);
  Console.println("SETUP");
  rc.initSerial(PFOD_BAUDRATE);

  // keep battery switched ON
  pinMode(PIN_BATTERY_SWITCH, OUTPUT);
  digitalWrite(PIN_BATTERY_SWITCH, HIGH);

  // LED, buzzer, battery
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, 0);
  pinMode(PIN_BATTERY_VOLTAGE, INPUT);
  pinMode(PIN_CHARGE_CURRENT, INPUT);
  pinMode(PIN_CHARGE_VOLTAGE, INPUT);
  pinMode(PIN_CHARGE_RELAY, OUTPUT);
  setActuator(ACT_CHGRELAY, 0);

  // wheel motors
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  digitalWrite(PIN_MOTOR_ENABLE, HIGH);

  // left wheel motor
  pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_SENSE, INPUT);
  pinMode(PIN_MOTOR_LEFT_FAULT, INPUT);

  // right wheel motor
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_SENSE, INPUT);
  pinMode(PIN_MOTOR_RIGHT_FAULT, INPUT);

  // mower motor
  pinMode(PIN_MOTOR_MOW_DIR, OUTPUT);
  pinMode(PIN_MOTOR_MOW_PWM, OUTPUT);
  pinMode(PIN_MOTOR_MOW_SENSE, INPUT);
  pinMode(PIN_MOTOR_MOW_RPM, INPUT);
  pinMode(PIN_MOTOR_MOW_ENABLE, OUTPUT);
  digitalWrite(PIN_MOTOR_MOW_ENABLE, HIGH);
  pinMode(PIN_MOTOR_MOW_FAULT, INPUT);

  // lawn sensor
  lawnSensor.setup(PIN_LAWN_FRONT_SEND, PIN_LAWN_FRONT_RECV,
                   PIN_LAWN_BACK_SEND, PIN_LAWN_BACK_RECV);

  // perimeter
  perimeter.setup(PIN_PERIMETER_LEFT, PIN_PERIMETER_RIGHT);

  // button
  button.setup(PIN_BUTTON);

  // bumpers
  bumper[Bumper::LEFT].setup(PIN_BUMBER_LEFT);
  bumper[Bumper::RIGHT].setup(PIN_BUMBER_RIGHT);

  // drops
  drop[Drop::LEFT].setup(PIN_DROP_LEFT, DROP_CONTACT_NO);
  drop[Drop::RIGHT].setup(PIN_DROP_LEFT, DROP_CONTACT_NO);

  // sonar
  sonar[Sonar::LEFT].setup(PIN_SONAR_LEFT_TRIGGER, PIN_SONAR_LEFT_ECHO);
  sonar[Sonar::RIGHT].setup(PIN_SONAR_RIGHT_TRIGGER, PIN_SONAR_RIGHT_ECHO);
  sonar[Sonar::CENTER].setup(PIN_SONAR_CENTER_TRIGGER, PIN_SONAR_CENTER_ECHO);

  // rain
  pinMode(PIN_RAIN, INPUT);

  // R/C
  pinMode(PIN_REMOTE_MOW, INPUT);
  pinMode(PIN_REMOTE_STEER, INPUT);
  pinMode(PIN_REMOTE_SPEED, INPUT);
  pinMode(PIN_REMOTE_SWITCH, INPUT);

  // odometry
  pinMode(PIN_ODOMETRY_LEFT, INPUT_PULLUP);
  pinMode(PIN_ODOMETRY_LEFT_2, INPUT_PULLUP);
  pinMode(PIN_ODOMETRY_RIGHT, INPUT_PULLUP);
  pinMode(PIN_ODOMETRY_RIGHT_2, INPUT_PULLUP);

  // user switches
  pinMode(PIN_USER_SWITCH_1, OUTPUT);
  pinMode(PIN_USER_SWITCH_2, OUTPUT);
  pinMode(PIN_USER_SWITCH_3, OUTPUT);

  // other
  pinMode(PIN_VOLTAGE_MEASUREMENT, INPUT);

  // PWM frequency setup
  // For obstacle detection, motor torque should be detectable - torque can be computed by motor current.
  // To get consistent current values, PWM frequency should be 3.9 Khz
  // http://wiki.ardumower.de/index.php?title=Motor_driver
  // http://sobisource.com/arduino-mega-pwm-pin-and-frequency-timer-control/
  // http://www.atmel.com/images/doc2549.pdf
  TCCR3B = (TCCR3B & 0xF8) | 0x02;  // set PWM frequency 3.9 Khz (pin2,3,5)

  // enable interrupts
  // R/C
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT4);
  PCMSK0 |= (1 << PCINT5);
  PCMSK0 |= (1 << PCINT6);
  PCMSK0 |= (1 << PCINT1);

  // odometry
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);
  PCMSK2 |= (1 << PCINT21);
  PCMSK2 |= (1 << PCINT22);
  PCMSK2 |= (1 << PCINT23);

  // mower motor speed sensor interrupt
  //attachInterrupt(5, rpm_interrupt, CHANGE);
  PCMSK2 |= (1 << PCINT19);

  // ADC
  ADCMan.init();
  ADCMan.setCapture(PIN_CHARGE_CURRENT, 1, true); //Aktivierung des LaddeStrom Pins beim ADC-Managers
  ADCMan.setCapture(PIN_MOTOR_MOW_SENSE, 1, true);
  ADCMan.setCapture(PIN_MOTOR_LEFT_SENSE, 1, true);
  ADCMan.setCapture(PIN_MOTOR_RIGHT_SENSE, 1, true);
  ADCMan.setCapture(PIN_BATTERY_VOLTAGE, 1, false);
  ADCMan.setCapture(PIN_CHARGE_VOLTAGE, 1, false);
  ADCMan.setCapture(PIN_VOLTAGE_MEASUREMENT, 1, false);

  imu.init(PIN_BUZZER);
  gps.init();

  Robot::setup();
}

void checkMotorFault()
{
  if (digitalRead(PIN_MOTOR_LEFT_FAULT) == LOW)
  {
    robot.addErrorCounter(ERR_MOTOR_LEFT);
    Console.println(F("Error: motor left fault"));
    robot.setNextState(STATE_ERROR, 0);
    //digitalWrite(pinMotorEnable, LOW);
    //digitalWrite(pinMotorEnable, HIGH);
  }
  if (digitalRead(PIN_MOTOR_RIGHT_FAULT) == LOW)
  {
    robot.addErrorCounter(ERR_MOTOR_RIGHT);
    Console.println(F("Error: motor right fault"));
    robot.setNextState(STATE_ERROR, 0);
    //digitalWrite(pinMotorEnable, LOW);
    //digitalWrite(pinMotorEnable, HIGH);
  }
  if (digitalRead(PIN_MOTOR_MOW_FAULT) == LOW)
  {
    robot.addErrorCounter(ERR_MOTOR_MOW);
    Console.println(F("Error: motor mow fault"));
    robot.setNextState(STATE_ERROR, 0);
    //digitalWrite(pinMotorMowEnable, LOW);
    //digitalWrite(pinMotorMowEnable, HIGH);
  }
}

void Mower::resetMotorFault(void)
{
  if (digitalRead(PIN_MOTOR_LEFT_FAULT) == LOW)
  {
    digitalWrite(PIN_MOTOR_ENABLE, LOW);
    digitalWrite(PIN_MOTOR_ENABLE, HIGH);
    Console.println(F("Reset motor left fault"));
  }
  if (digitalRead(PIN_MOTOR_RIGHT_FAULT) == LOW)
  {
    digitalWrite(PIN_MOTOR_ENABLE, LOW);
    digitalWrite(PIN_MOTOR_ENABLE, HIGH);
    Console.println(F("Reset motor right fault"));
  }
  if (digitalRead(PIN_MOTOR_MOW_FAULT) == LOW)
  {
    digitalWrite(PIN_MOTOR_MOW_ENABLE, LOW);
    digitalWrite(PIN_MOTOR_MOW_ENABLE, HIGH);
    Console.println(F("Reset motor mow fault"));
  }
}

int Mower::readSensor(char type)
{
  switch (type)
  {
// motors-----------------------------------------------------------------------
    case SEN_MOTOR_MOW:
      return ADCMan.read(PIN_MOTOR_MOW_SENSE);
      break;
    case SEN_MOTOR_RIGHT:
      checkMotorFault();
      return ADCMan.read(PIN_MOTOR_RIGHT_SENSE);
      break;
    case SEN_MOTOR_LEFT:
      checkMotorFault();
      return ADCMan.read(PIN_MOTOR_LEFT_SENSE);
      break;
    //case SEN_MOTOR_MOW_RPM:
    //  break; // not used - rpm is upated via interrupt

// perimeter--------------------------------------------------------------------
    case SEN_PERIM_LEFT:
      return perimeter.getMagnitude(0);
      break;
      //case SEN_PERIM_RIGHT:
      //  return Perimeter.getMagnitude(1);
      //  break;

// battery----------------------------------------------------------------------
    case SEN_BAT_VOLTAGE:
      ADCMan.read(PIN_VOLTAGE_MEASUREMENT);
      return ADCMan.read(PIN_BATTERY_VOLTAGE);
      break;
    case SEN_CHG_VOLTAGE:
      return ADCMan.read(PIN_CHARGE_VOLTAGE);
      break;
    //case SEN_CHG_VOLTAGE:
    //  return((int)(((double)analogRead(pinChargeVoltage)) * batFactor));
    //  break;
    case SEN_CHG_CURRENT:
      return ADCMan.read(PIN_CHARGE_CURRENT);
      break;

// lawn detector----------------------------------------------------------------
    //case SEN_LAWN_FRONT:
    //  return (measureLawnCapacity(pinLawnFrontSend, pinLawnFrontRecv));
    //  break;
    //case SEN_LAWN_BACK:
    //  return (measureLawnCapacity(PIN_LAWN_BACK_SEND, PIN_LAWN_BACK_RECV));
    //  break;

// imu--------------------------------------------------------------------------
    //case SEN_IMU:
    //  imuYaw = imu.ypr.yaw;
    //  imuPitch = imu.ypr.pitch;
    //  imuRoll = imu.ypr.roll;
    //  break;

// rtc--------------------------------------------------------------------------
    case SEN_RTC:
      if (!readDS1307(datetime))
      {
        Console.println("RTC data error!");
        addErrorCounter(ERR_RTC_DATA);
        setNextState(STATE_ERROR, 0);
      }
      break;

// rain-------------------------------------------------------------------------
    case SEN_RAIN:
      if (digitalRead(PIN_RAIN) == LOW)
      {
        return 1;
      }
      break;

  }
  return 0;
}

void Mower::setActuator(char type, int value)
{
  switch (type)
  {
    case ACT_MOTOR_MOW:
      setArdumoto(PIN_MOTOR_MOW_DIR, PIN_MOTOR_MOW_PWM, value);
      break;
    case ACT_MOTOR_LEFT:
      setArdumoto(PIN_MOTOR_LEFT_DIR, PIN_MOTOR_LEFT_PWM, value);
      break;
    case ACT_MOTOR_RIGHT:
      setArdumoto(PIN_MOTOR_RIGHT_DIR, PIN_MOTOR_RIGHT_PWM, value);
      break;
    case ACT_BUZZER:
      if (value == 0)
      {
        noTone(PIN_BUZZER);
      }
      else
      {
        tone(PIN_BUZZER, value);
      }
      break;
    case ACT_LED:
      digitalWrite(PIN_LED, value);
      break;
    case ACT_USER_SW1:
      digitalWrite(PIN_USER_SWITCH_1, value);
      break;
    case ACT_USER_SW2:
      digitalWrite(PIN_USER_SWITCH_2, value);
      break;
    case ACT_USER_SW3:
      digitalWrite(PIN_USER_SWITCH_3, value);
      break;
    case ACT_RTC:
      if (!setDS1307(datetime))
      {
        Console.println("RTC comm error!");
        addErrorCounter(ERR_RTC_COMM);
        setNextState(STATE_ERROR, 0);
      }
      break;
    case ACT_CHGRELAY:
      digitalWrite(PIN_CHARGE_RELAY, value);
      break;
    //case ACT_CHGRELAY:
    //  digitalWrite(pinChargeRelay, !value);
    //  break;
    case ACT_BATTERY_SW:
      digitalWrite(PIN_BATTERY_SWITCH, value);
      break;
  }
}

void Mower::configureBluetooth(boolean quick)
{
  BluetoothConfig bt;
  bt.setParams(name, PFOD_PIN, PFOD_BAUDRATE, quick);
}

#endif

