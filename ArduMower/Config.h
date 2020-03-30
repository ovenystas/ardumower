#pragma once

// ------ pins---------------------------------------
//#define PIN_MOTOR_ENABLE  37          // EN motors enable

//Defined by MotorShield
//#define PIN_MOTOR_LEFT_PWM 3          // M1_IN1 left motor PWM pin
//#define PIN_MOTOR_LEFT_DIR 12         // M1_IN2 left motor Dir pin
//#define PIN_MOTOR_LEFT_SENSE A1       // M1_FB  left motor current sense
//#define PIN_MOTOR_LEFT_BRAKE 9        // M1_SF  left motor brake
//
//#define PIN_MOTOR_RIGHT_PWM  11       // M2_IN1 right motor PWM pin
//#define PIN_MOTOR_RIGHT_DIR 13        // M2_IN2 right motor Dir pin
//#define PIN_MOTOR_RIGHT_SENSE A0      // M2_FB  right motor current sense
//#define PIN_MOTOR_RIGHT_BRAKE 8       // M2_SF  right motor brake
//
//#define PIN_MOTOR_MOW_PWM 2           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
//#define PIN_MOTOR_MOW_DIR 29          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
//#define PIN_MOTOR_MOW_SENSE A4        // M1_FB  mower motor current sense
//#define PIN_MOTOR_MOW_BRAKE 26        // M1_SF  mower motor brake   (if using MOSFET/L298N, keep unconnected)
#define PIN_MOTOR_MOW_RPM A11

#define PIN_BUMBER_LEFT 39            // bumper pins
#define PIN_BUMBER_RIGHT 38

#define PIN_DROP_LEFT 45              // drop pins - Dropsensor
#define PIN_DROP_RIGHT 23             // drop pins - Dropsensor

#define PIN_SONAR_CENTER_TRIGGER 24   // ultrasonic sensor pins
#define PIN_SONAR_CENTER_ECHO 22
#define PIN_SONAR_RIGHT_TRIGGER 30
#define PIN_SONAR_RIGHT_ECHO 32
#define PIN_SONAR_LEFT_TRIGGER 34
#define PIN_SONAR_LEFT_ECHO 36

// TODO: Instead of left/right use front/rear
#define PIN_PERIMETER_RIGHT A3        // perimeter
#define PIN_PERIMETER_LEFT A2

#define PIN_LED 5                     // LED
#define PIN_BUZZER 53                 // Buzzer
#define PIN_TILT 35                   // Tilt sensor (required for TC-G158 board)
#define PIN_BUTTON 51                 // digital ON/OFF button
#define PIN_BATTERY_VOLTAGE A5        // battery voltage sensor
#define PIN_BATTERY_SWITCH 4          // battery-OFF switch
#define PIN_CHARGE_VOLTAGE A9         // charging voltage sensor
#define PIN_CHARGE_CURRENT A8         // charge current sensor
#define PIN_CHARGE_RELAY 50           // charge relay
//#define PIN_VOLTAGE_MEASUREMENT A7    // test pin for your own voltage measurements
#define PIN_RAIN 44                   // rain sensor

#define PIN_REMOTE_MOW 53             // remote control mower motor
#define PIN_REMOTE_STEER 6            // remote control steering
#define PIN_REMOTE_SPEED 10           // remote control speed
#define PIN_REMOTE_SWITCH 52          // remote control switch


#define PIN_ODOMETER_LEFT A12         // left odometer sensor
#define PIN_ODOMETER_LEFT_2 A13       // left odometer sensor (optional two-wire)
#define PIN_ODOMETER_RIGHT A14        // right odometer sensor
#define PIN_ODOMETER_RIGHT_2 A15      // right odometer sensor (optional two-wire)

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
#define PFOD_BLUETOOTH_PIN_CODE 1234  // Bluetooth pin code

// ------- perimeter ---------------------------------
#define PERIMETER_USE_DEVELOPER_TEST 0        // New perimeter signal test (developers)
#define PERIMETER_USE_BARKER_CODE 1           // Using Barker code for perimeter signal

// ------- odometer ---------------------------------
#define ODOMETER_TICKS_PER_REVOLUTION 1060  // encoder ticks per one full resolution
#define ODOMETER_TICKS_PER_CM 13.49F        // encoder ticks per cm
#define ODOMETER_WHEELBASE_CM 36.0F         // wheel-to-wheel distance (cm)
#define ODOMETER_SWAP_DIR_LEFT true         // inverse left encoder direction?
#define ODOMETER_SWAP_DIR_RIGHT false       // inverse right encoder direction?

