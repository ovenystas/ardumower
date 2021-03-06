# Scratchpad

## Motor object hierarchy
                  Motor
                /        \
    MotorWheel               MotorMow
     /       \                  |
Ardumoto DeekRobotMotorShield Mosfet

## New classes
Rain  
Odometer  
Motor  
MotorWheel  
MotorMow  
Ardumoto  
DeekRobotMotorShield  
Mosfet  

## Keys
'd' Menu  
'v' Change consoleMode  
'h' Drive home  
't' Track perimeter  
'l' Simulate left bumper  
'r' Simulate right bumper  
'j' Simulate left drop sensor  
'k' Simulate right drop sensor  
's' Simulate lawn sensor  
'm' Toggle cutter motor  
'c' Simulate in station  
'a' Simulate in station charging  
'+' Rotate 90 degrees clockwise (IMU)  
'-' Rotate 90 degrees anti-clockwise (IMU)  
'i' Toggle imu.use  
'3' Activate model RC  
'0' Turn OFF  
'1' Automode  

## Button
Beeps Mode  
  '1'   Start normal with random moving / OFF'  
  '2'   Start normal with bidir moving'  
  '3'   Remote control  
  '4'   Start normal without perimeter  
  '5'   Drive home  
  '6'   Track perimeter  
  '7'   Start normal with lanes mowing  

## IMU
Don't have:
* GY-80 Multi Sensor Board - 3 Axis Gyro -3 Axis Accelerometer - 3 Axis Magnetometer - Barometer - Thermometer

Have:
* GY-273 HMC5883L 3-AXIS DIGITAL COMPASS MODULE
* GY-521 MPU6050 Triple 3-Axis Accelerometer Gyroscope I2C

Will use:
* MinIMU-9 Gyro, Accelerometer, and Compass (L3G4200D and LSM303DLM Carrier) <https://www.pololu.com/product/1265>
* Arduino code for L3G4200D - <https://github.com/pololu/l3g-arduino>
* Arduino code for LSM303DLM - <https://github.com/pololu/lsm303-arduino>
