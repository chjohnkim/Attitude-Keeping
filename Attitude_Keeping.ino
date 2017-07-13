#include <Wire.h> // include wire library so we can communicate with gyro
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

//===========================================================================
// Data Variables
//===========================================================================
// Motor Data================================================================
int motorStop = 1500; // Pulse width to stop motor
int pulse = 1500; // Pulse width to be sent to motor in PID control
int thrust = 5; // Step thrust to control motor mannually

// Control Data==============================================================
//int control = 0; // Control output 
int xPropGain = 5; // Proportional Gain constant 
int yPropGain = 5;
int zPropGain = 5;
double xPropOutput;
int xIntegralGain = 1; //Integral Gain Constant 
double xIntegralOutput;
int xDerivativeGain = 1; // Derivative Gain Constant
double xDerivativeOutput;
double eulXPrev;
double xOutput, yOutput, zOutput;


// Quaternion Data===========================================================
imu::Quaternion Qref; // reference position in quaternion form
double qRef1 = Qref.w(), qRef2 = Qref.x(), qRef3 = Qref.y(), qRef4 = Qref.z(); // Desired/Reference/Home Orientation 
double qMeas1, qMeas2, qMeas3, qMeas4; // Actual Orientation from sensor
double qMeasCon1, qMeasCon2, qMeasCon3, qMeasCon4; // Conjugate of measured quaternion to calculate relative error
double qErr1, qErr2, qErr3, qErr4; // Calculated relative error values with respect to inertial frame
double qMeasPrev1=1, qMeasPrev2=0, qMeasPrev3=0, qMeasPrev4=0; //Temporary storage variables to filter out sensor error
double qLength; //length of unit quaternion
double eulX, eulY, eulZ; // variables for quaternion to euler conversion 
double qRefCon1, qRefCon2, qRefCon3, qRefCon4; //Conjugate of the desired/reference/home orientation 
double temp1, temp2, temp3, temp4; //Temporary storage for converting relative error from inertial frame to home/body frame
double qErrHome1, qErrHome2, qErrHome3, qErrHome4; //Relative Error with respect to inertial frame 

//===========================================================================
// Helper Variables
//===========================================================================

int s = 0; // temporary variable to read the serial monitor command
// Quaternion Data===========================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29); // create sensor object
char rx_byte = 0; // trigger variable from serial monitor to change desired orientation

//===========================================================================
// Main Setup Function
//===========================================================================
void setup() {
//Inititate motor at stop pulse
motor1.attach(3);
motor2.attach(5);
motor3.attach(6);
motor4.attach(9);
motor1.writeMicroseconds(motorStop);
motor2.writeMicroseconds(motorStop);
motor3.writeMicroseconds(motorStop);
motor4.writeMicroseconds(motorStop);

Serial.begin(9600); // start the serial monitor for logging
if(bno.begin()) Serial.println("Sensor Initialized"); // initialize the sensor, default mode is NDOF
  Qref = bno.getQuat();// start quaternion and store desired orientation

}

//===========================================================================
// Main Loop Function
//===========================================================================
void loop() 
{
  if(Serial.available())
  {
    char val = Serial.read(); /* read the user input 
    x: stop motor 
    f: forward
    b: backward
    s: start attitude keeping control
      1: reset home configuration
      0: stop attitude keeping control 
    */
    switch(val)
    {
      case 'x':
      pulse = motorStop;
      motor1.writeMicroseconds(pulse);
      motor2.writeMicroseconds(pulse);
      motor3.writeMicroseconds(pulse);
      motor4.writeMicroseconds(pulse);
      Serial.print("Stop / Pulse : "), Serial.print(pulse), Serial.println();
      break;

      case 'f':
      motor1.writeMicroseconds(pulse+thrust);
      motor2.writeMicroseconds(pulse+thrust);
      motor3.writeMicroseconds(pulse+thrust);
      motor4.writeMicroseconds(pulse+thrust);
      pulse=pulse+thrust;
      Serial.print("Forward / Pulse : "); Serial.print(pulse), Serial.println();
      break;

      case 'b':
      motor1.writeMicroseconds(pulse-thrust);
      motor2.writeMicroseconds(pulse-thrust);
      motor3.writeMicroseconds(pulse-thrust);
      motor4.writeMicroseconds(pulse-thrust);
       pulse=pulse-thrust; 
      Serial.print("Backward / Pulse : "), Serial.print(pulse), Serial.println();
     
      break;

      case 's':
      s=1;
      Serial.println("Testing commence");
      break; 

      default:
      break;
      
    }


    while (s==1)
    {
//Calculate Error===========================================================
      imu::Quaternion Qm = bno.getQuat(); // find the quaternion representing current orientation
      qMeas1=Qm.w(); // store the current quaternion 
      qMeas2=Qm.x();
      qMeas3=Qm.y();
      qMeas4=Qm.z();
      filter(); // filter quaternion readings that has a length less than 0.99
      trigger(); // check if triggered to change home configuration (press: 1) or to stop attitude keeping (press: 0)
      qError(); // calculate relative orientation error
      qErrorHome(); // Convert relative error from inertial frame to home frame 
      euler(); // calculate euler angles from relative quaternion      
      qMeasPrev1=qMeas1; // store the measured quaternion for later use in filtering 
      qMeasPrev2=qMeas2;
      qMeasPrev3=qMeas3;
      qMeasPrev4=qMeas4;

//==========================================================================
//Implement PID Control
//==========================================================================

//ROLL (X) CONTROL==========================================================
//Calculate Proportional Control============================================
xPropOutput = eulX * xPropGain;

//Calculate Integral Control================================================
xIntegralOutput += xIntegralGain*eulX;

//Caculate Derivative Control===============================================
xDerivativeOutput = xDerivativeGain*(eulX-eulXPrev);
eulXPrev = eulX;
//Caculate total control output from PID
      if(eulX > 2 | eulX < -2) // Deadband 
      {
        xOutput = 1500 + xPropOutput;
      }

      if(eulY > 2 | eulY < -2) // Deadband 
      {
        yOutput = eulY * yPropGain + 1500;
      }

      if(eulZ > 2 | eulZ < -2) // Deadband 
      {
        zOutput = eulZ * zPropGain + 1500;
      }

//Feed the corrected pulse to the motor
      motor1.writeMicroseconds(xOutput);
      motor2.writeMicroseconds(pulse);
      motor3.writeMicroseconds(pulse);
      motor4.writeMicroseconds(pulse);
        
      printData(); // print data to serial monitor
    }

  
  }

}
