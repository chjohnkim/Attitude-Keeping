#include <Wire.h> // include wire library so we can communicate with gyro
#include <Adafruit_BNO055.h>

//==========================================================================
// Data Variables
//==========================================================================
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

//==========================================================================
// Helper Variables
//==========================================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28); // create sensor object
char rx_byte = 0; // trigger variable from serial monitor to change desired orientation

//==========================================================================
// Setup Routine
//==========================================================================
void setup() {
  Serial.begin(9600); // start the serial monitor for logging
  if(bno.begin()) Serial.println("Sensor Initialized"); // initialize the sensor, default mode is NDOF
  Qref = bno.getQuat();// start quaternion and store desired orientation
}

//==========================================================================
// Main Program Loop
//==========================================================================
void loop() {
  imu::Quaternion Qm = bno.getQuat(); // find the quaternion representing current orientation
  qMeas1=Qm.w(); // store the current quaternion 
  qMeas2=Qm.x();
  qMeas3=Qm.y();
  qMeas4=Qm.z();
  filter(); // filter quaternion readings that has a length less than 0.99
  trigger(); // check if triggered to change desired orientation
  qError(); // calculate relative orientation error
  qErrorHome(); // Convert relative error from inertial frame to home frame 
  euler(); // calculate euler angles from relative quaternion 
  printData(); // print data to serial monitor
  qMeasPrev1=qMeas1; // store the measured quaternion for later use in filtering 
  qMeasPrev2=qMeas2;
  qMeasPrev3=qMeas3;
  qMeasPrev4=qMeas4;
//delay(200);  
}
