//===========================================================================
//This method prints some of the variables to the serial monitor for debugging
//===========================================================================
void printData(){
// Serial.print("Home: (");
// Serial.print(qRef1);Serial.print(" "); Serial.print(qRef2);Serial.print(" ");Serial.print(qRef3);Serial.print(" ");Serial.print(qRef4);
// Serial.print(")"); 
 
// Serial.print("    ");Serial.print("Measured:(");
// Serial.print(qMeas1);Serial.print(" "); Serial.print(qMeas2);Serial.print(" ");Serial.print(qMeas3);Serial.print(" ");Serial.print(qMeas4);  
// Serial.print(")"); 
 
// Serial.print("    ");Serial.print("Inertial Error:(");
// Serial.print(qErr1);Serial.print(" "); Serial.print(qErr2);Serial.print(" ");Serial.print(qErr3);Serial.print(" ");Serial.print(qErr4); 
// Serial.print(")");

// Serial.print("    ");Serial.print("Home Error:(");
 Serial.print(qErrHome1);Serial.print(" "); Serial.print(qErrHome2);Serial.print(" ");Serial.print(qErrHome3);Serial.print(" ");Serial.print(qErrHome4); 
// Serial.print(")"); 
Serial.print("    ");
 
// Serial.print("    ");Serial.print("Euler Error:(");
 Serial.print(eulX);Serial.print(" "); Serial.print(eulY);Serial.print(" ");Serial.print(eulZ);
// Serial.print(" )");

  Serial.print("    ");Serial.print(xPropOutput); Serial.print("    ");Serial.print(xDerivativeOutput); Serial.print("    ");Serial.print(xOutput);
//  Serial.print("    ");Serial.print(xOutput); Serial.print("    ");Serial.print(yOutput); Serial.print("    ");Serial.print(zOutput);
  Serial.println();
}

//==========================================================================
//This method caculates the relative error between the desired and actual (qErr=qRef x qMeasCon)
//==========================================================================
void qError(){
  qMeasCon1 = qMeas1;
  qMeasCon2 = -1*qMeas2;
  qMeasCon3 = -1*qMeas3;
  qMeasCon4 = -1*qMeas4;
  
  qErr1=(qRef1*qMeasCon1-qRef2*qMeasCon2-qRef3*qMeasCon3-qRef4*qMeasCon4);
  qErr2=(qRef1*qMeasCon2+qRef2*qMeasCon1+qRef3*qMeasCon4-qRef4*qMeasCon3);
  qErr3=(qRef1*qMeasCon3-qRef2*qMeasCon4+qRef3*qMeasCon1+qRef4*qMeasCon2);
  qErr4=(qRef1*qMeasCon4+qRef2*qMeasCon3-qRef3*qMeasCon2+qRef4*qMeasCon1);
    
}

//==========================================================================
//This method caculates the relative error with respect to home configuration (qErrHome = qRefCon x qErr x qRef)
//==========================================================================
void qErrorHome(){
  qRefCon1 = qRef1;
  qRefCon2 = -1*qRef2;
  qRefCon3 = -1*qRef3;
  qRefCon4 = -1*qRef4;
  
  temp1=(qErr1*qRef1-qErr2*qRef2-qErr3*qRef3-qErr4*qRef4);
  temp2=(qErr1*qRef2+qErr2*qRef1+qErr3*qRef4-qErr4*qRef3);
  temp3=(qErr1*qRef3-qErr2*qRef4+qErr3*qRef1+qErr4*qRef2);
  temp4=(qErr1*qRef4+qErr2*qRef3-qErr3*qRef2+qErr4*qRef1);

  qErrHome1=(qRefCon1*temp1-qRefCon2*temp2-qRefCon3*temp3-qRefCon4*temp4);
  qErrHome2=(qRefCon1*temp2+qRefCon2*temp1+qRefCon3*temp4-qRefCon4*temp3);
  qErrHome3=(qRefCon1*temp3-qRefCon2*temp4+qRefCon3*temp1+qRefCon4*temp2);
  qErrHome4=(qRefCon1*temp4+qRefCon2*temp3-qRefCon3*temp2+qRefCon4*temp1);
  
}


//==========================================================================
//This filters measured unit quaternions if the length is less than 0.99
//==========================================================================

void filter(){
  qLength=sqrt(sq(qMeas1)+sq(qMeas2)+sq(qMeas3)+sq(qMeas4));
  if (qLength < 0.99){
    qMeas1=qMeasPrev1;
    qMeas2=qMeasPrev2;
    qMeas3=qMeasPrev3;
    qMeas4=qMeasPrev4;
    
  }
}

//==========================================================================
//This receives a trigger from the user via serial monitor to change the desired orientation (enter 1 into the serial monitor to trigger) 
//==========================================================================
void trigger() 
{
  if (Serial.available() > 0)
  {
    rx_byte = Serial.read();
    switch(rx_byte)
    {
      case '1':
        qRef1 = qMeas1;
        qRef2 = qMeas2;
        qRef3 = qMeas3;
        qRef4 = qMeas4;
        break;
      case '0':
        s=0;  
        break; 
    }
  }
}



//==========================================================================
//This converts the relative quaternion to euler angles
//==========================================================================
void euler(){
eulZ=atan2(2.0*(qErrHome2*qErrHome3+qErrHome4*qErrHome1),(sq(qErrHome2)-sq(qErrHome3)-sq(qErrHome4)+sq(qErrHome1)));
eulY=asin(-2.0*(qErrHome2*qErrHome4-qErrHome1*qErrHome3)/(sq(qErrHome2)+sq(qErrHome3)+sq(qErrHome4)+sq(qErrHome1)));
eulX=atan2(2.0*(qErrHome3*qErrHome4+qErrHome1*qErrHome2),(-sq(qErrHome2)-sq(qErrHome3)+sq(qErrHome4)+sq(qErrHome1)));
eulZ=eulZ*180/3.1415;
eulY=eulY*180/3.1415;
eulX=eulX*180/3.1415;
}
