/*
 * M90393.ccp - Library for simplified MLX90393 SPI communication.
 * Created by Michael van den Bos, January 19th, 2019
 * 
 * Copyright (c) 2019 by Michael van den Bos <m.f.vandenbos@student.tudelft.nl>
 *
 * MLX90393: a library to provide functions to interface via SPI with the 
 * MLX90393 magnetic sensor. It is based on high level energia functions. 
 * It is possible to use this library in  Energia (the Arduino port for MSP microcontrollers) 
 * It is specifically written for msp-exp432p401r (RED)
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
 *
 */


//#include "Energia.h"
#include "M90393.h"


// constructor assigns pins according to the number of sensors
// constructor contains gain lookup table
// constructor cannot add pullups to inputs, 
// constructor cannot set output pins for CS, SCLK, MOSI, 
// call void M90393::mcinit(){ to initialize 
M90393::M90393(int sensors){
	num_s = sensors;

	_MISO[0] = MISO0;
	_MISO[1] = MISO1;
	_MISO[2] = MISO2;
	_MISO[3] = MISO3;
	_MISO[4] = MISO4;
	_MISO[5] = MISO5;
	_MISO[6] = MISO6;
	_MISO[7] = MISO7;
	_MISO[8] = MISO8;
	_MISO[9] = MISO9;
	_MISO[10] = MISO10;
	_MISO[11] = MISO11;
	_MISO[12] = MISO12;
	_MISO[13] = MISO13;
	_MISO[14] = MISO14;
	_MISO[15] = MISO15;
	_MISO[16] = MISO16;

	gainlookup[0][0]=0.751;gainlookup[0][1]=1.210;
	gainlookup[1][0]=0.601;gainlookup[1][1]=0.968;
	gainlookup[2][0]=0.451;gainlookup[2][1]=0.726;
	gainlookup[3][0]=0.376;gainlookup[3][1]=0.605;
	gainlookup[4][0]=0.3004;gainlookup[4][1]=0.484;
	gainlookup[5][0]=0.2504;gainlookup[5][1]=0.403;
	gainlookup[6][0]=0.2004;gainlookup[6][1]=0.323;
	gainlookup[7][0]=0.1502;gainlookup[7][1]=0.242;
	// can only initialize the values like this when assigning the array. Otherwise it has to be done item by item. 
	// end of constructor
};

void M90393::mcinit(){
  // pins are automatically set to input without pull up. Make sure to connect a pull up to each input pin. 
  // this can not be put in the constructor
  // use a for loop to initialize each pin as an output for Master Out Slave In:
  // MOSI pins are connected to a pull up. 
 // for (sensorSelect = 0; sensorSelect < num_s+1; sensorSelect++)  {
 //   pinMode(_MISO[sensorSelect], INPUT_PULLUP);
 // }
  MAP_GPIO_setOutputHighOnPin( CSP, CSB ); //CS
  MAP_GPIO_setOutputHighOnPin( SCLKP, SCLKB ); //SCLK
  MAP_GPIO_setOutputHighOnPin( MOSIP, MOSIB ); //SCLK
  GPIO_setAsOutputPin(CSP, CSB);
  GPIO_setAsOutputPin(SCLKP, SCLKB);
  GPIO_setAsOutputPin(MOSIP, MOSIB);

  // set knownRes to false: this forces to run readRes. 
  knownRes = false;
  knowntcomp = false; 
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++ Command functions +++++++++++++++++++++++++++++++++++++++++++++++++++++
//Mid level functions which are used to send commands to the sensor. Except for Read measurement and Read register, they will return
//the sensor status byte. These functions are called by higher level functions or by user.  
unsigned char M90393::cmdNOP(){
    unsigned char inputchar = 0x00;
    return sendCommand1(inputchar);
}
unsigned char M90393::cmdEX(){
    unsigned char inputchar = 0x80;
    return sendCommand1(inputchar);
}
unsigned char M90393::cmdSB(char zyxt){
	// this function has not been tested
    unsigned char inputchar = 0x10;
    // warning add zyxt
    inputchar |= zyxt;
    return sendCommand1(inputchar);
}
unsigned char M90393::cmdSWOC(char zyxt){
	// this function has not been tested
    unsigned char inputchar = 0x20;
    // warning add zyxt
    inputchar = inputchar|zyxt;
    return sendCommand1(inputchar);
}
unsigned char M90393::cmdSM(char zyxt){
	// this function will command the sensor to measure the axis specified in zyxt
    unsigned char inputchar = 0x30;
    // warning add zyxt
    inputchar = inputchar|zyxt;
    return sendCommand1(inputchar);
}
void M90393::cmdRM(char zyxt){
	// this function will command the sensor to read back the axis specified in zyxt
    unsigned char inputchar = 0x40;
    // warning add zyxt
    inputchar = inputchar|zyxt;
	if(zyxt == 0x0F){
		sendCommandlong(inputchar, 0x00, 0x00, 0x00, num_s, 1 ,8);}
	else{
		sendCommandlong(inputchar, 0x00, 0x00, 0x00, num_s, 1 ,8, zyxt);}
}
unsigned char M90393::cmdRT(){
	// this function will command the sensor to reset. 
	// This has to be done each time the sensor is started
	// It makes sure that the senor volatile memory is equal to the permanent memory
    unsigned char inputchar = 0xF0;
    return sendCommand1(inputchar);
}
unsigned char M90393::cmdWR(unsigned char c2, unsigned char c3, unsigned char c4){
	// this function will command the sensor take the data in it's volatile memory and store it in its permanent memory
	// Use sparingly and with caution. The sensor can perform this operation a limited amount of times
	// Upon next reset the new permanent memory will be read to the volatile memory. This function can therefore alter the reset state
	unsigned char inputchar = 0x60;
	sendCommandlong(inputchar, c2, c3, (c4<<2), num_s, 4, 0);
	return statusmatrix[0];
	// maybe return a bitwise sum instead of just the first one. This allows for finding errors.  	
}
unsigned short M90393::cmdRR(unsigned char c2){
	// this function will command the sensor to read back the specified memory location
	resetdatamatrix();
	unsigned char inputchar = 0x50;
	unsigned char c3 = 0x00;
	unsigned char c4 = 0x00;
	sendCommandlong(inputchar, (c2<<2), c3, c4, num_s, 2, 2);
	return datamatrix[0][0];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++ Calibration functions +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // temperature
	// there is a temperature value in the memory of the sensor
void M90393::temp_CAL(){
	TREF = cmdRR(0x24); 
	for(int count = 0; count < num_s; count++){ 
		tREF_ACT[count] = datamatrix[count][0];
	}
}
		//adjust Sens_TC 
		
// zero calibration
	// set calibration to zero. store the offset either in the sensor or in the processor
void M90393::zero_CAL(){
	// this thing is way to complex. I will reduce the number of things this has to do. 
	unsigned char zyxt = 0x0F;
	unsigned char outp; 
	// next part creates baseline
	for(int count = 0; count < 5; count++){ // do 5 measurements
		cmdSM(zyxt);
		   // introduce a small delay to make sure we avoid bus contention
		    uint32_t d = conversT(zyxt) MAP_CS_getMCLK() * 3 / baudrate : MAP_CS_getMCLK() * 4 / baudrate;

		    for(uint32_t k = 0; k < d;  k++)
		    {
		        __asm("  nop");
		    }
		cmdRM(zyxt); 
		filter(zyxt);
	} 
	// the folowing ensures that the offset_matrix is reset to a reasonable value
	for(int count = 0; count < num_s; count++){ 
		for(int count2 = 1; count2 < 4; count2++){ 
			offset_matrix[count][count2-1] = resultmatrixfilter[count][count2];
		}
	}	
	// idea is to stop when convergence is good enough. However it usually is so noisy it does not reach convergence
	int _cycles = 0; float convergence2 = 0; 	float convergence = 1000;
	while (convergence > 100){
		cmdSM(zyxt);
		delay(conversT(zyxt));
		cmdRM(zyxt); 
		filter(zyxt);
		for(int count = 0; count < num_s; count++){ 
			for(int count2 = 1; count2 < 4; count2++){ 
			convergence2 = convergence2 + fabs(offset_matrix[count][count2-1] - resultmatrixfilter[count][count2]);
			offset_matrix[count][count2-1] = resultmatrixfilter[count][count2];
			}
		}
		// make a list of convergence
		convergence_matrix[_cycles] = convergence2/(num_s*3); 
		_cycles = _cycles +1;
		if(_cycles == 25){_cycles = 0;} // matrix only has 25 slots
		// this makes sure the loop will end even if convergence is not reached. 
		convergence = convergence - 50;
		convergence2 = 0;
		for (int count = 0; count < _cycles; count++){
			convergence2 = convergence_matrix[count] + convergence2;
		}
		convergence2 = convergence2/_cycles;
		if(convergence2 < 10){
			convergence = convergence2;
		}
	}	
	int devi = 1;
	for(int count3 = 0; count3 < num_s; count3++){ 
			for(int count2 = 1; count2 < 4; count2++){ 
				movingaverage[count3][count2] = 0;}}
	for(int count = 0; count < 50; count++){ 
		cmdSM(zyxt);
		delay(conversT(zyxt));
		cmdRM(zyxt); 
		filter(zyxt);
		
		for(int count3 = 0; count3 < num_s; count3++){ 
			for(int count2 = 1; count2 < 4; count2++){ 
				movingaverage[count3][count2] = movingaverage[count3][count2]+resultmatrixfilter[count3][count2];
				
			}
		}			
		//  
		Serial.print("z1 = "); Serial.print(retrieve(0,4)/100);  Serial.print(" ");
		Serial.print("zm = "); Serial.print(movingaverage[0][3]/devi/100);  Serial.print(" ");	
		Serial.print("zf = "); Serial.println(retrieve_filter(0,4)/100);  
		devi = devi + 1;
	}
	// moving average is then the offset
	for(int count3 = 0; count3 < num_s; count3++){ 
		for(int count2 = 0; count2 < 3; count2++){ 
			offset_matrix[count3][count2] = movingaverage[count3][count2+1]/devi; 
				
		}
	}		
}
	
// kalman filter
// use the value of the processor
// use 100 ms digital filter
		
void M90393::filter(unsigned char zyxt){
	grab(zyxt); // takes raw data from datamatrix and converts it into floats in resultmatrix. 
		//time update
		// x_k is predicted to be x_k-1
		//Pk* = Pk-1 + Q
		// R = 250 nT
		//measurement update
		// Pk = (1-KkH)Pk* 
		// H = 1;, Kk = Pk*/(Pk*+R)
	for(int count = 0; count < num_s; count++){ 
		//time update
		filtermatrix[count][3] = filtermatrix[count][3] + filtermatrix[count][0];
		for(int count2 = 0; count2 < 3; count2++){ 
			pmatrix[count][count2] = pmatrix[count][count2] + filtermatrix[count][0];
		}
		// measurement update
		//Kk
		filtermatrix[count][2] = filtermatrix[count][3]/(filtermatrix[count][3]+filtermatrix[count][1]);
		for(int count2 = 0; count2 < 3; count2++){ 
			kmatrix[count][count2] = pmatrix[count][count2]/(pmatrix[count][count2] + filtermatrix[count][1]);
		}
		// T, x_k, y_k, z_k
		resultmatrixfilter[count][0] = resultmatrix[count][0]; // Temperature is not filtered copied for consistency
		for(int count2 = 1; count2 < 4; count2++){ 
			Kalmanupdate(count, count2);
		} 
	}
	// estimate dynamic process noise
	double mx = 0; double mf = 0; double mq = filtermatrix[0][1];
	for(int count = 0; count < num_s; count++){ 
		mx = pow(pow(resultmatrix[count][0], 2)+ pow(resultmatrix[count][1], 2)+ pow(resultmatrix[count][2], 2),0.5);
		mf = pow(pow(resultmatrixfilter[count][0], 2)+ pow(resultmatrixfilter[count][0], 2)+ pow(resultmatrixfilter[count][0], 2),0.5);
		mq = fabs(mf-mx); // absolute floating point difference
		filtermatrix[count][0] = qDet(filtermatrix[count][0], mq, filtermatrix[count][1]);
	}
}
		
void M90393::Kalmanupdate(int sensor, int axis){
	resultmatrixfilter[sensor][axis] = resultmatrixfilter[sensor][axis]+kmatrix[sensor][(axis-1)]*(resultmatrix[sensor][axis]-resultmatrixfilter[sensor][axis]);
	pmatrix[sensor][axis-1] = (1-kmatrix[sensor][axis])*pmatrix[sensor][axis-1]; 
}
		
double M90393::qDet(double qold, double mq, double R_r){
	//The measurement should be made of constant magnetic fields. Therefore the process noise should be low. 
	//However that would mean the filter becomes unresponsive. In order to make sure there is a response to a current change, 
	//process noise will be increased if the measurement value is too far from the filter value. 
	double qmin = 0.05*R_r; // results in 0.2 of sensor value response, 0.8 of average. 
	double qnew = 0.5*R_r; // results in 0.5 sensor value response, 0.5 of average. 
	double qmax = 4*R_r;  // results in in 0.8 of sensor value response, 0.2 of average. 
	if(mq > 8*R_r ){
		if(qold < 5*qmin){
			qnew = 4*qold; // rapid increase
		}else{
		qnew = qmax;	// if increasing the qnew does not work, increase to max. 
		}
	}
	else{qnew = qold/2; // if mq is low, steer toward. 0.25 of sensor value. 
		}
	if(qnew < qmin){
		qnew = qmin; 
	}		
	return qnew;
}
			
		
void M90393::filter_init(float P0, float X_0, float Y_0, float Z_0, float R){
//  float filtermatrix[16][4]; // stores Q, R, K, and P for the filter. 
	for(int count = 0; count < num_s; count++){
		filtermatrix[count][0] = qDet(4*R, 4*R, R); // initializes Q
		filtermatrix[count][1] = R; 	// Stores R
		filtermatrix[count][2] = 1; 	// K begins at 1. 
		filtermatrix[count][3] = P0;	// 
		
		resultmatrixfilter[count][0] = 25; //temperature
		resultmatrixfilter[count][0] = X_0;
		resultmatrixfilter[count][0] = Y_0;
		resultmatrixfilter[count][0] = Z_0;
		for(int count2 = 0; count2 < 3; count2++){ 
			kmatrix[count][count2] = filtermatrix[count][2];	 
			pmatrix[count][count2] = filtermatrix[count][3]; 	// determines the ahead error covariance. should be square of noise measured
			// exact value of P_o makes little difference over time, but it determines how fast the filter corrects to the right value in the beginning.  
		}						
	}
}		
		
//T conversion :  calculate conversion time based on the formula in the table and the safety factor. 
unsigned char M90393::conversT(unsigned char zyxt){
	int digf = Dig_filt_actual;
	int osrf = OSR_actual;
	float TconvT = (2.f + pow(2.f, digf))*pow(2.f, osrf)*64.f/1000.f;
	unsigned char multipli = 0;
	if ((zyxt & 0x08) > 0){
	multipli = multipli + 1;}
	if ((zyxt & 0x04) > 0){
	multipli = multipli + 1;}
	if ((zyxt & 0x02) > 0){
	multipli = multipli + 1;}
	unsigned char conversionTime = 2;	
	conversionTime = conversionTime + multipli * ((int)TconvT+2); 
	return conversionTime;
}
		


// slope check 






//+++++++++++++++++++++++++++++++++++++++++++++++++++++ Utility functions +++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char M90393::sendCommand1(unsigned char c){
  digitalWrite(SCLK, HIGH);
  digitalWrite(CS, LOW); 

  delay(DELAY_TIME*4);

  writeMLX90393(c);

  delay(DELAY_TIME);

  unsigned char tmp = readMLX90393();

  delay(DELAY_TIME);

  digitalWrite(CS, HIGH);
  digitalWrite(MOSI0, HIGH);

  return tmp;
  
}
void M90393::sendCommandlong(unsigned char c, unsigned char c2, unsigned char c3, unsigned char c4, int ArraySize, int writebytes, int returnbytes){
  //zero datamatrix
  resetdatamatrix();
  
  // send command bytes
  writeseqMLX90393(c, c2, c3, c4, writebytes); //writeseq sets CS low. 
  // read return status byte CS remains low
  readMLX90393_multiC(ArraySize); // status byte is first byte of return
  transportMLX90393C(ArraySize); // status byte is transported to status matrix
  //Serial.println("hi");
  if (returnbytes > 0 ) {
	//read return data bytes
	for(int count8 = 0; count8 < (returnbytes/2); count8++){
	readMLX90393_multiS(ArraySize);
	transportMLX90393S(ArraySize, count8); // transfers bytes from short output to datamatrix
	delay(DELAY_TIME);
	}
  }
   digitalWrite(CS, HIGH); //command sequence finished. Reset by CS HIGH
   digitalWrite(MOSI0, HIGH);//command sequence finished. Reset by MOSI HIGH
}
void M90393::sendCommandlong(unsigned char c, unsigned char c2, unsigned char c3, unsigned char c4, int ArraySize, int writebytes, int returnbytes, unsigned char zyxt){
  // send command bytes
  // this overload is used when zxyt is not 0x0F and retrieving a measurement (cmd RM)
  writeseqMLX90393(c, c2, c3, c4, writebytes); //writeseq sets CS low. 
  // read return status byte CS remains low
  readMLX90393_multiC(ArraySize); // status byte is first byte of return
  transportMLX90393C(ArraySize); // status byte is transported to status matrix
  bool _ex = true;
	if ((zyxt & 0x01) == 0x01) {
		//read return data bytes
		readMLX90393_multiS(ArraySize);
		transportMLX90393S(ArraySize, 0); // transfers bytes from short output to datamatrix for temperature
		delay(DELAY_TIME);
	}else{if(_ex){resetdatamatrix(zyxt, ArraySize); _ex = false;}}
	if ((zyxt & 0x02) == 0x02) {
		//read return data bytes
		readMLX90393_multiS(ArraySize);
		transportMLX90393S(ArraySize, 1); // transfers bytes from short output to datamatrix for x-axis
		delay(DELAY_TIME);
	}else{if(_ex){resetdatamatrix(zyxt, ArraySize); _ex = false;}}
	if ((zyxt & 0x04) == 0x04) {
		//read return data bytes
		readMLX90393_multiS(ArraySize);
		transportMLX90393S(ArraySize, 2); // transfers bytes from short output to datamatrix for y-axis
		delay(DELAY_TIME);
	}else{if(_ex){resetdatamatrix(zyxt, ArraySize); _ex = false;}}
	if ((zyxt & 0x08) == 0x08) {
		//read return data bytes
		readMLX90393_multiS(ArraySize);
		//int _time = micros();
		transportMLX90393S(ArraySize, 3); // transfers bytes from short output to datamatrix for z-axis
		//_time = micros()-_time; 
		//Serial.print("time = "); Serial.println(_time);
		delay(DELAY_TIME);
	}else{if(_ex){resetdatamatrix(zyxt, ArraySize); _ex = false;}}
  
   digitalWrite(CS, HIGH); //command sequence finished. Reset by CS HIGH
   digitalWrite(MOSI0, HIGH);//command sequence finished. Reset by MOSI HIGH
}


void M90393::readMLX90393_multiC(int ArraySize){
// this function reads one byte	
// charoutputmatrix is set to 0
// for loop takes 37 microseconds for 16 pins. which is not a problem as long as we use delay(). once using delaymicros() it becomes a problem. 
for(int count2 = 0; count2 < ArraySize; count2++){
		charoutputmatrix[count2] =0x00; // set variable to zero
      }

	  // first loop goes through 8 bits
 for(int count = 1; count < 9; count++){
	  // clock goes high
      digitalWrite(SCLK, HIGH);
      delay(DELAY_TIME); // wait
	  // second loop goes through all relevant read pins.
	  // for loop takes 37 microseconds for 16 pins. 
      for(int count2 = 0; count2 < ArraySize; count2++){
        charoutputmatrix[count2] |= (digitalRead(_MISO[count2]) & 0x01 ) << (8 - count);
      }
      delay(DELAY_TIME);
      digitalWrite(SCLK, LOW); // clock goes low. (here the values are thought of as read)
      delay(DELAY_TIME);   
  }
  digitalWrite(SCLK, HIGH); // clock goes high at the end
}
void M90393::readMLX90393_multiS(int ArraySize){
// this function reads two bytes	
// first loop goes through 16 bits
// for loop takes 37 microseconds for 16 pins. which is not a problem as long as we use delay(). once using delaymicros() it becomes a problem.
// this has to be here because the entire array can only be edited in a for loop. 
// this has to be here because if it isn't it is possible that a 1 of an old measurement remains, where there should be a 0. 
 for(int count2 = 0; count2 < ArraySize; count2++){
		shortoutputmatrix[count2] = 0x00; // set variable to zero
      }
 for(int count = 1; count < 17; count++){
	  // clock goes high
      digitalWrite(SCLK, HIGH);
      delay(DELAY_TIME); // wait
	  // second loop goes through all relevant read pins.
	  // for loop takes 37 microseconds for 16 pins. 
      for(int count2 = 0; count2 < ArraySize; count2++){
		//Serial.print("this is readMLX90393_multiS, I am at count2 ");
		//Serial.println(count2);
		//Serial.print("this is readMLX90393_multiS, I am at count ");
		//Serial.println(count);
		//Serial.print("this is readMLX90393_multiS, the retun is ");
		//Serial.println(digitalRead(_MISO[count2]) & 0x01);
        shortoutputmatrix[count2] |= (digitalRead(_MISO[count2]) & 0x01 ) << (16 - count);
		//Serial.print(digitalRead(_MISO[count2]));
		//Serial.print(" ");
      }
	 // Serial.println();
      delay(DELAY_TIME); //!
      digitalWrite(SCLK, LOW); // clock goes low. (here the values are changed by slave)
      delay(DELAY_TIME);   
  }
  digitalWrite(SCLK, HIGH);
}
void M90393::transportMLX90393C(int ArraySize){
  
  for(int count = 0; count < ArraySize; count++){
    
    statusmatrix[count] = charoutputmatrix[count];
  }
}
void M90393::transportMLX90393S(int ArraySize, int partsel){
  
  for(int count = 0; count < ArraySize; count++){
    //Serial.print("this is transportMLX90393S I am transporting ");
	//Serial.println(shortoutputmatrix[count], BIN);
    datamatrix[count][partsel] = shortoutputmatrix[count];
  }
}
unsigned char M90393::readMLX90393(){

  unsigned char tmp = 0;

  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  tmp |= (digitalRead(MISO0) & 0x01 ) <<7;
  delay(DELAY_TIME);
  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME);

  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  tmp |= (digitalRead(MISO0) & 0x01 ) <<6;
  delay(DELAY_TIME);
  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME);

  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  tmp |= (digitalRead(MISO0) & 0x01) << 5;
  delay(DELAY_TIME);
  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME);

  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  tmp |= (digitalRead(MISO0) & 0x01) << 4;
  delay(DELAY_TIME);
  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME);

  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  tmp |= (digitalRead(MISO0) & 0x01) << 3;
  delay(DELAY_TIME);
  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME);

  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  tmp |= (digitalRead(MISO0) & 0x01) << 2;
  delay(DELAY_TIME);
  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME);

  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  tmp |= (digitalRead(MISO0) & 0x01) << 1;
  delay(DELAY_TIME);
  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME);

  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  tmp |= (digitalRead(MISO0) & 0x01);
  delay(DELAY_TIME);
  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME);

  digitalWrite(SCLK, HIGH);

  return tmp;
  
  
}	

void M90393::writeMLX90393(unsigned char c){
 
		
  digitalWrite(SCLK, LOW);
  delayMicroseconds(DELAY_TIME*40);
  digitalWrite(MOSI0, (c & 0x80) ? HIGH : LOW);
  delay(DELAY_TIME);
  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);
  
  digitalWrite(SCLK, LOW);
  delayMicroseconds(DELAY_TIME*40);
  digitalWrite(MOSI0, (c & 0x40) ? HIGH : LOW);
  delay(DELAY_TIME);
  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);

  digitalWrite(SCLK, LOW);
  delayMicroseconds(DELAY_TIME*40);
  digitalWrite(MOSI0, (c & 0x20) ? HIGH : LOW);
  delay(DELAY_TIME);
  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);

  digitalWrite(SCLK, LOW);
  delayMicroseconds(DELAY_TIME*40);
  digitalWrite(MOSI0, (c & 0x10) ? HIGH : LOW); 
  delay(DELAY_TIME);
  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);

  digitalWrite(SCLK, LOW);
  delayMicroseconds(DELAY_TIME*40);
  digitalWrite(MOSI0, (c & 0x08) ? HIGH : LOW);
  delay(DELAY_TIME);
  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);

  digitalWrite(SCLK, LOW);
  delayMicroseconds(DELAY_TIME*40);
  digitalWrite(MOSI0, (c & 0x04) ? HIGH : LOW);
  delay(DELAY_TIME);
  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);

  digitalWrite(SCLK, LOW);
  delayMicroseconds(DELAY_TIME*40);
  digitalWrite(MOSI0, (c & 0x02) ? HIGH : LOW);
  delay(DELAY_TIME);
  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);

  digitalWrite(SCLK, LOW);
  delayMicroseconds(DELAY_TIME*40);
  digitalWrite(MOSI0, (c & 0x01) ? HIGH : LOW);
  delay(DELAY_TIME);
  digitalWrite(SCLK, HIGH);
  delay(DELAY_TIME);

  digitalWrite(SCLK, LOW);
  delay(DELAY_TIME*4);
  
}

void M90393::writeseqMLX90393(unsigned char c, unsigned char c2, unsigned char c3, unsigned char c4, int writebytes){
  digitalWrite(SCLK, HIGH); 
  digitalWrite(CS, LOW); 
  delay(DELAY_TIME*4); 
  writeMLX90393(c);
  delay(DELAY_TIME*3);
    if ( writebytes == 1 ) {}
	else {
        writeMLX90393(c2);
		delay(DELAY_TIME*3);
		if ( writebytes == 3 ){
		  writeMLX90393(c3);
          delay(DELAY_TIME*3);
		  }
        if (writebytes == 4){
			writeMLX90393(c3);
			delay(DELAY_TIME*3);
            writeMLX90393(c4);
            delay(DELAY_TIME*4);
          }     
    }
}

unsigned char M90393::setFilter(unsigned char DIG_FILT){
	unsigned short readout; 
	readout = cmdRR(0x02);
	unsigned short mask = DIG_FILT << 2;
	mask |= readout; // this makes sure that none of the other values are changed. 
	unsigned short mask2 = 0xFFE3; // 0xFFE3 = 1111 1111 1110 0011 // leaving the digital filter bits low
	mask2 |= DIG_FILT << 2; // This to set the bits that are high in the input high in mask2. 
	mask &= mask2; // This bitwise and makes sure that bits that should be zero are zero in the mask
	// Otherwise, the digital filter cannot be set to zero. 
	unsigned char c2 = 0x00;  
	unsigned char c3 = 0x00;
	c2 |= mask >>8;
	c3 |= mask; // This breaks up the mask in to chars that can be input in the cmdWR function. 
	unsigned char wrstatus =0x00;
	wrstatus = cmdWR(c2, c3, 0x02); //Write the mask onto volatile memory. That means it will be cleared during a reset. 
	
	Dig_filt_actual = DIG_FILT; // stores value 
	return wrstatus;
	

}

unsigned char M90393::setOSR(unsigned char OSR, unsigned char OSR2){
	
	// OSR sets magnetic oversampling
	// OSR2 sets thermal oversampling
	OSR_actual = OSR;
	unsigned short readout; 
	readout = cmdRR(0x02);
	unsigned short mask = OSR;
	mask |= OSR2 << 11;
	mask |= readout; // this makes sure that none of the other values are changed.
	
	unsigned short mask2 = 0xE7FC; // 0xE7FC = 1110 0111 1111 1100 // leaving the OSR bits low
	mask2 |= OSR2 << 11; // This to set the bits that are high in the input high in mask2. 
	mask2 |= OSR; // This to set the bits that are high in the input high in mask2. 
	mask &= mask2; // This bitwise and makes sure that bits that should be zero are zero in the mask
	// Otherwise, the digital filter cannot be set to zero. 
	
	
	unsigned char c2 = 0x00;  
	unsigned char c3 = 0x00;
	c2 |= mask >>8;
	c3 |= mask;// This breaks up the mask in to chars that can be input in the cmdWR function. 
	unsigned char wrstatus =0x00;
	wrstatus = cmdWR(c2, c3, 0x02); //Write the mask onto volatile memory. That means it will be cleared during a reset. 
	return wrstatus;
}

unsigned char M90393::setADC(unsigned char OSR, unsigned char OSR2, unsigned char DIG_FILT, unsigned char RES_X, unsigned char RES_Y, unsigned char RES_Z){
	knownRes = false;
  knowntcomp = false; 
  
  
  
  
  
  
  if (knownRes){}
	else{readRes(); 
		knownRes = true;}
	if (knowntcomp){}
	else{readTcomp();
	knowntcomp = true;}
  
	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++ retrieval functions +++++++++++++++++++++++++++++++++++++++++++++++++++++
float M90393::getSenseZ(){
	float tmp;
	int gain = 0;
	int res = 0;
	unsigned short value_gain;
	unsigned short value_res;
	unsigned short value_hallconf;
	unsigned short readout;
	readout = cmdRR(0x00);
	value_gain = (readout & 0x70) >> 4;
	value_hallconf = (readout & 0xF);
	readout = cmdRR(0x02);
	value_res = (readout & 0x60) >> 5;
	res = value_res;
	gain = value_gain;
	tmp = gainlookup[gain][1];
	if (res==0){
	return tmp;
	}
	else{
		tmp = tmp*(float)(1 << res);
		return tmp; 
	}
}
float M90393::getGainZ(int res, int gain){
	float tmp;
	tmp = gainlookup[gain][1];
	if (res==0){
	return tmp;
	}
	else{
		tmp = tmp*(float)(1 << res);
		return tmp; 
	}
}
float M90393::getGainXY(int res, int gain){
	float tmp;
	tmp = gainlookup[gain][0];
	if (res==0){
	return tmp;
	}
	else{
		tmp = tmp*(float)(1 << res);
		return tmp; 
	}
}
float M90393::getSenseX(){
	float tmp;
	int gain = 0;
	int res = 0;
	unsigned short value_gain;
	unsigned short value_res;
	unsigned short value_hallconf;
	unsigned short readout;
	readout = cmdRR(0x00);
	value_gain = (readout & 0x70) >> 4;
	value_hallconf = (readout & 0xF);
	readout = cmdRR(0x02);
	value_res = (readout & 0x600) >> 9;
	res = value_res;
	Serial.println();
	Serial.print("SenseX says res = " ); Serial.println(res);
	gain = value_gain;
	Serial.print("SenseX says gain = " ); Serial.println(gain);
	tmp = gainlookup[gain][0];
	if (res==0){
	return tmp;
	}
	else{
		tmp = tmp*(float)(1 << res);
		return tmp; 
	}
}
float M90393::getSenseY(){
	float tmp;
	int gain = 0;
	int res = 0;
	unsigned short value_gain;
	unsigned short value_res;
	unsigned short value_hallconf;
	unsigned short readout;
	readout = cmdRR(0x00);
	value_gain = (readout & 0x70) >> 4;
	value_hallconf = (readout & 0xF);
	readout = cmdRR(0x02);
	value_res = (readout & 0x180) >> 7;
	res = value_res;
	gain = value_gain;
	tmp = gainlookup[gain][0];
	if (res==0){
	return tmp;
	}
	else{
		tmp = tmp*(float)(1 << res);
		return tmp; 
	}
}
void M90393::readRes(){
	//should be called after modfiying the res and gain 
	unsigned short value_res;
	unsigned short readout;
	readout = cmdRR(0x02);
	value_res = (readout & 0x600) >> 9;
	res_x = value_res;
	value_res = (readout & 0x180) >> 7;
	res_y = value_res;
	value_res = (readout & 0x60) >> 5;
	res_z = value_res;
	sense_x = getSenseX();
	sense_y = getSenseY();
	sense_z = getSenseZ();
}
void M90393::readTcomp(){
	unsigned short readout;
	readout = cmdRR(0x01);
	unsigned short value_tcomp;
	value_tcomp = (readout & 0x400) >> 10;
	if (value_tcomp == 0x01){
		tcomp = 1;}
	else{
		tcomp = 0;}
	
}

unsigned char M90393::getTcomp(){
	if (knowntcomp){}
	else{readTcomp();
		knowntcomp = true;}
	Serial.print("tcomp = "); Serial.println(tcomp);
	unsigned char tmp = 0;
	tmp = tcomp;
	return tmp;
}
void M90393::readHallConF(){
	unsigned short readout;
	readout = cmdRR(0x00);
	unsigned short value_hallconf;
	hallConF = (readout & 0x0F) ;
	
}


unsigned char M90393::getHallConF(){
	readHallConF();
	return hallConF;
}



int M90393::getResX(){
	//should be called after modfiying the res and gain 
	unsigned short value_res;
	unsigned short readout;
	readout = cmdRR(0x02);
	Serial.print("readout = "); Serial.println(readout, BIN);
	value_res = (readout & 0x600) >> 9;
	Serial.print("value_res = "); Serial.println(value_res, BIN);
	res_x = value_res;
	return res_x;
}


unsigned short M90393::boostRet(int sensor, unsigned short mode){
	switch(mode){
		case 0: {
			unsigned short tmp = 0;
			tmp = statusmatrix[sensor];
			return tmp; } break;
		case 1:
			return datamatrix[sensor][0];
			break;
		case 2: 
			return datamatrix[sensor][1];
			break;
		case 3:
			return datamatrix[sensor][2];
			break;
		case 4:
			return datamatrix[sensor][3];
			break;
		default: 
		{unsigned short tmp = 0x00;
		return tmp;}
			break;
	}
}
void M90393::grab(unsigned char zyxt){
	if ((zyxt & 0x08 )> 0){
	//datamatrix[sensor][3] = z-axis
		float sense_zn = 1000*sense_z;
		unsigned short tmpda = 0;
		for(int count = 0; count < (num_s+1); count++){
			tmpda = datamatrix[count][3];
			//if (res_z < 2){
					signed short tmpdx = tmpda;
					resultmatrix[count][3] = tmpdx*sense_zn;
					// this should be it (O_O)
			//}else{ //THIS IS WRONG. THE PROGRAM CANNOT HANDLE res > 1; 
				//if(res_z == 3){
					//tmpda = datamatrix[count][3]; //THIS IS WRONG. THE PROGRAM CANNOT HANDLE res > 1; 
			//}}
		}
	}
	if((zyxt & 0x04 )> 0){
	//datamatrix[sensor][2] = y-axis
		float sense_yn =1000*sense_y;
		unsigned short tmpda; 
		for(int count = 0; count < (num_s+1); count++){
			tmpda = datamatrix[count][2];
			if (res_y <2){ //THE PROGRAM CANNOT HANDLE res > 1; 
				signed short tmpdx = tmpda;
				resultmatrix[count][2] = tmpdx*sense_yn;
				// this should be it (O_O)
			}
		}
	}
	if((zyxt & 0x02 )> 0){
	//datamatrix[sensor][1] = x-axis
		float sense_xn =1000*sense_x;
		unsigned short tmpda; 
		for(int count = 0; count < (num_s+1); count++){
			tmpda = datamatrix[count][1];
			if (res_y <2){ //THE PROGRAM CANNOT HANDLE res > 1; 
				signed short tmpdx = tmpda;
				resultmatrix[count][1] = tmpdx*sense_xn;
				// this should be it (O_O)
			}
		}
	}
	if((zyxt & 0x01 )> 0){
	float temp = 1; 
		// datamatrix[sensor][0] = temperature
			unsigned short temp2 = 0;
			for(int count = 0; count < (num_s+1); count++){
			if (datamatrix[count][0] == 46244){temp = 25;}
			else if (datamatrix[count][0]>46244){
				temp2 = datamatrix[count][0]-46244;
				temp = 25 + temp2/45.2; 
			}// is this consistent with int math?
			else if (datamatrix[count][0]<46244){	
				temp2 = 46244-datamatrix[count][0];
				temp = 25 - temp2/45.2; 
			}
			resultmatrix[count][0]= temp;
		}
		
	}
}

float M90393::retrieve_raw(int sensor, unsigned short mode){
	//re
	return 0;


}

float M90393::retrieve_filter(int sensor, unsigned short mode){
	// mode is axis - 1 
	if(mode == 0 ){
			return 0;
	}
	else{
	return resultmatrixfilter[sensor][(mode-1)];
	}
}

float M90393::retrieve_Calib(int sensor, unsigned short mode){
	// mode is axis - 1 
	if(mode == 0 ){
			return 0;
	}
	else{
		float output_calib = resultmatrixfilter[sensor][(mode-1)]-offset_matrix[sensor][mode-2];
	return output_calib;
	}
}


float M90393::retrieve(int sensor, unsigned short mode){
	switch(mode){
		case 0: {
			unsigned short tmp = 0;
			tmp = statusmatrix[sensor];
			return tmp; } break;
		case 1: {
		float temp = 1; 
		// datamatrix[sensor][0] = temperature
			
			unsigned short temp2 = 0;
			if (datamatrix[sensor][0] == 46244){temp = 25;}
			else if (datamatrix[sensor][0]>46244){
				temp2 = datamatrix[sensor][0]-46244;
				temp = 25 + temp2/45.2; 
			}// is this consistent with int math?
			else if (datamatrix[sensor][0]<46244){	
				temp2 = 46244-datamatrix[sensor][0];
				temp = 25 - temp2/45.2; 
			}
			return temp;}
			break;
		case 2:{
			//datamatrix[sensor][1] = x-axis
			float temp = 0; 
			unsigned short tmpda = datamatrix[sensor][1];
			if (res_x <2){
				//Serial.println();
				//Serial.print("tmpda = "); Serial.println(tmpda, BIN);
					signed short tmpdx = tmpda;
					//Serial.print("tmpdx = "); Serial.println(tmpdx, BIN);
					float sense_xn =1000*sense_x;
					temp = tmpdx*sense_xn;
					// this should be it (O_O)
			}
			//Serial.println("I am at case 2, in retrieve. ");
			//Serial.print("tmpda = "); Serial.println(tmpda);
			//Serial
			// you should put an else here if res > 1
			return temp;}
		break;
		case 3: {
			//datamatrix[sensor][2] = y-axis
			float temp = 0; 
			unsigned short tmpda = datamatrix[sensor][2];
			if (res_y <2){
				//Serial.println();
				//Serial.print("tmpda = "); Serial.println(tmpda, BIN);
					signed short tmpdx = tmpda;
					//Serial.print("tmpdx = "); Serial.println(tmpdx, BIN);
					float sense_yn =1000*sense_y;
					temp = tmpdx*sense_yn;
					// this should be it (O_O)
			}
			return temp;}
			break;
		case 4: {
				//datamatrix[sensor][3] = z-axis
			float temp = 0; 
			unsigned short tmpda = datamatrix[sensor][3];
			if (res_z < 2){
				//Serial.println();
				//Serial.print("tmpda = "); Serial.println(tmpda, BIN);
					signed short tmpdx = tmpda;
					//Serial.print("tmpdx = "); Serial.println(tmpdx, BIN);
					float sense_zn = 1000*sense_z;
					//Serial.print("sense_zn = "); Serial.println(sense_zn);
					temp = tmpdx*sense_zn;
					//Serial.print("sense_z = ");Serial.println(sense_z);
					//Serial.print("temp = "); Serial.println(temp);
					// this should be it (O_O)
			}else{
				if(res_z == 3){
					tmpda = datamatrix[sensor][3];
			}}
			
			return temp;
		}
			break;
		default: {
			unsigned short tmp = 0x00;
		return tmp;}
			break;
	}
}
unsigned char M90393::retrieveS(int sensor){
	return statusmatrix[sensor];
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++ test functions +++++++++++++++++++++++++++++++++++++++++++++++++++++
void M90393::datamatrixrand(){
	int randa = rand(); // outside the loop since rand is slow. 
	for(int count = 0; count < 17; count++){  
      for(int count2 = 0; count2 < 5; count2++){
		  
		  unsigned short temprand;
			temprand = randa;
			temprand = (temprand + count + count2) >> 1;
		datamatrix[count][count2] = temprand; // set variable to temprand
      }
  }
 
}
int M90393::pintest()
{ 
	
    digitalWrite(MOSI0, LOW);   // turn output pin low
    digitalWrite(MOSI0, HIGH);   // turn the pin high
	
    delay(1000); // keep high for one second
	int tim = micros();
    digitalWrite(MOSI0, LOW); //turn the pin low. 
	tim = micros()-tim;
	return tim;
  }

int M90393::pintest2(){
	char tempchar;
	int tim = micros();
	int count = 1;
	for(int count2 = 0; count2 < 16; count2++){
        tempchar |= (digitalRead(_MISO[count2]) & 0x01 ) << (8 - count);
      }
	tim = micros()-tim;
	return tim; 
}

void M90393::resetdatamatrix(){
	for(int count = 0; count < 17; count++){  
      for(int count2 = 0; count2 < 5; count2++){
		datamatrix[count][count2] = 0x00; // set variable to zero
      }
  }
}
void M90393::resetdatamatrix(unsigned char zyxt, int ArraySize){
	 
		if ((zyxt & 0x01)>0){}
			else{
				for(int count = 0; count < ArraySize; count++){ 
				datamatrix[count][0] = 0x00; // set variable temp to zero	
				}
			}
		if ((zyxt & 0x02)>0){}
			else{
				for(int count = 0; count < ArraySize; count++){ 
				datamatrix[count][1] = 0x00; // set variable x to zero
				}
			}
		if ((zyxt & 0x04)>0){}
			else{
				for(int count = 0; count < ArraySize; count++){ 
				datamatrix[count][3] = 0x00; // set variable y to zero
				}
			}
		if ((zyxt & 0x04)>0){}
		else{
				for(int count = 0; count < ArraySize; count++){ 
				datamatrix[count][4] = 0x00; // set variable z to zero
				}
			}
}
