/*
  M90393.h - Library for simplified MLX90393 SPI communication.
  Created by Michael van den Bos, November 30, 2018.
  
*/
#ifndef M90393_h
#define M90393_h

#include <driverlib.h>

#include "msp.h"
//#include "Energia.h"
#define DELAY_TIME 1

#define MOSIP 4 		//P4.3
#define MOSIB 3


#define MISO0 7    	//P1.5
#define MISO1 8		//P4.6
#define MISO2 23	//P6.1
#define MISO3 24    //P4.0
#define MISO4 25    //P4.2
#define MISO5 26    //P4.4
#define MISO6 27    //P4.5
#define MISO7 28    //P4.7
#define MISO8 29    //P5.4
#define MISO9 30    //P5.5
#define MISO10 40   //P2.7
#define MISO11 39   //P2.6
#define MISO12 38   //P2.4
#define MISO13 37   //P5.6
#define MISO14 36   //P6.6
#define MISO15 35   //P6.7
#define MISO16 34   //P2.3


#define SCLKP 6		//P6.0
#define SCLKB 0
#define CSP 4		//4.1
#define CSB 1
//SCL(1)=9 and SDA(1)=10, SDA(0) = 15, SCL(0) = 14 
class M90393
{
  public:
    M90393(int sensors);
	void mcinit();
	// test functions
    int pintest();
    int pintest2();
	void datamatrixrand();
	void resetdatamatrix();
	void resetdatamatrix(unsigned char zyxt, int ArraySize);
	//command functions
    unsigned char cmdNOP();
    unsigned char cmdEX();
    unsigned char cmdSB(char zyxt);
    unsigned char cmdSWOC(char zyxt);
    unsigned char cmdSM(char zyxt);
	void cmdRM(char zyxt);
	unsigned char cmdRT();
	unsigned char cmdWR(unsigned char c2, unsigned char c3, unsigned char c4);
	unsigned short cmdRR(unsigned char c2);
	// utility functions
    void writeMLX90393(unsigned char c);
	unsigned char readMLX90393();
	unsigned char sendCommand1(unsigned char c);
	void readMLX90393_multiC(int ArraySize);
	void readMLX90393_multiS(int ArraySize);
	void transportMLX90393C(int ArraySize);
	void transportMLX90393S(int ArraySize, int partsel);
	void writeseqMLX90393(unsigned char c, unsigned char c2, unsigned char c3, unsigned char c4, int writebytes);
	void sendCommandlong(unsigned char c, unsigned char c2, unsigned char c3, unsigned char c4, int ArraySize, int writebytes, int returnbytes);
	void sendCommandlong(unsigned char c, unsigned char c2, unsigned char c3, unsigned char c4, int ArraySize, int writebytes, int returnbytes, unsigned char zyxt);
	unsigned char setFilter(unsigned char DIG_FILT);
	unsigned char setOSR(unsigned char OSR, unsigned char OSR2);
	unsigned char setADC(unsigned char OSR, unsigned char OSR2, unsigned char DIG_FILT, unsigned char RES_X, unsigned char RES_Y, unsigned char RES_Z);
	
	
	// retrieval functions
	float retrieve(int sensor, unsigned short mode);
	unsigned char retrieveS(int sensor);
	unsigned char getTcomp();
	unsigned char getHallConF();
	unsigned short boostRet(int sensor, unsigned short mode);
	float getGainZ(int res, int gain);
	float getGainXY(int res, int gain); 
	float getSenseZ();
	float getSenseX();
	float getSenseY();	
	void readRes();
	void readTcomp();
	void readHallConF();
	int getResX();
	float retrieve_Calib(int sensor, unsigned short mode);
	float retrieve_filter(int sensor, unsigned short mode);
	float retrieve_raw(int sensor, unsigned short mode);
	// filter functions
	void filter_init(float P0, float X_0, float Y_0, float Z_0, float R);
	void filter(unsigned char zyxt);  
	void Kalmanupdate(int count, int axis);
	double qDet(double qold, double mq, double R_r);
	void grab(unsigned char zyxt);
	
	//calibration functions
	void temp_CAL();
	void zero_CAL();
	unsigned char conversT(unsigned char zyxt);
	
	
	// example functions
	void dot();
    void dash();
    
	// variables 
    int num_s;
	unsigned short datamatrix[16][4];
	unsigned char statusmatrix[16];
	unsigned char charoutputmatrix[16];
	unsigned short shortoutputmatrix[16];
	
	
    int _selct;
    int sensorSelect;
	
	// filter variables
	float resultmatrix[16][4]; // grab takes variables, converts them into values and stores them in this matrix
	float resultmatrixfilter[16][4]; // here filtered values are stored. filter updates them with the results from resultmatrix. 
	float filtermatrix[16][4]; // stores Q, R, K, and P for the filter. 
	float kmatrix[16][3]; // stores divergent K for each axis
	float pmatrix[16][3]; // stores divergent P for each axis.  
	unsigned short tREF_ACT[16]; //stores the reference temperatures for all sensors  
	float offset_matrix[16][3]; // stores x, y and z offsets. It was chosen to use this matrix rather than the offset values on the MLX90393 chip because it gives better control.
	
	
   private:
    
	// Besides, the offset values on the chip are used by the temperature compensation algorithm on the chip. It is still not clear to me exactly how, but,
	// It is clear that the offset values on the chip are used only if TC is turned on. If TC is turned off, there are also other problems to deal with. 
	float convergence_matrix[25];
	float movingaverage[16][4];
	unsigned char Dig_filt_actual; //stores the actual value of digital filter
	unsigned char OSR_actual; // stores the actual value of the OSR
		
	unsigned short TREF; 
  	int UsEL;
  	int _CS;
  	int _CLK;
	float sense_x;
	float sense_y;
	float sense_z;
	unsigned short res_x;
	unsigned short res_y;
	unsigned short res_z;
	unsigned short tcomp;
	unsigned short hallConF;
	bool knownRes;
	bool knowntcomp;
  	char _MISO_P[16];
  	char _MOSI_P[16];
  	int _MISO[17];
  	int _MOSI[17];
	int sensors;
	double gainlookup[8][2];
};

#endif
