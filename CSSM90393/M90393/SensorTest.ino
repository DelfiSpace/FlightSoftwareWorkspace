
/*
#define LED RED_LED
#define show1 75
#define show2 76
#define show3 77
#define show4 78



#define numsensors 1

#include <M90393.h>  
//create class of M90393 called m
M90393 m(numsensors);
// define global output matrices
bool ledflip = true;
unsigned char shortoutputmatrix[numsensors];
unsigned char longoutputmatrix[numsensors][16];
int x_MISO[17] = {MISO0, MISO1, MISO2, MISO3, MISO4, MISO5, MISO6, MISO7, MISO8, MISO9, MISO10, MISO11, MISO12, MISO13, MISO14, MISO15, MISO16};


void setup() 
{
  // put your setup code here, to run once:
  digitalWrite(CS, HIGH); 
  digitalWrite(SCLK, HIGH); 
  digitalWrite(MOSI0, HIGH);
  pinMode(CS, OUTPUT);
  pinMode(SCLK, OUTPUT);
 // pinMode(MISO0, INPUT_PULLUP); done by class
  pinMode(MOSI0, OUTPUT);
  pinMode(LED, OUTPUT); 
   pinMode(show1, OUTPUT);
    pinMode(show2, OUTPUT);
     pinMode(LED, OUTPUT);
      pinMode(LED, OUTPUT);
  Serial.begin(9600); 

}

void loop() 
{
  // put your main code here, to run repeatedly: 
  delay(100);
  //Serial.println("At least you know the serial port is open");
  delay(5000);
  //Serial.println(ledflip ? "true" : "false");
  unsigned char t;
  Serial.println();
  Serial.println("Reset");
  t = cmdRT();
  Serial.print("CdmSMstatus = "); Serial.println(t, BIN);
  delay(DELAY_TIME);
   Serial.println("Exit");
  t = m.cmdEX();
  Serial.print("CdmSMstatus = "); Serial.println(t, BIN);

 // Serial.println();
  //Serial.println(t, BIN);
  //Serial.println(0xF0, BIN)//Serial.write(63);
 // Serial.println("executing pin test");
 // int tim = m.pintest2();
 // Serial.print("time to read 16 input pins is : "); Serial.print(tim); 
//  Serial.println(" microseconds");
  Serial.println("STart Measurement");
  t = m.cmdSM(0xF);
  //Serial.println("send measurement command zxyt = 1111");
 // unsigned char cmdChar = 0x30 | 0xF; 
 // unsigned char inputchar = 0x30;
 // inputchar |= 0xF;
  //Serial.print("inputchar = ");Serial.println(inputchar, BIN);
  delay(250);
 // Serial.print("CdmSM = "); Serial.println(cmdChar, BIN);
 Serial.print("CdmSMstatus = "); Serial.println(t, BIN);
  cmdRM(0xF);
  Serial.println("read measurement");
  //Serial.print("CmdRM = " ); Serial.println(meas, BIN);
  for(int count8 = 0; count8 < 16; count8++){
   Serial.print("word; "); Serial.print(longoutputmatrix[0][count8], BIN);
    
    }

  unsigned short temp = retrieveT(0);
  Serial.println();
  
  Serial.print("temperature"); Serial.println(temp, HEX); 
  Serial.print("temperature"); Serial.println(temp);   
  Serial.println(); Serial.println("end  measurement");
  
}
//Command codes not in library.  
void cmdRM(char zyxt){
    unsigned char inputchar = 0x40;
    // warning add zyxt
    inputchar = inputchar|zyxt;
    //readMLX90393_multi(shortoutputmatrix, numsensors);
    sendCommandlong(inputchar, 0x00, 0x00, 0x00, numsensors);
}

unsigned char cmdRT()
{
    unsigned char inputchar = 0xF0;
    return m.sendCommand1(inputchar);
}



unsigned char sendCommandshort(unsigned char c,  unsigned char shortoutputmatrix[])
{
  digitalWrite(SCLK, HIGH);
  digitalWrite(CS, LOW); 

  delay(DELAY_TIME*4);

  m.writeMLX90393(c);

  delay(DELAY_TIME);

  unsigned char tmp = m.readMLX90393();

  delay(DELAY_TIME);

  digitalWrite(CS, HIGH);
  digitalWrite(MOSI0, HIGH);

  return tmp;
  
}

unsigned int sendCommandAdv(unsigned char c){
  digitalWrite(SCLK, HIGH); 
  digitalWrite(CS, LOW); 
  delay(DELAY_TIME*4); 
  m.writeMLX90393(c);
  delay(DELAY_TIME*4);
  unsigned char tmp1 = m.readMLX90393();
  unsigned char tmp2;
  unsigned char tmp3;
  unsigned char tmp4;
  
  if (tmp1 & 0x02){
    delay(DELAY_TIME);
    tmp2 = m.readMLX90393();
    delay(DELAY_TIME);
    tmp3 = m.readMLX90393();
    if (tmp1 & 0x01){
      delay(DELAY_TIME);
      tmp4 = m.readMLX90393();
    }
  }
  
    else{
  unsigned int tmpI = (unsigned int)tmp1; 
  delay(DELAY_TIME);
  digitalWrite(CS, HIGH);   
  return tmpI;
  }
  unsigned int tmpI;
  tmpI |= (unsigned int)tmp1 << 24;
  tmpI |= (unsigned int)tmp2 << 16;
  tmpI |= (unsigned int)tmp3 << 8;
  tmpI |= (unsigned int)tmp4 << 0;
  delay(DELAY_TIME);
  digitalWrite(CS, HIGH);
  digitalWrite(MOSI0, HIGH);
return tmpI; 
}

void sendCommandlong(unsigned char c, unsigned char c2, unsigned char c3, unsigned char c4, int ArraySize){
  // send command bytes
  writeseqMLX90393(c, c2, c3, c4);
  // read return bytes
  readMLX90393_multi(shortoutputmatrix, ArraySize);
  transportMLX90393(ArraySize, 0); // status byte becomes first byte of return
  for(int count8 = 1; count8 < 16; count8++){
  readMLX90393_multi(shortoutputmatrix, ArraySize);
   transportMLX90393(ArraySize, count8); // transfers bytes from short output to long output
  delay(DELAY_TIME);
  }
  
  unsigned char masked_n = shortoutputmatrix[0] & 0x02;
//  if (masked_n == 0x02){ 
//    transportMLX90393(ArraySize, 0); // status byte becomes first byte of return
//    for(int count8 = 1; count8 < 16; count8++){
//    readMLX90393_multi(shortoutputmatrix, ArraySize);
//    transportMLX90393(ArraySize, count8); // transfers bytes from short output to long output
//    delay(DELAY_TIME);
//    }
//  }
}

signed int retrieveZ(int sensor){
  unsigned int tmp;
  tmp |= longoutputmatrix[sensor][2];
  tmp |= (tmp << 8) ; 
  tmp |= longoutputmatrix[sensor][3];
  signed int tmpa;
  int masked_n = (0x01 << 15) & tmp;
  signed int multiplier = -1; 
  if ((0x01 << 15) == masked_n){
    tmp = tmp - (0x01 << 15);
    tmpa = multiplier*abs(tmp);
    return tmpa; 
  }else{
    tmpa = abs(tmp);
  return tmpa; 
  }
}

signed int retrieveX(int sensor){
  // test first
  unsigned int tmp;
  tmp |= longoutputmatrix[sensor][5];
  tmp |= (tmp << 8) ; 
  tmp |= longoutputmatrix[sensor][6];
  signed int tmpa;
  int masked_n = (0x01 << 15) & tmp;
  signed int multiplier = -1; 
  if ((0x01 << 15) == masked_n){
    tmp = tmp - (0x01 << 15);
    tmpa = multiplier*abs(tmp);
    return tmpa; 
  }else{
    tmpa = abs(tmp);
  return tmpa; 
  }
}

signed int retrieveY(int sensor){
  // test first
  unsigned int tmp;
  tmp |= longoutputmatrix[sensor][8];
  tmp |= (tmp << 8) ; 
  tmp |= longoutputmatrix[sensor][9];
  signed int tmpa;
  int masked_n = (0x01 << 15) & tmp;
  signed int multiplier = -1; 
  if ((0x01 << 15) == masked_n){
    tmp = tmp - (0x01 << 15);
    tmpa = multiplier*abs(tmp);
    return tmpa; 
  }else{
    tmpa = abs(tmp);
  return tmpa; 
  }
}

unsigned short retrieveT(int sensor){
  // test first
  unsigned short tmp;
  tmp = longoutputmatrix[sensor][1];
  tmp |= (tmp << 8) ; 
  tmp |= longoutputmatrix[sensor][2];
  return tmp;
  //signed int tmpa;
  //int masked_n = (0x01 << 15) & tmp;
  //signed int multiplier = -1; 
  //if ((0x01 << 15) == masked_n){
  //  tmp = tmp - (0x01 << 15);
  //  tmpa = multiplier*abs(tmp);
  //  return tmpa; 
  //}else{
  //  tmpa = abs(tmp);
  //return tmpa; 
  //}
}

double interpretZ(int tmp, int gainselect, int res){
  float tmp1 = m.getGainZ(res, gainselect);
  double tmp2 = (double)tmp; 
 // Serial.print(" tmp 2     = " ); Serial.println(tmp2); 
 // Serial.print(" tmp 1     = " ); Serial.println(tmp1); 
  tmp2 = tmp2*tmp1;
  return tmp2; 
}

double interpretT(unsigned short tmp){
  float temp = tmp ; 
  temp = (temp - 46244) / (-45.2);
  
  = 46244 - tmp; 
  float tmp1 = 45.2;
  double tmp2 = (double)tmp; 
 // Serial.print(" tmp 2     = " ); Serial.println(tmp2); 
 // Serial.print(" tmp 1     = " ); Serial.println(tmp1); 
  
  tmp2 = tmp2*tmp1;
  
  return tmp2; 
}

void writeseqMLX90393(unsigned char c, unsigned char c2, unsigned char c3, unsigned char c4){
  digitalWrite(SCLK, HIGH); 
  digitalWrite(CS, LOW); 
  delay(DELAY_TIME*4); 
  m.writeMLX90393(c);
  delay(DELAY_TIME*3); 
    if ( c2 == 0x00) {}
    else{  
          m.writeMLX90393(c2);
          delay(DELAY_TIME*3);
          if ( c3 == 0x00 ){}
          else{
              m.writeMLX90393(c3);
              delay(DELAY_TIME*3);
              m.writeMLX90393(c4);
              delay(DELAY_TIME*4);
          }     
    }
}

void transportMLX90393(int ArraySize, int partsel){
  
  for(int count = 0; count < ArraySize; count++){
    unsigned char tmp = shortoutputmatrix[count];
    longoutputmatrix[count][partsel] =  tmp;
  }
}



void readMLX90393_multi(unsigned char *shortoutputmatrix, int ArraySize){
  // size of the array 
  unsigned char tmp = 0;
  for(int count2 = 0; count2 < ArraySize; count2++){
        shortoutputmatrix[count2] =0x00;
      }

 for(int count = 1; count < 9; count++){
      digitalWrite(SCLK, HIGH);
      delay(DELAY_TIME);
      for(int count2 = 0; count2 < ArraySize; count2++){
        shortoutputmatrix[count2] |= (digitalRead(x_MISO[count2]) & 0x01 ) << (8 - count);
      }
      delay(DELAY_TIME);
      digitalWrite(SCLK, LOW);
      delay(DELAY_TIME);   
  }
  Serial.print("readMLX90393 feedback = "); Serial.println(shortoutputmatrix[0], BIN); 
  digitalWrite(SCLK, HIGH);
}

void setCS(bool value)
{
  digitalWrite(CS, value ? HIGH : LOW);
}
*/
