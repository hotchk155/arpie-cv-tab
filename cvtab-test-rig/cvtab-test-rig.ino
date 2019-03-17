#include <Wire.h>


#define MV_CV 2
#define MV_CLOCK 0
#define MV_GATE 1

#define MV_ON   8100
#define MV_OFF  5

// parameters for the gain calibration process
#define GAIN_CAL_BASE_NOTE  24
#define GAIN_CAL_INTERVAL   12
#define GAIN_CAL_INTERVAL_MV ((1000.0*(GAIN_CAL_INTERVAL))/12.0)
#define GAIN_CAL_CYCLES     6

// parameters for the offset calibration process
#define OFS_CAL_BASE_NOTE  24
#define OFS_CAL_INTERVAL   12
#define OFS_CAL_BASE_MV ((1000.0*(OFS_CAL_BASE_NOTE-12))/12.0)
#define OFS_CAL_INTERVAL_MV ((1000.0*(OFS_CAL_INTERVAL))/12.0)
#define OFS_CAL_CYCLES     6

#define OCTAVES 8

#define MIDI_CC_NRPN_HI 		99
#define MIDI_CC_NRPN_LO 		98
#define MIDI_CC_DATA_HI 		6
#define MIDI_CC_DATA_LO 		38

#define NRPNH_GLOBAL                    1
#define NRPNH_CV1		        21
#define NRPNH_CV2		        22
#define NRPNH_CV3		        23
#define NRPNH_CV4		        24

#define NRPNL_CAL_SCALE  	        98
#define NRPNL_CAL_OFS  		        99
#define NRPNL_SAVE                      100

// ADC ADDRESS  
#define I2C_ADDR 0b1001111

// I2C ADDRESSES FOR DAUGHTER BOARDS
#define DAC_ADDR    0b1100000
#define EEPROM_ADDR 0b1010000

#define P_LED 11
#define P_SWITCH 9
#define P_HH_CVTAB_GATE 13
#define P_HH_CVTAB_CLOCK 12

char hhCVCalScale = 0;
char hhCVCalOfs = 0;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////
byte hhReadMemory(int addr, byte *dest, int len) 
{

  // set the start address
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(addr>>8);
  Wire.write(addr&0xFF);
  Wire.endTransmission();
  
  // Arduino Wire library can read a maximum
  // of 32 bytes at a time, so we need to read
  // multiple blocks  
  while(len > 0) {  

    // get next block or all remaining bytes if less
    int blockSize = len;
    if(blockSize>32) {
      blockSize = 32;
    }      
    len-=blockSize;
    if(Wire.requestFrom(EEPROM_ADDR, blockSize) != blockSize) {
      return 0;
    }

    // copy to destination
    while(blockSize-- > 0) {
      *dest++ = Wire.read();
    }  
  }
  return 1;
}

/////////////////////////////////////////////////////
// Address must be on a 32-byte boundary, ie (addr & ~31) == 0
byte hhWriteMemory(int addr, byte *src, int len) {    
    // while there are more bytes to send
    while(len > 0) {

      // since the arduino Wire library has a 32 byte buffer size
      // we will need 2 write cycles to fill a 32 byte page on the
      // EEPROM (since 2 byte write address must also be sent)
      // Therefore we'll send each 32 byte page as two 16 bit 
      // writes
      Wire.beginTransmission(EEPROM_ADDR);
      Wire.write(addr>>8);
      Wire.write(addr&0xFF);

      int blockSize = len;
      if(blockSize > 16) {
        blockSize = 16;
      }
      addr+=blockSize;
      len-=blockSize;
      for(int i=0; i<blockSize; ++i) {
        Wire.write(*src++);
      }
      Wire.endTransmission();      
      
      // wait for the write to complete
      for(;;) {
        Wire.beginTransmission(EEPROM_ADDR);
        Wire.write(0);
        Wire.write(0);
        if(2 != Wire.endTransmission()) {
          break;
        }
      }      
    }      
    return 1;
}

// ROUTINES FOR ACCESSING THE CV OUTPUT
void hhSetDAC(int dac) {
      Wire.beginTransmission(DAC_ADDR); 
      Wire.write((dac>>8)&0xF); 
      Wire.write((byte)dac); 
      Wire.endTransmission();         
}
void hhSetCV(long note) {
      long cv = (((note-12) * 500)/12);
      cv = ((cv * (4096 + hhCVCalScale))/4096) + hhCVCalOfs;
      while(cv<0) cv+=500;
      while(cv>4095) cv-=500;
      hhSetDAC(cv);
}

void hhSetGate(byte state) {
  digitalWrite(P_HH_CVTAB_GATE,state);      
}
void hhSetClock(byte state) {
  digitalWrite(P_HH_CVTAB_CLOCK,state);      
}

#define HH_CAL_ADDR     0x1FF7 // top 8 bytes of slot for patch 15
#define HH_CAL_COOKIE   0x12
byte hhCVCalSave() {
    byte data[3] = {
      HH_CAL_COOKIE,
      (byte)hhCVCalScale,
      (byte)hhCVCalOfs
    };
    return hhWriteMemory(HH_CAL_ADDR, data, sizeof(data));
}

byte hhCVCalLoad() {
    byte data[3] = {0};
    if(hhReadMemory(HH_CAL_ADDR, data, sizeof(data))) {
      if(data[0] == HH_CAL_COOKIE) {
        hhCVCalScale = (char)data[1];
        hhCVCalOfs = (char)data[2];
      }
      return 1;
    }
    return 0;
}
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////



void setup() {  

  pinMode(P_LED, OUTPUT);
  pinMode(P_SWITCH, INPUT_PULLUP);
  pinMode(P_HH_CVTAB_GATE, OUTPUT);
  pinMode(P_HH_CVTAB_CLOCK, OUTPUT);

  Wire.begin();
  Serial.begin(9600);
  Serial.println("Begin");


  // configure ADC
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x01);
  Wire.write(0b00011111);
  Wire.endTransmission();

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x02);
  Wire.write(0b0);
  Wire.endTransmission();

  
}


double read_mv() {
  unsigned result = ((int)Wire.read() << 8) | Wire.read();
  int q = result & 0x3FFF;
  if(result & 0x4000) {
    return 0.0;
  }
  else {
    return (q * 2 * 0.30518);
  }
}


// Fetch the voltage output for a note
boolean read_inputs(double mv[]) {

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x06);
  Wire.endTransmission(false);  
  Wire.requestFrom(I2C_ADDR,8,true);

  int q=1000;
  while(Wire.available() < 8 && q>0) {
    --q;
    delayMicroseconds(5);
  }
  mv[0] = read_mv();
  mv[1] = read_mv();
  mv[2] = read_mv();
  mv[3] = read_mv();
  return !!q;
}

void set_scale_adj(int amount) 
{
  if(amount > 63) amount = 63;
  if(amount < -63) amount = -63;
  hhCVCalScale = amount;
}
void set_ofs_adj(int amount) 
{
  if(amount > 63) amount = 63;
  if(amount < -63) amount = -63;
  hhCVCalOfs = amount;
}
void save_calibration() 
{
//  send_nrpn(NRPNH_GLOBAL, NRPNL_SAVE, 0, 0);
}

byte test_note(int note,double mv[]) {
  hhSetCV(note);
  delay(200);
  return read_inputs(mv);
}



// Try a gain adjustment on a channel and return the mean error
boolean try_gain_adjustment(int gain_adj, double& mean_error) 
{
 
  // set the requested gain adjustment
  set_scale_adj(gain_adj);
  set_ofs_adj(0);

  Serial.print("GAIN:");
  Serial.print("trying ");
  Serial.println(gain_adj);
  
  // Scan through the required number of octaves
  int note = GAIN_CAL_BASE_NOTE;
  double delta_total = 0;
  double delta = 0;
  double last_result = 0;
  for(int i=0; i<GAIN_CAL_CYCLES; ++i) {
    double mv[4];
    test_note(note,mv);
    double result = mv[MV_CV];
    Serial.print("GAIN:");
    Serial.print("note ");
    Serial.print(note, DEC);
    Serial.print("->");
    Serial.print(result);
    if(i>0) {
      // get difference between this and the last result
      delta = result - last_result;
      if(abs(GAIN_CAL_INTERVAL_MV-delta) > (GAIN_CAL_INTERVAL_MV/10)) {
        Serial.print(" *** FAIL - OUT OF TOLERANCE ");
        Serial.println(delta);
        return false;
      }
      // subtract the expected 1000mV between octaves so that we get the 
      // deviation from the expected difference
      delta -= GAIN_CAL_INTERVAL_MV;
      Serial.print(" diff ");
      Serial.print(delta);

      // total up all the deviations from expected
      delta_total += delta;
    }    
    Serial.println("");
    
    // store last result
    last_result = result;
        

    // ready for the next octave
    note += GAIN_CAL_INTERVAL;
    
  }
  
  // get the mean deviation from 1V/octave across all octaves
  delta_total /= (GAIN_CAL_CYCLES-1);
  Serial.print("mean ");
  Serial.println(delta_total);
  mean_error = delta_total;  
  return true;
}

//////////////////////////////////////////////////////////////////////////
boolean gain_calibration() 
{
  double mean_error;
  int gain_adj = 0;

  Serial.print("GAIN:");
  
  // establish the initial gain adjustment
  if(!try_gain_adjustment(0, mean_error)) 
    return false;
  gain_adj = 0.5 - 4.0 * mean_error;
  if(gain_adj < -60 || gain_adj > 60) {
    Serial.println("*** ERROR: OUT OF TOLERANCE ***");
    return false;
  }
  Serial.print("initial adjustment ");
  Serial.println(gain_adj);  

  
  double min_mean_error = 9999999;
  int low_adj = gain_adj - 5;
  int high_adj = gain_adj + 5;
  low_adj = constrain(low_adj, -63, 63);
  high_adj = constrain(high_adj, -63, 63);
  
  // multiple attempts to find the offset value which works best
  for(int this_adj = low_adj; this_adj <= high_adj; ++this_adj) {

    if(!try_gain_adjustment(this_adj, mean_error)) 
      return false;
    
    // is this the best one yet?
    if(abs(mean_error) < abs(min_mean_error)) {
      min_mean_error = mean_error;
      gain_adj = this_adj;
      Serial.println("    >>>> best result so far");
    }     
  }

  // store best value
  set_scale_adj(gain_adj);
  Serial.println("");
  Serial.print("GAIN adjustment =");
  Serial.print(gain_adj);
  Serial.print(" with mean error of ");
  Serial.println(min_mean_error);
  return true;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// OFFSET CALIBRATION ROUTINES
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
boolean try_offset_adjustment(int offset_adj, double& mean_error) 
{
 

  Serial.print("OFS:");
  Serial.print("Trying ");
  Serial.println(offset_adj);
  set_ofs_adj(offset_adj);


  mean_error = 0;

  int note = OFS_CAL_BASE_NOTE;
  double expected_result = OFS_CAL_BASE_MV;
  double this_error = 0;
  for(int i=0; i<OFS_CAL_CYCLES; ++i) {
    double mv[4];
    test_note(note,mv);
    double result = mv[MV_CV];
    this_error = result - expected_result;
    mean_error += this_error;
    Serial.print("OFS:");
    Serial.print("note ");
    Serial.print(note, DEC);
    Serial.print(" expected ");
    Serial.print(expected_result);
    Serial.print("->");
    Serial.print(result);
    Serial.print(" error ");
    Serial.println(this_error);
    //if(abs(this_error) > (OFS_CAL_INTERVAL_MV/5)) {
    //    Serial.println(" *** FAIL - OUT OF TOLERANCE ");
    //    return false;
    //}

    // ready for the next octave
    note += OFS_CAL_INTERVAL;
    expected_result += OFS_CAL_INTERVAL_MV;
  }
  mean_error /= OFS_CAL_CYCLES;
  Serial.print("mean ");
  Serial.println(mean_error);
  return true;
}


//////////////////////////////////////////////////////////////////////////
boolean offset_calibration() 
{
  double mean_error;
  int ofs_adj = 0;

  Serial.print("OFS:");
  
  // establish the initial gain adjustment
  if(!try_offset_adjustment(0, mean_error)) 
    return false;

  // intial offset adjustment is (-e
  ofs_adj = (int)(0.5 - mean_error);

  double min_mean_error = 9999999999;
  
  if(mean_error < 0) { // voltages are too low
    ofs_adj = (int)(fabs(mean_error/2)+0.5);
  }
  else {
    ofs_adj = -(int)(fabs(mean_error/2)+0.5);
  }
  if(ofs_adj < -60 || ofs_adj > 60) {
    Serial.println("*** ERROR: OUT OF TOLERANCE ***");
    return false;
  }
  int low_adj = constrain(ofs_adj-3, -63, 63);
  int high_adj = constrain(ofs_adj+3, -63, 63);
  Serial.print("initial adjustment ");
  Serial.println(ofs_adj);  

  
  
  // multiple attempts to find the offset value which works best
  for(int this_adj = low_adj; this_adj <= high_adj; ++this_adj) {

    if(!try_offset_adjustment(this_adj, mean_error)) 
      return false;
    
    // is this the best one yet?
    if(abs(mean_error) < abs(min_mean_error)) {
      min_mean_error = mean_error;
      ofs_adj = this_adj;
      Serial.println("    >>>> best result so far");
    }     
  }

  // store best value
  set_ofs_adj(ofs_adj);
  Serial.println("");
  Serial.print("OFFSET adjustment =");
  Serial.print(ofs_adj);
  Serial.print(" with mean error of ");
  Serial.println(min_mean_error);
  return true;
}


void octave_check() 
{  
  // re-evaluate
  int note = 24;
  int error;
  int i;
  double error_total = 0;
  int expected = 1000;
  double mv[4] = {   0  };
  
  Serial.print("Calibration check CV");

  for(i=0; i<OCTAVES; ++i) {
    if(!test_note(note,mv)) {
      Serial.println("*** COMMS ERROR ***");
    }

    error = mv[MV_CV] - expected;
    error_total += error;

    Serial.print("C");
    Serial.print(i+1, DEC);
    Serial.print("->");
    Serial.print(mv[MV_CV], DEC);
    Serial.print("mV");
    Serial.print(" expected ");
    Serial.print(expected);
    Serial.print("mV");
    Serial.print(" error ");
    Serial.print(error);
    Serial.println("mV");    
    expected += 1000;
    note += 12;
  }
}

boolean calibrate() {
    octave_check();    
    if(!gain_calibration()) {
        return false;
     }
    if(!offset_calibration()) {
      return false;
    }
    octave_check();    
  save_calibration();
  Serial.println("Done...");
  Serial.print("scale ");
  Serial.print(hhCVCalScale, DEC);
  Serial.print(" ");
  Serial.print("ofs ");
  Serial.print(hhCVCalOfs, DEC);
  Serial.println(" ");
  return 1;
}

void pad(int val, int pad, boolean sign) {
  int a = fabs(val);
  if(a>=1000) pad -= 4;
  else if(a>=100) pad -= 3;
  else if(a>=10) pad -= 2;
  else --pad;
  if(sign) --pad;
  while(pad-->0) {
    Serial.print(" ");
  }
  if(sign && val>=0) {
    Serial.print("+");    
  }
  Serial.print(val, DEC);
}

void scale_check() 
{  
  // re-evaluate
  int note = 36;
  double expected = 1000.0;
  double mv[4] = {   0  };
  
  //Serial.println("Calibration check");
  //all_notes_off();
  //delay(1000);

  double td0 = 0.0;
  int readings = 0;
  while(note <= 120) {
    test_note(note,mv);

    double d0 = mv[0]-expected;

    td0 += d0;
    ++readings;
    
    pad(note,3,false);
    Serial.print(" | ");

    Serial.print(expected);
    Serial.print(" | ");
    Serial.print(mv[0]);
    Serial.print(" | ");
    pad((int)(0.5+d0),5,true);

    Serial.println("");    
    expected += 1000.0/12.0;
    note ++;
  }
  
  Serial.print("CV error - mean ");  Serial.print(td0/readings); Serial.print(", total "); Serial.println(td0);
  
}

byte TEST_memory() {
  byte p0[32];
  byte p1[32];
  byte p2[32];
  byte tt[32];

  memset(p0, 0xAA, 32);
  memset(p1, 0x55, 32);
  memset(p2, 0x00, 32);
  memset(tt, 0x00, 32);
  Serial.print("MEMORY TEST");
  for(int addr=0; addr<8192; addr+=32) {
    if(!hhWriteMemory(addr, p0, 32) || !hhReadMemory(addr, tt, 32) || memcmp(p0,tt,32)) {
      Serial.print("***FAILED(AA) at ");
      Serial.println(addr, DEC);
      return 0;
    }
    if(!hhWriteMemory(addr, p1, 32) || !hhReadMemory(addr, tt, 32) || memcmp(p1,tt,32)) {
      Serial.print("***FAILED(55) at ");
      Serial.println(addr, DEC);
      return 0;
    }
    if(!hhWriteMemory(addr, p2, 32) || !hhReadMemory(addr, tt, 32) || memcmp(p2,tt,32)) {
      Serial.print("***FAILED(00) at ");
      Serial.println(addr, DEC);
      return 0;
    }    
    if(!(addr&0xFF)) {
      Serial.print(".");
    }
  }
  Serial.println("PASS");
  return 1;

}

byte TEST_gate() {
  double mv[4];
  Serial.print("TEST GATE");
  hhSetClock(0);
  hhSetGate(0);

  for(int i=0; i<10; ++i) {
    Serial.print(".");
    hhSetGate(0);
    delay(100);
    if(!read_inputs(mv)) {
      Serial.println("***FAILED reading DAC");
      return 0;
    }
    if(mv[MV_GATE] > MV_OFF || mv[MV_CLOCK] > MV_OFF) {
      Serial.println("***FAILED voltage error OFF expected");
      return 0;  
    }
  
    hhSetGate(1);
    delay(100);
    if(!read_inputs(mv)) {
      Serial.println("***FAILED reading DAC");
      return 0;
    }
    if(mv[MV_GATE] < MV_ON || mv[MV_CLOCK] > MV_OFF) {
      Serial.println("***FAILED voltage error ON expected");
      return 0;  
    }
  }  
  Serial.println("PASS");
  return 1;
}

byte TEST_clock() {
  double mv[4];
  Serial.print("TEST CLOCK");
  hhSetClock(0);
  hhSetGate(0);

  for(int i=0; i<10; ++i) {
    Serial.print(".");
    hhSetClock(0);
    delay(100);
    if(!read_inputs(mv)) {
      Serial.println("***FAILED reading DAC");
      return 0;
    }
    if(mv[MV_CLOCK] > MV_OFF || mv[MV_GATE] > MV_OFF) {
      Serial.println("***FAILED voltage error OFF expected");
      return 0;  
    }
  
    hhSetClock(1);
    delay(100);
    if(!read_inputs(mv)) {
      Serial.println("***FAILED reading DAC");
      return 0;
    }
    if(mv[MV_CLOCK] < MV_ON || mv[MV_GATE] > MV_OFF) {
      Serial.println("***FAILED voltage error ON expected");
      return 0;  
    }
  }  
  Serial.println("PASS");
  return 1;
}

int d=0;
void loop() {  

  digitalWrite(P_LED, HIGH);
  while(digitalRead(P_SWITCH));

  // configure DAC
  Wire.beginTransmission(DAC_ADDR); 
  Wire.write(0b10011001); // buffered Vref, powered up, 2x
  Wire.endTransmission();       
  
  if(TEST_clock() && TEST_gate()) {
    if(calibrate()) {
      if(TEST_memory()) {
        hhCVCalSave();
        Serial.println("SUCCESS");
      }
    }
  }
  
/*  
  if(d>4095) d=0; else d+=100;
  //hhSetDAC(d);
  delay(100);
  int mv[4];
  digitalWrite(P_HH_CVTAB_GATE, digitalRead(P_SWITCH));
  if(read_inputs(mv))
  {
    Serial.print(mv[0],DEC);
    Serial.print(",");
    Serial.print(mv[1],DEC);
    Serial.print(",");
    Serial.print(mv[2],DEC);
    Serial.print(",");
    Serial.print(mv[3],DEC);    
    Serial.println(".");
  }  */
}






