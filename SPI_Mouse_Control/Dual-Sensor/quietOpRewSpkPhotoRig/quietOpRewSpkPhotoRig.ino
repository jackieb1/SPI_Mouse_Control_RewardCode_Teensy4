#include <SPI.h>
#include "init_variables.h"

byte initComplete=0;
byte Motion = 0;
byte xH;
byte xL;
byte yH;
byte yL;
int xydat[2];
int xy2dat[2];
int incomingByte = 0;   // for incoming serial data
double dP;
double dR;
double dY;

// pins
const int ncs = 0;
const int ncs2 = 1;
const int pVelPin = 3;
const int rVelPin = 4;
const int yVelPin = 5;
const int spkr = 23;
const int valv = 22;
const int licks2 = 17;

//variables
//const int durCorrTone = 150; 
//const int durErrTone = 150; 
//const int timeValv = 100;
int checkRewTime = 0;
int checkErrToneTime = 0;

unsigned long timeRewStart;
unsigned long timeErrorToneStart;
unsigned long valveDur = 0; // reward valve open time (ms) from the latest serial command

// Velocity output is updated once per fixed wall-clock interval rather than
// once per loop(). loop() runs much faster on the Teensy 4 than the old
// Teensy 3, so a per-loop write made the time-averaged voltage per ball
// rotation scale with the loop period (~10x smaller here). Accumulating raw
// sensor counts and emitting them on a fixed interval makes the output
// independent of loop() speed, so the calibrated gain is portable across rigs.
long accX1 = 0;
long accY1 = 0;
long accX2 = 0;
long accY2 = 0;
elapsedMicros velTimer;                     // microseconds since last DAC update
const unsigned long velIntervalUs = 10000;  // fixed 10 ms velocity update window

// Clamp a velocity value to the 12-bit DAC range [0, 4095].
int clampDAC(double v) {
  long iv = lround(v);
  if (iv < 0)    iv = 0;
  if (iv > 4095) iv = 4095;
  return (int)iv;
}


const double px1 = 0.8151;
const double rx1 = 0.1849;
const double yx1 = -0.5959;
const double py1 = 0.2414;
const double ry1 = -0.2414;
const double yy1 = -0.7779;
const double px2 = -0.1849;
const double rx2 = -0.8151;
const double yx2 = 0.5959;
const double py2 = -0.2414;
const double ry2 = 0.2414;
const double yy2 = -0.7779;

//const double px1 = 1;
//const double rx1 = 0;
//const double yx1 = 0;
//const double py1 = 0;
//const double ry1 = 0;
//const double yy1 = -1.557;
//const double px2 = 0;
//const double rx2 = -1;
//const double yx2 = 1.1918;
//const double py2 = 0;
//const double ry2 = 0;
//const double yy2 = 0;

// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

extern const unsigned short firmware_length;
extern const uint8_t firmware_data[];

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  analogWriteFrequency(pVelPin,11500);
  analogWriteFrequency(rVelPin,11500);
  analogWriteFrequency(yVelPin,11500);
  analogWriteResolution(12);
  pinMode (ncs, OUTPUT);
  pinMode (ncs2, OUTPUT);
  pinMode(valv,OUTPUT);  
  pinMode(spkr,OUTPUT);
  pinMode(licks2,OUTPUT);

  digitalWrite(valv,LOW);
  digitalWrite(licks2,LOW);

  
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  
  delay(1000);
  performStartup();
  delay(10);
  performStartup2();
  delay(10);
  adns_write_reg(REG_Configuration_I, 0x10);
  //adns_write_reg(REG_Configuration_I, 0x29); // maximum resolution
  //adns_write_reg(REG_Configuration_I, 0x09); // default resolution
  //adns_write_reg(REG_Configuration_I, 0x01); // minimum resolution
  delay(10);
  adns2_write_reg(REG_Configuration_I, 0x10);
  //adns2_write_reg(REG_Configuration_I, 0x09); // default resolution
  //adns2_write_reg(REG_Configuration_I, 0x01); // minimum resolution
  delay(1500);  
  dispRegisters();
  delay(1500);
  dispRegisters2();
  delay(1500);
  initComplete=9;
  


}

void adns_com_begin(){
  digitalWrite(ncs, LOW);
}

void adns2_com_begin(){
  digitalWrite(ncs2, LOW);
}

void adns_com_end(){
  digitalWrite(ncs, HIGH);
}

void adns2_com_end(){
  digitalWrite(ncs2, HIGH);
}

byte adns_read_reg(byte reg_addr){
  adns_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

byte adns2_read_reg(byte reg_addr){
  adns2_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns2_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data){
  adns_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns2_write_reg(byte reg_addr, byte data){
  adns2_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns2_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  //Serial.println("Uploading firmware to chip 1...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
}

void adns2_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  //Serial.println("Uploading firmware to chip 2...");
  // set the configuration_IV register in 3k firmware mode
  adns2_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns2_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns2_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns2_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns2_com_end();
}


void performStartup(void){
  adns_com_end(); // ensure that the serial port is reset
  adns_com_begin(); // ensure that the serial port is reset
  adns_com_end(); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  
  delay(10);

  //Serial.println("Optical Chip 1 Initialized");
}

void performStartup2(void){
  adns2_com_end(); // ensure that the serial port is reset
  adns2_com_begin(); // ensure that the serial port is reset
  adns2_com_end(); // ensure that the serial port is reset
  adns2_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns2_read_reg(REG_Motion);
  adns2_read_reg(REG_Delta_X_L);
  adns2_read_reg(REG_Delta_X_H);
  adns2_read_reg(REG_Delta_Y_L);
  adns2_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns2_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0_2 = adns2_read_reg(REG_LASER_CTRL0);
  adns2_write_reg(REG_LASER_CTRL0, laser_ctrl0_2 & 0xf0 );
  
  delay(10);

  //Serial.println("Optical Chip 2 Initialized");
}


void dispRegisters(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x0F  };
  const char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","CPI"  };
  byte regres;

  digitalWrite(ncs,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    //Serial.println("---");
    //Serial.println(oregname[rctr]);
    //Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    //Serial.println(regres,BIN);  
    //Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ncs,HIGH);
}

void dispRegisters2(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x0F  };
  const char* oregname[] = {
    "Product_ID2","Inverse_Product_ID2","SROM_Version2","CPI2"  };
  byte regres;

  digitalWrite(ncs2,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    //Serial.println("---");
    //Serial.println(oregname[rctr]);
    //Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    //Serial.println(regres,BIN);  
    //Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ncs2,HIGH);
}

void readXY(int *xy){
  //digitalWrite(ncs,LOW);
  
  Motion = (adns_read_reg(REG_Motion) & (1 << 8-1)) != 0;
  xL = adns_read_reg(REG_Delta_X_L);
  xH = adns_read_reg(REG_Delta_X_H);
  yL = adns_read_reg(REG_Delta_Y_L);
  yH = adns_read_reg(REG_Delta_Y_H);
  xy[0] = (xH << 8) + xL;
  xy[1] = (yH << 8) + yL;

  if(xy[0] & 0x8000){
    xy[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy[1] & 0x8000){
    xy[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
  
  //digitalWrite(ncs,HIGH);     
}

void readXY2(int *xy){
  //digitalWrite(ncs2,LOW);
  
  Motion = (adns2_read_reg(REG_Motion) & (1 << 8-1)) != 0;
  xL = adns2_read_reg(REG_Delta_X_L);
  xH = adns2_read_reg(REG_Delta_X_H);
  yL = adns2_read_reg(REG_Delta_Y_L);
  yH = adns2_read_reg(REG_Delta_Y_H);
  xy[0] = (xH << 8) + xL;
  xy[1] = (yH << 8) + yL;

  if(xy[0] & 0x8000){
    xy[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy[1] & 0x8000){
    xy[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
  
  //digitalWrite(ncs2,HIGH);     
}


// Parse a 6-digit reward command from ViRMen (giveReward.m -> writeValveCommand):
// chars 0-2 = valve1 ms, chars 3-5 = valve2 ms. ViRMen sends the reward duration
// (set in getRigInfo.m rewardPulseDurationDict) in the valve2 field, so open the
// reward solenoid (valv) for that many ms.
void interpretCommand(String message) {
  message.trim();
  if (message.length() != 6) {
    Serial.println("#"); // "#" means error / malformed command
    return;
  }
  long dur = message.substring(3).toInt(); // valve2 field carries the reward duration
  if (dur > 0) {
    digitalWrite(valv, HIGH);
    timeRewStart = millis();
    valveDur     = dur;
    checkRewTime = 1;
    Serial.println(String(3) + '\t' + String(millis())); // reward-on event code (unchanged)
  }
}


  void loop() {

    // Read both sensors every loop and accumulate raw delta-counts. Reads
    // auto-clear the sensor delta registers, so counts are conserved across
    // the interval (nothing is lost between DAC updates).
    readXY(&xydat[0]);
    readXY2(&xy2dat[0]);
    accX1 += xydat[0];
    accY1 += xydat[1];
    accX2 += xy2dat[0];
    accY2 += xy2dat[1];

    // Emit velocity once per fixed interval, regardless of loop() rate.
    if (velTimer >= velIntervalUs) {
      velTimer -= velIntervalUs;   // subtract (don't zero) to stay phase-locked, no drift

      dP = px1*accX1 + py1*accY1 + px2*accX2 + py2*accY2;
      dR = rx1*accX1 + ry1*accY1 + rx2*accX2 + ry2*accY2;
      dY = yx1*accX1 + yy1*accY1 + yx2*accX2 + yy2*accY2;

      analogWrite(pVelPin, clampDAC(dP + 2048));
      analogWrite(rVelPin, clampDAC(dR + 2048));
      analogWrite(yVelPin, clampDAC(dY + 2048));

      accX1 = 0; accY1 = 0; accX2 = 0; accY2 = 0;

      // If loop() blocked long enough to fall a whole interval behind, resync
      // rather than firing repeatedly to "catch up".
      if (velTimer >= velIntervalUs) {
        velTimer = 0;
      }
    }
    
        // Process all available serial bytes. Tone commands stay single
        // non-printable bytes (1 = correct, 2 = error). Reward is now a
        // \n-terminated 6-digit ASCII command from ViRMen (giveReward.m ->
        // writeValveCommand); tone bytes (1, 2) and 'S' never collide with the
        // ASCII digits / '\n' of the valve command.
        static String usbMessage = "";   // reward-command buffer, persists across loops
        while (Serial.available() > 0) {
          char inByte = Serial.read();
          if (inByte == 'S') {                // ViRMen connectToTeensy handshake
            Serial.println('S');
          } else if (inByte == 1) {           // play correct tone
            Serial.println(String(1) + '\t' + String(millis()));
            tone(spkr,7000,durCorrTone);
            Serial.println(String(4) + '\t' + String(millis()));
          } else if (inByte == 2) {           // play noise tone
            Serial.println(String(2) + '\t' + String(millis()));
            tone(spkr,1000,durErrTone);
            Serial.println(String(5) + '\t' + String(millis()));
          } else if (inByte == '\n') {        // complete reward command
            interpretCommand(usbMessage);
            usbMessage = "";
          } else {                            // accumulate digits of the reward command
            usbMessage = usbMessage + inByte;
          }
        }

              if(checkRewTime == 1){

              unsigned long tHigh = millis()-timeRewStart;
              if((tHigh>valveDur) || (tHigh>10000) ){

                     digitalWrite(valv,LOW);
                     Serial.println(String(6) + '\t' + String(millis()));
                     checkRewTime = 0;

                   }

              }


//               if (checkErrToneTime == 1){
//
//               if((millis()-timeErrorToneStart)<durErrTone){
//                   if (random(2) == 1)
//                   {
//                    digitalWrite(spkr,HIGH);
//                   }
//                 else
//                 {
//                    digitalWrite(spkr,LOW);
//                    }
//                  }
//
//                  else{
//
//                    checkErrToneTime = 0;
//                    digitalWrite(spkr,LOW);
//                    Serial.println(String(5) + '\t' + String(millis()));
//                  }
//
//                
//               }

    // No delay(): loop() runs at the SPI-limited rate so multiple sensor
    // reads accumulate per velIntervalUs window. The DAC update cadence is set
    // by velTimer above, not by a blocking delay.

  }

