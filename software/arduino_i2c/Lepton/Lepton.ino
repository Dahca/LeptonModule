/*
Copyright (c) 2014, Pure Engineering LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Wire.h>
#include <SPI.h>

byte x = 0;
#define ADDRESS (0x2A)

#define AGC (0x01)
#define SYS (0x02)
#define VID (0x03)
#define OEM (0x08)

#define GET (0x00)
#define SET (0x01)
#define RUN (0x02)

#define VOSPI_FRAME_SIZE (164)

SPISettings flir_settings(25000000, MSBFIRST, SPI_MODE3);
#define CS_PIN 10

void setup() {
  //pinMode(pin, INPUT);           // set pin to input
  //digitalWrite(pin, HIGH);

  Wire.begin();
  Serial.begin(115200);
  while(!Serial); // wait for the Serial monitor

  pinMode(CS_PIN, OUTPUT);
  //SPI.setDataMode(SPI_MODE3);
  //SPI.setClockDivider(0);

  SPI.begin();

  Serial.println("setup complete");
}

void scan_for_i2c() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    Wire.write("01234567890123456789");
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error <= 4) {
      Serial.print("Unknown error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.print(address,HEX);
      Serial.print(" error = ");
      Serial.println(error, HEX);
    } else {
      Serial.print("Bytes = ");
      Serial.println(error);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
}

int spi_read_word(int data) {
  int read_data;
  SPI.beginTransaction(flir_settings);
  // take the CS pin low to select the chip:
  digitalWrite(CS_PIN, LOW);
  // send in the address and value via SPI:
  read_data = SPI.transfer(data >> 8) << 8;
  read_data |= SPI.transfer(data);
  // take the CS pin high to de-select the chip:
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return read_data;
}

#define IMAGE_SIZE (800)
byte lepton_frame_packet[VOSPI_FRAME_SIZE];
byte image[IMAGE_SIZE];
int image_index;

void read_lepton_frame(void) {
  for (int i = 0; i < (VOSPI_FRAME_SIZE / 2); i++) {
    SPI.beginTransaction(flir_settings);
    digitalWrite(CS_PIN, LOW);
    //PORTB &= ~(1 << 2);
    //  send in the address and value via SPI:
    lepton_frame_packet[2 * i] = SPI.transfer(0x00);
    lepton_frame_packet[2 * i + 1] = SPI.transfer(0x00);

    // take the SS pin high to de-select the chip:
    digitalWrite(10, HIGH);
    SPI.endTransaction();

    //PORTB |= 1 << 2;
  }
}

void lepton_sync(void) {
  int data = 0x0f;

  SPI.beginTransaction(flir_settings);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  //PORTB |= 1 << 2;
  delay(185);
  while ((data & 0x0f) == 0x0f) {
    SPI.beginTransaction(flir_settings);
    digitalWrite(CS_PIN, LOW);
    //PORTB &= ~(1 << 2);
    data = SPI.transfer(0x00) << 8;
    data |= SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    //PORTB |= 1 << 2;

    for (int i = 0; i < ((VOSPI_FRAME_SIZE - 2) / 2); i++) {

      SPI.beginTransaction(flir_settings);
      digitalWrite(CS_PIN, LOW);
      //PORTB &= ~(1 << 2);

      SPI.transfer(0x00);
      SPI.transfer(0x00);

      digitalWrite(CS_PIN, HIGH);
      SPI.endTransaction();
      //PORTB |= 1 << 2;
    }
  }

}

// Not for readability, rather for parseability.
void print_lepton_frame(void) {
  Serial.print("S");
  for (int i = 0; i < (VOSPI_FRAME_SIZE); i++) {
    Serial.print(lepton_frame_packet[i], HEX);
    //Serial.print(",");
  }
  Serial.print("X");
}

void print_image(void) {
  for (int i = 0; i < (IMAGE_SIZE); i++) {
    Serial.print(image[i], HEX);
    Serial.print(",");
  }
  Serial.println(" ");
}

void lepton_command(unsigned int moduleID, unsigned int commandID,
                    unsigned int command) {
  byte error;
  Wire.beginTransmission(ADDRESS);

  // Command Register is a 16-bit register located at Register Address 0x0004
  Wire.write(0x00);
  Wire.write(0x04);

  //OEM module ID
  if (moduleID == 0x08) {
    Wire.write(0x48);
  } else {
    Wire.write(moduleID & 0x0f);
  }
  Wire.write( ((commandID << 2 ) & 0xfc) | (command & 0x3));

  error = Wire.endTransmission();    // stop transmitting
  if (error <= 4) {
    Serial.print("error=");
    Serial.println(error);
  }
}

void agc_enable() {
  byte error;
  Wire.beginTransmission(ADDRESS); // transmit to device #4
  Wire.write(0x01);
  Wire.write(0x05);
  Wire.write(0x00);
  Wire.write(0x01);

  error = Wire.endTransmission();    // stop transmitting
  if (error) {
    Serial.print("error=");
    Serial.println(error);
  }
}

void set_reg(unsigned int reg)
{
  byte error;
  Wire.beginTransmission(ADDRESS);   // transmit to device #4
  Wire.write(reg >> 8 & 0xff);
  Wire.write(reg & 0xff);            // sends one byte

  error = Wire.endTransmission();    // stop transmitting
  if (error) {
    Serial.print("error=");
    Serial.println(error);
  }
}

//Status reg 15:8 Error Code  7:3 Reserved 2:Boot Status 1:Boot Mode 0:busy

int read_reg(unsigned int reg)
{
  int reading = 0;
  set_reg(reg);

  Wire.requestFrom(ADDRESS, 2);

  reading = Wire.read();  // receive high byte (overwrites previous reading)
  //Serial.println(reading);
  reading = reading << 8;    // shift high byte to be high 8 bits

  reading |= Wire.read(); // receive low byte as lower 8 bits
  Serial.print("reg:");
  Serial.print(reg);
  Serial.print("==0x");
  Serial.print(reading, HEX);
  Serial.print(" binary:");
  Serial.println(reading, BIN);
  return reading;
}

void read_data() {
  while (read_reg(0x2) & 0x01) {
    Serial.println("busy");
    delay(1000);
  }

  int payload_length = read_reg(0x6);
  Serial.print("payload_length=");
  Serial.println(payload_length);

  Wire.requestFrom(ADDRESS, payload_length);
  //set_reg(0x08);
  for (int i = 0; i < (payload_length / 2); i++) {
    int data = Wire.read() << 8;
    data |= Wire.read();
    Serial.println(data, HEX);
  }
}


void loop() {
  //int i;
  //int reading = 0;
  String debugString;

  // Just wait for the user to hit enter.
  Serial.println("Press enter to continue.");
  while(Serial.available() == 0);
  Serial.read();

  Serial.println("beginTransmission");

  /*
  while(1) {
    scan_for_i2c();
    delay(5000);
  }

  set_reg(0);

  read_reg(0x0);

  while(1);

  read_reg(0x2);

  Serial.println("SYS Camera Customer Serial Number");
  lepton_command(SYS, 0x28 >> 2 , GET);
  read_data();

  Serial.println("SYS Flir Serial Number");
  lepton_command(SYS, 0x2 , GET);
  read_data();

  Serial.println("SYS Camera Uptime");
  lepton_command(SYS, 0x0C >> 2 , GET);
  read_data();

  Serial.println("SYS Fpa Temperature Kelvin");
  lepton_command(SYS, 0x14 >> 2 , GET);
  read_data();

  Serial.println("SYS Aux Temperature Kelvin");
  lepton_command(SYS, 0x10 >> 2 , GET);
  read_data();

  Serial.println("OEM Chip Mask Revision");
  lepton_command(OEM, 0x14 >> 2 , GET);
  read_data();

  //Serial.println("OEM Part Number");
  //lepton_command(OEM, 0x1C >> 2 , GET);
  //read_data();

  Serial.println("OEM Camera Software Revision");
  lepton_command(OEM, 0x20 >> 2 , GET);
  read_data();

  Serial.println("AGC Enable");
  //lepton_command(AGC, 0x01  , SET);
  agc_enable();
  read_data();

  Serial.println("AGC READ");
  lepton_command(AGC, 0x00  , GET);
  read_data();

  // Serial.println("SYS Telemetry Enable State");
  //lepton_command(SYS, 0x19>>2 ,GET);
  // read_data();
  */

  while (1) {
    //lepton_sync();
    read_lepton_frame();
    print_lepton_frame();
    Serial.println();
    delay(5000);
    /*
    if(lepton_frame_packet[i]&0x0f != 0x0f ) {
      print_lepton_frame();
    }
    */
  }

  x++;
  delay(10000);
}

