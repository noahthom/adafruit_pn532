
#include "Arduino.h"

#include <Wire.h>
#ifdef __SAM3X8E__ // arduino due
#define WIRE Wire1
#else
#define WIRE Wire
#endif

#include <SPI.h>

#include "Adafruit_PN532_noah.h"

byte pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
byte pn532response_firmwarevers[] = {0x00, 0x00, 0xFF, 0x06, 0xFA, 0xD5};

// Uncomment these lines to enable debug output for PN532(SPI) and/or MIFARE
// related code

// #define PN532DEBUG
// #define MIFAREDEBUG

// If using Native Port on Arduino Zero or Due define as SerialUSB
#define PN532DEBUGPRINT Serial
//#define PN532DEBUGPRINT SerialUSB

#define PN532_PACKBUFFSIZ 64
byte pn532_packetbuffer[PN532_PACKBUFFSIZ];

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

/**************************************************************************/
/*!
    @brief  Sends a single byte via I2C

    @param  x    The byte to send
*/
/**************************************************************************/
static inline void i2c_send(uint8_t x) {
#if ARDUINO >= 100
  WIRE.write((uint8_t)x);
#else
  WIRE.send(x);
#endif
}

/**************************************************************************/
/*!
    @brief  Reads a single byte via I2C
*/
/**************************************************************************/
static inline uint8_t i2c_recv(void) {
#if ARDUINO >= 100
  return WIRE.read();
#else
  return WIRE.receive();
#endif
}

/**************************************************************************/
/*!
    @brief  Instantiates a new PN532 class using software SPI.

    @param  clk       SPI clock pin (SCK)
    @param  miso      SPI MISO pin
    @param  mosi      SPI MOSI pin
    @param  ss        SPI chip select pin (CS/SSEL)
*/
/**************************************************************************/
Adafruit_PN532::Adafruit_PN532(uint8_t clk, uint8_t miso, uint8_t mosi,
                               uint8_t ss) {
  spi_dev = new Adafruit_SPIDevice(ss, clk, miso, mosi, 1000000,
                                   SPI_BITORDER_LSBFIRST, SPI_MODE0);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new PN532 class using I2C.

    @param  irq       Location of the IRQ pin
    @param  reset     Location of the RSTPD_N pin
*/
/**************************************************************************/
Adafruit_PN532::Adafruit_PN532(uint8_t irq, uint8_t reset)
    : _irq(irq), _reset(reset) {
  pinMode(_irq, INPUT);
  pinMode(_reset, OUTPUT);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new PN532 class using hardware SPI.

    @param  ss        SPI chip select pin (CS/SSEL)
*/
/**************************************************************************/
Adafruit_PN532::Adafruit_PN532(uint8_t ss) {
  spi_dev =
      new Adafruit_SPIDevice(ss, 1000000, SPI_BITORDER_LSBFIRST, SPI_MODE0);
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
void Adafruit_PN532::begin() {
  if (spi_dev != NULL) {
    // SPI initialization
    spi_dev->begin();

    // not exactly sure why but we have to send a dummy command to get synced up
    pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
    sendCommandCheckAck(pn532_packetbuffer, 1);
    // ignore response!
  } else {
    // I2C initialization.
    WIRE.begin();

    // Reset the PN532
    digitalWrite(_reset, HIGH);
    digitalWrite(_reset, LOW);
    delay(400);
    digitalWrite(_reset, HIGH);
    delay(
        10); // Small delay required before taking other actions after reset.
             // See timing diagram on page 209 of the datasheet, section 12.23.
  }
}

/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters

    @param  data      Pointer to the byte data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void Adafruit_PN532::PrintHex(const byte *data, const uint32_t numBytes) {
  uint32_t szPos;
  for (szPos = 0; szPos < numBytes; szPos++) {
    PN532DEBUGPRINT.print(F("0x"));
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
      PN532DEBUGPRINT.print(F("0"));
    PN532DEBUGPRINT.print(data[szPos] & 0xff, HEX);
    if ((numBytes > 1) && (szPos != numBytes - 1)) {
      PN532DEBUGPRINT.print(F(" "));
    }
  }
  PN532DEBUGPRINT.println();
}

/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters, along with
            the char equivalents in the following format

            00 00 00 00 00 00  ......

    @param  data      Pointer to the byte data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void Adafruit_PN532::PrintHexChar(const byte *data, const uint32_t numBytes) {
  uint32_t szPos;
  for (szPos = 0; szPos < numBytes; szPos++) {
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
      PN532DEBUGPRINT.print(F("0"));
    PN532DEBUGPRINT.print(data[szPos], HEX);
    if ((numBytes > 1) && (szPos != numBytes - 1)) {
      PN532DEBUGPRINT.print(F(" "));
    }
  }
  PN532DEBUGPRINT.print(F("  "));
  for (szPos = 0; szPos < numBytes; szPos++) {
    if (data[szPos] <= 0x1F)
      PN532DEBUGPRINT.print(F("."));
    else
      PN532DEBUGPRINT.print((char)data[szPos]);
  }
  PN532DEBUGPRINT.println();
}

/**************************************************************************/
/*!
    @brief  Checks the firmware version of the PN5xx chip

    @returns  The chip's firmware version and ID
*/
/**************************************************************************/
uint32_t Adafruit_PN532::getFirmwareVersion(void) {
  uint32_t response;

  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

  if (!sendCommandCheckAck(pn532_packetbuffer, 1)) {
    return 0;
  }

  // read data packet
  readdata(pn532_packetbuffer, 12);

  // check some basic stuff
  if (0 != memcmp((char *)pn532_packetbuffer,
                  (char *)pn532response_firmwarevers, 6)) {
#ifdef PN532DEBUG
    PN532DEBUGPRINT.println(F("Firmware doesn't match!"));
#endif
    return 0;
  }

  int offset = 7;
  response = pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];

  return response;
}

/**************************************************************************/
/*!
    @brief  Sends a command and waits a specified period for the ACK

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    The size of the command in bytes
    @param  timeout   timeout before giving up

    @returns  1 if everything is OK, 0 if timeout occured before an
              ACK was recieved
*/
/**************************************************************************/
// default timeout of one second
bool Adafruit_PN532::sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen,
                                         uint16_t timeout) {
  // uint16_t timer = 0;

  // write the command
  writecommand(cmd, cmdlen);

  // Wait for chip to say its ready!
  if (!waitready(timeout)) {
    return false;
  }

#ifdef PN532DEBUG
  if (spi_dev == NULL) {
    PN532DEBUGPRINT.println(F("IRQ received"));
  }
#endif

  // read acknowledgement
  if (!readack()) {
#ifdef PN532DEBUG
    PN532DEBUGPRINT.println(F("No ACK frame received!"));
#endif
    return false;
  }

  // For SPI only wait for the chip to be ready again.
  // This is unnecessary with I2C.
  if (spi_dev != NULL) {
    if (!waitready(timeout)) {
      return false;
    }
  }

  return true; // ack'd command
}



/**************************************************************************/
/*!
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
bool Adafruit_PN532::SAMConfig(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (!sendCommandCheckAck(pn532_packetbuffer, 4))
    return false;

  // read data packet
  readdata(pn532_packetbuffer, 8);

  int offset = 6;
  return (pn532_packetbuffer[offset] == 0x15);
}


/*****************************************************************************/
/*!
	@brief  Configure PN532 as Activator for Target.
	@param  NONE.
	@return 0 - failed
            1 - successfully
*/
/*****************************************************************************/
bool Adafruit_PN532::P2PInitiator_Init()
{
  static uint8_t send_flag = 1;
  pn532_packetbuffer[0] = PN532_COMMAND_INJUMPFORDEP;
  pn532_packetbuffer[1] = 0x01; //active mode
  pn532_packetbuffer[2] = 0x02; //424 kbps
  pn532_packetbuffer[3] = 0x01;
  pn532_packetbuffer[4] = 0x00;
  pn532_packetbuffer[5] = 0xFF;
  pn532_packetbuffer[6] = 0xFF;
  pn532_packetbuffer[7] = 0x00;
  pn532_packetbuffer[8] = 0x00;

  if(send_flag)
  {
    send_flag = 0;
    if(!sendCommandCheckAck(pn532_packetbuffer, 9))
    {
      Serial.println("InJumpForDEP failed, no ACK \n");
      return false;
    }
    //Serial.println("InJumpForDEP sent \n");    
  }

  waitready(10);

  readdata(pn532_packetbuffer, 25);
  if(pn532_packetbuffer[5] != 0xD5)
  {
    return false;
  }

  if(pn532_packetbuffer[6] != (PN532_COMMAND_INJUMPFORDEP+1))
  {
    Serial.println("InJumpForDEP Failed \n");
    return false;
  }
  if(pn532_packetbuffer[7])
  {
    Serial.println("Time out \n");
    return false;
  }

  //Serial.println("InJumpForDEP tgt response read");

  send_flag = 0;
  return true; 
}

/*****************************************************************************/
/*!
	@brief  Configure PN532 as Target.
	@param  NONE.
	@return 0 - failed
            1 - successfully
*/
/*****************************************************************************/
bool Adafruit_PN532::P2PTarget_Init()
{
    static uint8_t send_flag = 1;
    pn532_packetbuffer[0] = PN532_COMMAND_TGINITASTARGET;
    /** 14443-4A Card only */
    pn532_packetbuffer[1] = 0x00;

    /** SENS_RES */
    pn532_packetbuffer[2] = 0x04;
    pn532_packetbuffer[3] = 0x00;

    /** NFCID1 */
    pn532_packetbuffer[4] = 0x12;
    pn532_packetbuffer[5] = 0x34;
    pn532_packetbuffer[6] = 0x56;

    /** SEL_RES */
    pn532_packetbuffer[7] = 0x40;      // DEP only mode

    /**Parameters to build POL_RES (18 bytes including system code) */
    pn532_packetbuffer[8] = 0x01;
    pn532_packetbuffer[9] = 0xFE;
    pn532_packetbuffer[10] = 0xA2;
    pn532_packetbuffer[11] = 0xA3;
    pn532_packetbuffer[12] = 0xA4;
    pn532_packetbuffer[13] = 0xA5;
    pn532_packetbuffer[14] = 0xA6;
    pn532_packetbuffer[15] = 0xA7;
    pn532_packetbuffer[16] = 0xC0;
    pn532_packetbuffer[17] = 0xC1;
    pn532_packetbuffer[18] = 0xC2;
    pn532_packetbuffer[19] = 0xC3;
    pn532_packetbuffer[20] = 0xC4;
    pn532_packetbuffer[21] = 0xC5;
    pn532_packetbuffer[22] = 0xC6;
    pn532_packetbuffer[23] = 0xC7;
    pn532_packetbuffer[24] = 0xFF;
    pn532_packetbuffer[25] = 0xFF;
    /** NFCID3t */
    pn532_packetbuffer[26] = 0xAA;
    pn532_packetbuffer[27] = 0x99;
    pn532_packetbuffer[28] = 0x88;
    pn532_packetbuffer[29] = 0x77;
    pn532_packetbuffer[30] = 0x66;
    pn532_packetbuffer[31] = 0x55;
    pn532_packetbuffer[32] = 0x44;
    pn532_packetbuffer[33] = 0x33;
    pn532_packetbuffer[34] = 0x22;
    pn532_packetbuffer[35] = 0x11;
    /** Length of general bytes  */
    pn532_packetbuffer[36] = 0x00;
    /** Length of historical bytes  */
    pn532_packetbuffer[37] = 0x00;


    if(send_flag)
    {
      send_flag = 0;
      if(!sendCommandCheckAck(pn532_packetbuffer, 38))
      {
        Serial.println("TgInitAsTarget failed, No ACK \n"); 
        return false;
      }
      //Serial.println("TgInitAsTarget sent \n");
    }
    
    waitready(10);
    readdata(pn532_packetbuffer, 24);
    if(pn532_packetbuffer[5] != 0xD5)
    {
      return false;
    }

    if(pn532_packetbuffer[6] != (PN532_COMMAND_TGINITASTARGET+1))
    {
      Serial.println("TgInitasTarget Failed \n"); 
      return false;
    }
    send_flag = 1;
    //Serial.println(F("TgInitasTarget Successful \n")); 
    return true;


}
/*****************************************************************************/
/*!
	@brief  Configure PN532 to send and recieve data as P2P Initiator
	@param  NONE.
	@return 0 - failed
            1 - successfully
*/
/*****************************************************************************/
bool Adafruit_PN532::P2PInitiator_TxRx(uint8_t *t_buffer, uint8_t t_length, uint8_t *r_buffer, uint8_t *r_length)
{

  waitready(15);
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 0x01; // logical number of the relevant target

  memcpy(pn532_packetbuffer+2, t_buffer, t_length);

  if(!sendCommandCheckAck(pn532_packetbuffer, t_length+2))
  {
    return false;
  }
  //Serial.println("Initiator DataExchange sent.");

  waitready(200);

  readdata(pn532_packetbuffer, 60);
  if(pn532_packetbuffer[5] != 0xD5)
  {
   return 0;
  }
  //Serial.println("Initiator DataExchange Get.");

  if(pn532_packetbuffer[6] != (PN532_COMMAND_INDATAEXCHANGE+1))
  {
    PrintHex(pn532_packetbuffer, pn532_packetbuffer[3]+7);
    Serial.println("Send data failed");
    return 0;
  }

  if(pn532_packetbuffer[7])
  {
    Serial.print("InExchangeData Error:");
    PrintHex(pn532_packetbuffer, pn532_packetbuffer[3]+7);
    Serial.println();
    return 0;
  }


  //PrintHex(pn532_packetbuffer, pn532_packetbuffer[3]+7);
  Serial.println();
    /** return read data */
  *r_length = pn532_packetbuffer[3]-3;
  memcpy(r_buffer, pn532_packetbuffer+8, *r_length);
  return 1;

}
/*****************************************************************************/
/*!
	@brief  Target sends and recievs data.
    @param  t_buffer --- data send buffer, user sets
            t_length --- data send legth, user sets.
            r_buffer --- data recieve buffer, returned by P2PInitiatorTxRx
            r_length --- data receive length, returned by P2PInitiatorTxRx
	@return 0 - send failed
            1 - send successfully
*/
/*****************************************************************************/
bool Adafruit_PN532::P2PTarget_TxRx(uint8_t *t_buffer, uint8_t t_length, uint8_t *r_buffer, uint8_t *r_length)
{
  pn532_packetbuffer[0] = PN532_COMMAND_TGGETDATA;
  if(!sendCommandCheckAck(pn532_packetbuffer, 1))
  {
    return 0;
  }
  waitready(100);
  readdata(pn532_packetbuffer, 60);
  if(pn532_packetbuffer[5] != 0xD5)
  {
    return 0;
  }

  if(pn532_packetbuffer[6] != (PN532_COMMAND_TGGETDATA+1))
  {
    PrintHex(pn532_packetbuffer, 20);
    Serial.println("Target GetData failed");
    return 0;
  }
  if(pn532_packetbuffer[7])
  {
    Serial.print("TgGetData Error:");
    PrintHex(pn532_packetbuffer, pn532_packetbuffer[3]+7);
    Serial.println();
    return 0;
  }
  //Serial.println("TgGetData:");
  //PrintHex(pn532_packetbuffer, pn532_packetbuffer[3]+7);
  Serial.println();

  /** return read data */
  *r_length = pn532_packetbuffer[3]-3;
  memcpy(r_buffer, pn532_packetbuffer+8, *r_length);

  pn532_packetbuffer[0] = PN532_COMMAND_TGSETDATA;
  memcpy(pn532_packetbuffer+1, t_buffer, t_length);

  if(!sendCommandCheckAck(pn532_packetbuffer, 1+t_length))
  {
    return 0;
  }
  waitready(100);
  readdata(pn532_packetbuffer, 26);

  if(pn532_packetbuffer[5] != 0xD5)
  {
    return 0;
  }
  if(pn532_packetbuffer[6] != (PN532_COMMAND_TGSETDATA+1))
  {
    PrintHex(pn532_packetbuffer, 20);
    Serial.println("Send data failed");
    return 0;
  }
  if(pn532_packetbuffer[7])
  {
    return 0;
  }

return 1;
}








/************** high level communication functions (handles both I2C and SPI) */

/**************************************************************************/
/*!
    @brief  Tries to read the SPI or I2C ACK signal
*/
/**************************************************************************/
bool Adafruit_PN532::readack() {
  uint8_t ackbuff[6];

  if (spi_dev) {
    uint8_t cmd = PN532_SPI_DATAREAD;
    spi_dev->write_then_read(&cmd, 1, ackbuff, 6);
  } else {
    readdata(ackbuff, 6);
  }

  return (0 == memcmp((char *)ackbuff, (char *)pn532ack, 6));
}

/**************************************************************************/
/*!
    @brief  Return true if the PN532 is ready with a response.
*/
/**************************************************************************/
bool Adafruit_PN532::isready() {
  if (spi_dev != NULL) {
    uint8_t cmd = PN532_SPI_STATREAD;
    uint8_t reply;
    spi_dev->write_then_read(&cmd, 1, &reply, 1);
    // Check if status is ready.
    // Serial.print("Ready? 0x"); Serial.println(reply, HEX);
    return reply == PN532_SPI_READY;
  } else {
    // I2C check if status is ready by IRQ line being pulled low.
    uint8_t x = digitalRead(_irq);
    return x == 0;
  }
}

/**************************************************************************/
/*!
    @brief  Waits until the PN532 is ready.

    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
bool Adafruit_PN532::waitready(uint16_t timeout) {
  uint16_t timer = 0;
  while (!isready()) {
    if (timeout != 0) {
      timer += 10;
      if (timer > timeout) {
#ifdef PN532DEBUG
        PN532DEBUGPRINT.println("TIMEOUT!");
#endif
        return false;
      }
    }
    delay(10);
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads n bytes of data from the PN532 via SPI or I2C.

    @param  buff      Pointer to the buffer where data will be written
    @param  n         Number of bytes to be read
*/
/**************************************************************************/
void Adafruit_PN532::readdata(uint8_t *buff, uint8_t n) {
  if (spi_dev) {
    uint8_t cmd = PN532_SPI_DATAREAD;

    spi_dev->write_then_read(&cmd, 1, buff, n);

#ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("Reading: "));
    for (uint8_t i = 0; i < n; i++) {
      PN532DEBUGPRINT.print(F(" 0x"));
      PN532DEBUGPRINT.print(buff[i], HEX);
    }
    PN532DEBUGPRINT.println();
#endif
  } else {
    // I2C write.
    uint16_t timer = 0;

    delay(2);

#ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("Reading: "));
#endif
    // Start read (n+1 to take into account leading 0x01 with I2C)
    WIRE.requestFrom((uint8_t)PN532_I2C_ADDRESS, (uint8_t)(n + 2));
    // Discard the leading 0x01
    i2c_recv();
    for (uint8_t i = 0; i < n; i++) {
      delay(1);
      buff[i] = i2c_recv();
#ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F(" 0x"));
      PN532DEBUGPRINT.print(buff[i], HEX);
#endif
    }
    // Discard trailing 0x00 0x00
    // i2c_recv();

#ifdef PN532DEBUG
    PN532DEBUGPRINT.println();
#endif
  }
}



/**************************************************************************/
/*!
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in bytes
*/
/**************************************************************************/
void Adafruit_PN532::writecommand(uint8_t *cmd, uint8_t cmdlen) {
  if (spi_dev != NULL) {
    // SPI command write.
    uint8_t checksum;
    uint8_t packet[8 + cmdlen];
    uint8_t *p = packet;
    cmdlen++;

    p[0] = PN532_SPI_DATAWRITE;
    p++;

    p[0] = PN532_PREAMBLE;
    p++;
    p[0] = PN532_STARTCODE1;
    p++;
    p[0] = PN532_STARTCODE2;
    p++;
    checksum = PN532_PREAMBLE + PN532_STARTCODE1 + PN532_STARTCODE2;

    p[0] = cmdlen;
    p++;
    p[0] = ~cmdlen + 1;
    p++;

    p[0] = PN532_HOSTTOPN532;
    p++;
    checksum += PN532_HOSTTOPN532;

    for (uint8_t i = 0; i < cmdlen - 1; i++) {
      p[0] = cmd[i];
      p++;
      checksum += cmd[i];
    }

    p[0] = ~checksum;
    p++;
    p[0] = PN532_POSTAMBLE;
    p++;

#ifdef PN532DEBUG
    Serial.print("Sending : ");
    for (int i = 1; i < 8 + cmdlen; i++) {
      Serial.print("0x");
      Serial.print(packet[i], HEX);
      Serial.print(", ");
    }
    Serial.println();
#endif

    spi_dev->write(packet, 8 + cmdlen);
  } else {
    // I2C command write.
    uint8_t checksum;

    cmdlen++;

#ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("\nSending: "));
#endif

    delay(2); // or whatever the delay is for waking up the board

    // I2C START
    WIRE.beginTransmission(PN532_I2C_ADDRESS);
    checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
    i2c_send(PN532_PREAMBLE);
    i2c_send(PN532_PREAMBLE);
    i2c_send(PN532_STARTCODE2);

    i2c_send(cmdlen);
    i2c_send(~cmdlen + 1);

    i2c_send(PN532_HOSTTOPN532);
    checksum += PN532_HOSTTOPN532;

#ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print((byte)PN532_PREAMBLE, HEX);
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print((byte)PN532_PREAMBLE, HEX);
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print((byte)PN532_STARTCODE2, HEX);
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print((byte)cmdlen, HEX);
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print((byte)(~cmdlen + 1), HEX);
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print((byte)PN532_HOSTTOPN532, HEX);
#endif

    for (uint8_t i = 0; i < cmdlen - 1; i++) {
      i2c_send(cmd[i]);
      checksum += cmd[i];
#ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F(" 0x"));
      PN532DEBUGPRINT.print((byte)cmd[i], HEX);
#endif
    }

    i2c_send((byte)~checksum);
    i2c_send((byte)PN532_POSTAMBLE);

    // I2C STOP
    WIRE.endTransmission();

#ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print((byte)~checksum, HEX);
    PN532DEBUGPRINT.print(F(" 0x"));
    PN532DEBUGPRINT.print((byte)PN532_POSTAMBLE, HEX);
    PN532DEBUGPRINT.println();
#endif
  }
}
