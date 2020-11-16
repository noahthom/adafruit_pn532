#include <Wire.h>
#include <Adafruit_PN532_noah.h>

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ   (19)
#define PN532_RESET (23)  // Not connected by default on the NFC Shield


Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

typedef struct
{
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
}data_t;

data_t sensors;



uint8_t mouse_tx[50];
uint8_t mouse_txLen;
uint8_t mouse_rx[50];
uint8_t mouse_rxLen;

void setup(void) 
{
  mouse_tx[0] = 20;
  mouse_tx[1] = 40;
  mouse_tx[2] = 60;

  
  
  Serial.begin(115200);
  while (!Serial) delay(10); // for Leonardo/Micro/Zero
  Serial.println("Hello!");
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  nfc.SAMConfig();
  
}


void loop(void) 
{
 /** device is configured as Target */
  if(nfc.P2PTarget_Init())
  {
    /**
      send data with a length parameter and receive some data,
      tx_buf --- data send buffer
      tx_len --- data send legth
      rx_buf --- data recieve buffer, return by P2PTargetTxRx
      rx_len --- data receive length, return by P2PTargetTxRx
    */
    Serial.println("Initiator is sensed.");
    mouse_txLen = sizeof(mouse_tx);
    if(nfc.P2PTarget_TxRx(mouse_tx, mouse_txLen, mouse_rx, &mouse_rxLen))
    {
      Serial.print("Data Received: ");
      Serial.write(mouse_rx, mouse_rxLen);
      Serial.println();
    }
    Serial.println();
  }
  
}
