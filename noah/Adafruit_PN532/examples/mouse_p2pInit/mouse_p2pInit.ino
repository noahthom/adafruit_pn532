#include <Wire.h>
#include <Adafruit_PN532_noah.h>
#include <SPI.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//these pins can be set to enable SPI using any GPIO pins if ALTERNATE_PINS is defined
#ifdef ALTERNATE_PINS
  #define VSPI_MISO   2
  #define VSPI_MOSI   4
  #define VSPI_SCLK   0
  #define VSPI_SS     33

  #define HSPI_MISO   26
  #define HSPI_MOSI   27
  #define HSPI_SCLK   25
  #define HSPI_SS     32
//otherwise the default VSPI and HSPI pins will be used as shown below
#else
  #define VSPI_MISO   MISO
  #define VSPI_MOSI   MOSI
  #define VSPI_SCLK   SCK
  #define VSPI_SS     SS

  #define HSPI_MISO   12
  #define HSPI_MOSI   13
  #define HSPI_SCLK   14
  #define HSPI_SS     15
#endif

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines. Can be any GPIO pin 
#define PN532_IRQ   (19)
#define PN532_RESET (23) 

//uncomment this line to use non-default SPI pins
// #define ALTERNATE_PINS

//defining macro for distance at which mouse is at platform
#define GOOD_DISTANCE 10

static const int spiClk = 1000000; // 1 MHz SPI speed

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

//defining nfc class with i2c connection
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);


typedef struct    //defining data_t structure for demonstration purposes
{
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
}data_t;

data_t platform; //defining object platform of type data_t

typedef struct   //defining solar_t structure for demonstration purposes
{
  uint8_t volt;
  uint8_t current;
  uint8_t soc;
}solar_t;      

solar_t panel;   //defining object panel of type solar_t


uint8_t platform_tx[50] = "Hello this is the MOUSE platform"; //buffer which contains data to send; if not needed, initialise as empty variable; array size can be edited
uint8_t platform_txLen;       //length of platform_tx in bytes
uint8_t platform_rx[50];      //buffer which will contain data received; array size can be edited
uint8_t platform_rxLen;       //size of platform_rx in bytes

TaskHandle_t xDistanceTaskHandle = NULL;    //handle of vTaskDistance task
TaskHandle_t xP2PTaskHandle = NULL;         //handle of vTaskP2P task


void setup(void) 
{
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);

  #ifndef ALTERNATE_PINS
  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
  #else
    //alternatively route through GPIO pins of your choice
    vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); //SCLK, MISO, MOSI, SS
  #endif

  #ifndef ALTERNATE_PINS
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin();
  #else
    //alternatively route through GPIO pins
    hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
  #endif

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(VSPI_SS, OUTPUT); //VSPI SS
  pinMode(HSPI_SS, OUTPUT); //HSPI SS

  Serial.begin(115200); //initialising serial link at 115200 bps
  
  nfc.begin();          //initialising nfc object
  uint32_t versiondata = nfc.getFirmwareVersion();                                   /*------------------------------------------------------------------------------------*/
  if (! versiondata) {                                                               /*------------------------------------------------------------------------------------*/
    Serial.print("Didn't find PN53x board");                                         /*----------------------THIS SECTION MERELY RECEIVES FIRMWARE VERSION-----------------*/
    while (1); // halt                                                               /*----------------------TO CONFIRM CONNECTION OF PN532 BOARD TO ESP32-----------------*/
  }                                                                                  /*------------------------------CAN BE REMOVED IF DESIRED-----------------------------*/
  // Got ok data, print it out!                                                      /*------------------------------------------------------------------------------------*/
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);     /*------------------------------------------------------------------------------------*/
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);       /*------------------------------------------------------------------------------------*/
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);                   /*------------------------------------------------------------------------------------*/
  nfc.SAMConfig();   //configuring Secure Access Module of PN532 

  //Creating two independent tasks
   xTaskCreatePinnedToCore(
    vTaskP2P
    ,  "TaskP2P"                // A name just for debugging
    ,  1024                     // 1024 byte stack
    ,  NULL
    ,  1                        // Priority level 1
    ,  &xP2PTaskHandle          // address of handle xP2PTaskHandle
    ,  ARDUINO_RUNNING_CORE);   // core on which task will run

   xTaskCreatePinnedToCore(
    vTaskDistance
    ,  "TaskDistance"           // A name just for debugging
    ,  1024                     // 1024 byte stack
    ,  NULL
    ,  2                        // Priority level 2
    ,  &xDistanceTaskHandle     // address of handle xDistanceTaskHandle
    ,  ARDUINO_RUNNING_CORE);   // core on which task will run
  
}


void loop(void) 
{
  //nothing happens
}




/*-----------------------------*/
/*-----------P2P Task ---------*/
/*-----------------------------*/

void vTaskP2P(void *pvParameters)
{
  (void) pvParameters;
  
  while(1)
  {
      if(nfc.P2PInitiator_Init()) // Initialising PN532 Initiator mode
      {
        Serial.println("Target is sensed.");
        platform_txLen = strlen((const char*)platform_tx); //calculating size of platform_tx buffer, if not a string message, use sizeof(platform_tx)
        //platform_txLen = sizeof(platform_tx)
        if(nfc.P2PInitiator_TxRx(platform_tx, platform_txLen, platform_rx, &platform_rxLen)) //begin data transfer between Initiator and Target
        {
          platform.data1 = platform_rx[0];    //storing received data from buffer to platform object members
          platform.data2 = platform_rx[1];
          platform.data3 = platform_rx[2];
          Serial.println("Data has been received"); //printing data to serial monitor
          Serial.println(platform.data1);
          Serial.println(platform.data2);
          Serial.println(platform.data3);
          RaspberryPi_Tx(platform.data1);           //sending data to Raspberry Pi
          RaspberryPi_Tx(platform.data2);
          RaspberryPi_Tx(platform.data3);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      vTaskResume(xDistanceTaskHandle);
  }

  
}
/*---------------------------------------*/
/*--------Dummy Ultrasonic task----------*/
/*---------------------------------------*/

void vTaskDistance(void *pvParameters)
{
  (void) pvParameters;
  uint8_t distance;

  while(1)
  {
    distance = Ultrasonic_Rx();
    if(distance == GOOD_DISTANCE)
    {
      vTaskSuspend(NULL); //suspends vTaskDistance
    }
  }
}


void RaspberryPi_Tx(uint8_t data) //uses VSPI
{
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); //needs to be tweaked based on rPI SPI config
  digitalWrite(VSPI_SS, LOW); //pull SS slow to prep other end for transfer
  vspi->transfer(data);  
  digitalWrite(VSPI_SS, HIGH); //pull ss high to signify end of data transfer
  vspi->endTransaction();
}

uint8_t Ultrasonic_Rx() 
{
  uint8_t data;
  uint8_t send_command = 20;
  
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); //needs to be tweaked based on Ultrasonic Arduino SPI config
  digitalWrite(HSPI_SS, LOW);
  data = hspi->transfer(send_command);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();

  return data;
}
