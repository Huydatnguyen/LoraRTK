
/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 04/04/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors. 
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a program that demonstrates the detailed setup of a LoRa test receiver.
  The program listens for incoming packets using the LoRa settings in the 'Settings.h' file. The pins 
  to access the lora device need to be defined in the 'Settings.h' file also.

  There is a printout on the Arduino IDE Serial Monitor of the valid packets received, the packet is 
  assumed to be in ASCII printable text, if it's not ASCII text characters from 0x20 to 0x7F, expect 
  weird things to happen on the Serial Monitor. The LED will flash for each packet received and the 
  buzzer will sound, if fitted.

  Sample serial monitor output;

  7s  Hello World 1234567890*,CRC,DAAB,RSSI,-52dBm,SNR,9dB,Length,23,Packets,5,Errors,0,IRQreg,50

  If there is a packet error it might look like this, which is showing a CRC error,

  968s PacketError,RSSI,-87dBm,SNR,-11dB,Length,23,Packets,613,Errors,2,IRQreg,70,IRQ_HEADER_VALID,IRQ_CRC_ERROR,IRQ_RX_DONE
  
  Serial monitor baud rate is set at 9600.
*******************************************************************************************************/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include <sfe_bus.h>
#include <u-blox_Class_and_ID.h>
#include <u-blox_GNSS.h>
#include <u-blox_config_keys.h>
#include <u-blox_external_typedefs.h>
#include <u-blox_structs.h>

#include <SPI.h>                                               //the lora device is SPI based so load the SPI library                                         
#include <SX128XLT.h>                                          //include the appropriate library  
#include "Settings.h"                                          //include the setiings file, frequencies, LoRa settings etc   

#include <HardwareSerial.h>


const char* ssid = "Dat Gunner";
const char* password = "12345678";

SFE_UBLOX_GNSS_SERIAL myGNSS;
HardwareSerial mySP(2); // use UART2

SX128XLT LT;                                     //create a library class instance called LT

//#define ENABLE_UPLOAD_OTA // comment this line for disabling OTA upload code
#define mySerial Serial2 // Use Serial2 to connect to the GNSS module. Change this if required


uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                 //create the buffer that received packets are copied into

uint8_t RXPacketL;                               //stores length of packet received
int16_t PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio (SNR) of received packet

void setup()
{
  //***************************************************************************************************
  //Setup Serial for testing
  //***************************************************************************************************
  Serial.begin(9600);


 /* _________________set up Wifi connection for OTA update ____________________ */  
#ifdef ENABLE_UPLOAD_OTA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Wifi config Ready");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
#endif
  /*_____________________________________________________________*/


  // SPI initialization
  SPI.begin();

  //SPI beginTranscation is normally part of library routines, but if it is disabled in library
  //a single instance is needed here, so uncomment the program line below
  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found
   if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
    }
  }

  //The function call list below shows the complete setup for the LoRa device using the information defined in the
  //Settings.h file.
  //The 'Setup LoRa device' list below can be replaced with a single function call;
  //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate); 
  //***************************************************************************************************
  //Setup LoRa device
  //***************************************************************************************************
  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();


  //***************************************************************************************************
  //Setup RTK module
  //***************************************************************************************************
  mySerial.begin(115200); // u-blox F9 and M10 modules default to 38400 baud. Change this if required

  myGNSS.connectedToUART2(); // This tells the library we are connecting to UART2 so it uses the correct configuration keys

  while (myGNSS.begin(mySerial) == false) //Connect to the u-blox module using mySerial (defined above)
  {
    Serial.println(F("u-blox GNSS not detected"));
    
    Serial.println(F("Attempting to enable the UBX protocol for output"));
    
    myGNSS.setUART2Output(COM_TYPE_UBX); // Enable UBX output. Disable NMEA output
    
    Serial.println(F("Retrying..."));
    delay (1000);
  }
  //***************************************************************************************************

  Serial.print(F("Rover ready"));
  Serial.println();
  //***************************************************************************************************

}

void loop()
{
// for uploading code OTA  
#ifdef ENABLE_UPLOAD_OTA  
  ArduinoOTA.handle();
#endif

// receiving Lora packet
  RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 60000, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout

  PacketRSSI = LT.readPacketRSSI();              //read the recived RSSI value
  PacketSNR = LT.readPacketSNR();                //read the received SNR value

  if (RXPacketL == 0)                            //if the LT.receive() function detects an error, RXpacketL is 0
  {
    packet_is_Error();
  }
  else
  {
    packet_is_OK();
  }
}


void packet_is_OK()
{
  uint16_t IRQStatus, localCRC;

  IRQStatus = LT.readIrqStatus();                 //read the LoRa device IRQ status register


  printf("Receiving RTCM msg over Lora______________________ \n");
  for(uint8_t i = 0; i < RXPacketL; i++)
  {
    printf("%02X ", RXBUFFER[i]);
  }
  printf("\n");

  // sending RTCM sentence to ZED F9P 
  myGNSS.pushRawData(RXBUFFER, RXPacketL, false);
  
}


void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register

  printElapsedTime();                               //print elapsed time to Serial Monitor

  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
  }
  else
  {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LT.readRXPacketL());               //get the device packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();                            //print the names of the IRQ registers set
  }

  delay(250);                                       //gives a longer buzzer and LED flash for error 
  
}

void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}
