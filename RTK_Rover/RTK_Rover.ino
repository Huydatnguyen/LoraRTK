
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


const char* ssid = "Dat Gunner";
const char* password = "12345678";

SFE_UBLOX_GNSS myGNSS;

#define myWire Wire // Connect using the Wire1 port. Change this if required
#define gnssAddress 0x42 // The default I2C address for u-blox modules is 0x42. Change this if required
//#define ENABLE_UPLOAD_OTA // comment this line for disabling OTA upload code

SX128XLT LT;                                     //create a library class instance called LT


uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                 //create the buffer that received packets are copied into
uint8_t RTCM_FULL_BUFFER[1029];                  // buffer for full size RTCM sentence, can be up to 1029 bytes

uint8_t RXPacketL;                               //stores length of packet received
int16_t PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio (SNR) of received packet

// variables for processing Lora packet received before sending out the RTCM sentences to RTK of rover
bool ready_for_sendingRTCM = false;
uint16_t totalSize_of_RTCM = 0; // max 1029
uint8_t nb_of_packets = 0 ; // nb of total Lora packets 

void setup()
{
  //***************************************************************************************************
  //Setup Serial for testing
  Serial.begin(9600);
  delay(1000);

  //***************************************************************************************************
  //set up Wifi connection for OTA update 
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
  //***************************************************************************************************

  //***************************************************************************************************
  // SPI and Lora initialization
  SPI.begin();

  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println("LoRa Device found");
    delay(1000);
  }
  else
  {
    Serial.println("No device responding");
    while (1)
    {
    }
  }


  //The function call below shows the complete setup for the LoRa device using the information defined in the
  //Settings.h file.
  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();

  //***************************************************************************************************

  //***************************************************************************************************
  //Setup I2C for communicating to RTK module
  myWire.begin(); // Start I2C

  while (myGNSS.begin(myWire, gnssAddress) == false) //Connect to the u-blox module using our custom port and address
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay (1000);
  }

  // myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Ensure RTCM3 is enabled for input
  // myGNSS.setI2COutput(COM_TYPE_UBX); // Ensure UBX is enabled for retrieving the info after correction  
  // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR
  
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

  receiveLoraPacket();
}

void receiveLoraPacket()
{
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
  uint8_t SYN;
  uint8_t nb_of_remaining_packets;

  //IRQStatus = LT.readIrqStatus();                 //read the LoRa device IRQ status register


  // Serial.print("Receiving RTCM msg over Lora______________________ \n");
  // for(uint8_t i = 0; i < RXPacketL; i++)
  // {
  //   Serial.print(RXBUFFER[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  //retrieve the SYN bit and the nb of remaining packets in the last byte of Lora packet
  SYN = RXBUFFER[RXPacketL-1] >> 7;
  nb_of_remaining_packets = (RXBUFFER[RXPacketL-1] >> 4) & B00000111;

  Serial.print("SYN bit:  ");
  Serial.print(SYN);
  Serial.print("         remaining :  ");
  Serial.print(nb_of_remaining_packets);
  Serial.println();

  if(SYN == 1) //the first packet
  {
    if(nb_of_remaining_packets == 0) // the first byte is also the last byte
    {
      // copy RXBUFFER from Lora to RTCM_FULL_BUFFER for sending to RTK module of rover
      memcpy(RTCM_FULL_BUFFER, RXBUFFER, (RXPacketL-1) * sizeof(uint8_t));

      ready_for_sendingRTCM = true;
      totalSize_of_RTCM = RXPacketL-1;    
    }
    else
    {
      nb_of_packets = nb_of_remaining_packets + 1;

      // copy RXBUFFER from Lora to RTCM_FULL_BUFFER for sending to RTK module of rover
      memcpy(RTCM_FULL_BUFFER, RXBUFFER, (RXPacketL-1) * sizeof(uint8_t));

      totalSize_of_RTCM += RXPacketL-1;
    }
  } 
  else // not the first packet
  {
    if(nb_of_packets != 0)
    {
      // copy RXBUFFER from Lora to RTCM_FULL_BUFFER for sending to RTK module of rover
      memcpy(RTCM_FULL_BUFFER + 254 * (nb_of_packets-nb_of_remaining_packets - 1), RXBUFFER, (RXPacketL-1) * sizeof(uint8_t));
      totalSize_of_RTCM += RXPacketL-1;

      if(nb_of_remaining_packets == 0) // the last packet
      {
        ready_for_sendingRTCM = true;
      }
    }
  }


  if(ready_for_sendingRTCM)
  {
    Serial.print("RTCM final Buffer: ");
    for(uint16_t i = 0; i < totalSize_of_RTCM; i++)
    {
      Serial.print(RTCM_FULL_BUFFER[i]);
      Serial.print(" ");
    } 
    Serial.println();

    Serial.println("Done");

    // reset some variables for making it ready to process the next turn
    ready_for_sendingRTCM = false;
    nb_of_packets = 0; 
    totalSize_of_RTCM = 0;

    // sending RTCM sentence to ZED F9P 
    //myGNSS.pushRawData(RTCM_FULL_BUFFER, totalSize_of_RTCM, false); 

  }
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
