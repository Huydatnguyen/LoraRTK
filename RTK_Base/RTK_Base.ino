
/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 04/04/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors. 
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - This is a program that demonstrates the detailed setup of a LoRa test transmitter. 
  A packet containing ASCII text is sent according to the frequency and LoRa settings specified in the
  'Settings.h' file. The pins to access the lora device need to be defined in the 'Settings.h' file also.

  The details of the packet sent and any errors are shown on the Arduino IDE Serial Monitor, together with
  the transmit power used, the packet length and the CRC of the packet. The matching receive program,
  '104_LoRa_Receiver' can be used to check the packets are being sent correctly, the frequency and LoRa
  settings (in Settings.h) must be the same for the transmitter and receiver programs. Sample Serial
  Monitor output;

  10dBm Packet> Hello World 1234567890*  BytesSent,23  CRC,DAAB  TransmitTime,64mS  PacketsSent,2

  Serial monitor baud rate is set at 9600
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

SX128XLT LT;     //create a library class instance called LT

//#define ENABLE_UPLOAD_OTA // comment this line for disabling OTA upload code
#define mySerial Serial2 // Use Serial2 to connect to the GNSS module. Change this if required


uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;

// struct for storing info abt RTCM frame
RTCM_FRAME_t storageRTCM;

uint16_t rtcmCounter = 0; // Tracks the type of incoming byte inside RTCM frame

// Depending on the sentence type the processor will load characters into different arrays
enum ublox_sentence_types
{
  UBLOX_SENTENCE_TYPE_NONE = 0,
  UBLOX_SENTENCE_TYPE_NMEA,
  UBLOX_SENTENCE_TYPE_UBX,
  UBLOX_SENTENCE_TYPE_RTCM
} current_Sentence = UBLOX_SENTENCE_TYPE_NONE;

void setup()
{
  //Setup Serial
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

  /* _______________________SPI initialization_______________________ */
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
  /*_____________________________________________________________*/


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

  Serial.print(F("Base station ready"));
  Serial.println();

}

void loop()
{
#ifdef ENABLE_UPLOAD_OTA  
  ArduinoOTA.handle();
#endif

  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  delay(250); //Don't pound too hard on the I2C bus
}

void sendLoraPacket(uint8_t buffer[], uint16_t size)
{
  // send fully RTCM msg to rover over Lora
  uint8_t *to_send = buffer;                           
  
  startmS =  millis();                                         
  //start transmit timer
  if (LT.transmit(to_send, size, 10000, TXpower, WAIT_TX))        //will return packet length sent if OK, otherwise 0 if transmit error
  {
    endmS = millis();                                          //packet sent, note end time
    packet_is_OK();
  }
  else
  {
    packet_is_Error();                                 //transmit packet returned 0, there was an error
  }

  delay(1000);                                 //have a delay between packets
}

//This function gets called from the SparkFun u-blox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void DevUBLOXGNSS::processRTCM(uint8_t incoming)
{
  // ready for receiving new UBLOX sentence
  if(current_Sentence == UBLOX_SENTENCE_TYPE_NONE)
    {
      if(incoming == 0xD3) // RTCM frames start with 0xD3
      {
          rtcmCounter = 0;
          current_Sentence = UBLOX_SENTENCE_TYPE_RTCM;

          printf("______________________Detected a new RTCM and processing it_____________________ \n");
      }
    }

  // deal with RTCM sentence
  if(current_Sentence == UBLOX_SENTENCE_TYPE_RTCM)
  {
    if(rtcmCounter == 0)
    {
        storageRTCM.dataMessage[0] = incoming;
        storageRTCM.rollingChecksum = 0; // Initialize the checksum. Seed is 0x000000
    }
    else if(rtcmCounter == 1)
    {
        storageRTCM.dataMessage[1] = incoming;
        storageRTCM.messageLength = (uint16_t)(incoming & 0x03) << 8; 
    }
    else if(rtcmCounter == 2)
    {
        storageRTCM.dataMessage[2] = incoming;
        storageRTCM.messageLength |= incoming;
    }

    // Store the mesage data (and CRC) - now that the message length is known
    if ((rtcmCounter >= 3) && (rtcmCounter < (storageRTCM.messageLength + 6)) && (rtcmCounter < (3 + SFE_UBLOX_MAX_RTCM_MSG_LEN + 3)))
      storageRTCM.dataMessage[rtcmCounter] = incoming;

    // Add incoming header and data bytes to the checksum
    if ((rtcmCounter < 3) || ((rtcmCounter >= 3) && (rtcmCounter < (storageRTCM.messageLength + 3))))
      crc24q(incoming, &storageRTCM.rollingChecksum);

    // Check if all bytes have been received
    if ((rtcmCounter >= 3) && (rtcmCounter == storageRTCM.messageLength + 5))
    {
      uint32_t expectedChecksum = storageRTCM.dataMessage[storageRTCM.messageLength + 3];
      expectedChecksum <<= 8;
      expectedChecksum |= storageRTCM.dataMessage[storageRTCM.messageLength + 4];
      expectedChecksum <<= 8;
      expectedChecksum |= storageRTCM.dataMessage[storageRTCM.messageLength + 5];

      // printf("Expected checksum:  %d  \n", expectedChecksum);
      // printf("Rolling Checksum:  %d  \n", storageRTCM.rollingChecksum);

      // checkSum??
      if (expectedChecksum == storageRTCM.rollingChecksum) // Does the checksum match?
      {
        // Extract the message number from the first 12 bits
        uint16_t messageType = ((uint16_t)storageRTCM.dataMessage[3]) << 4;
        messageType |= storageRTCM.dataMessage[4] >> 4;

        printf("Type: %d \n", messageType);
        printf("Length: %d \n", storageRTCM.messageLength);

        printf("RTCM msg:  ");
        for(uint8_t i = 0; i <= storageRTCM.messageLength+5; i++)
          { 
            printf("%02X ", storageRTCM.dataMessage[i]);
          }
        printf("\n");

        // send fully RTCM msg to rover over Lora
        sendLoraPacket(storageRTCM.dataMessage, storageRTCM.messageLength + 6);
      }
      else // Checksum does not match
        printf("Checksum failed..... \n");
    }

    rtcmCounter = rtcmCounter + 1; // Increment rtcmFrameCounter

    // Reset and start looking for next sentence type when done
    if (rtcmCounter == storageRTCM.messageLength + 6) 
    {
      current_Sentence = UBLOX_SENTENCE_TYPE_NONE;
    }
    else
      current_Sentence = UBLOX_SENTENCE_TYPE_RTCM;
  }

}

void packet_is_OK()
{
  printf("______________________Lora packet sent successfully_____________________\n");
}


void packet_is_Error()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                      //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                        //print IRQ status
  LT.printIrqStatus();                                 //prints the text of which IRQs set
}

void crc24q(uint8_t incoming, uint32_t *checksum)
{
  uint32_t crc = *checksum; // Seed is 0

  crc ^= ((uint32_t)incoming) << 16; // XOR-in incoming

  for (uint8_t i = 0; i < 8; i++)
  {
    crc <<= 1;
    if (crc & 0x1000000)
      // CRC-24Q Polynomial:
      // gi = 1 for i = 0, 1, 3, 4, 5, 6, 7, 10, 11, 14, 17, 18, 23, 24
      // 0b 1 1000 0110 0100 1100 1111 1011
      crc ^= 0x1864CFB; // CRC-24Q
  }

  *checksum = crc & 0xFFFFFF;
}
