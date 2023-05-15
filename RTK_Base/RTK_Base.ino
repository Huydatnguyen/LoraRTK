
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

SX128XLT LT;     //create a library class instance called LT


// for testing 
uint8_t buffer[1029];

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

  myGNSS.setI2COutput(COM_TYPE_RTCM3); // Ensure RTCM3 is enabled for output
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR
  //***************************************************************************************************


  Serial.print(F("Base station ready"));
  Serial.println();

  // // just for testing the case RTCM size > 255
  // Serial.print("Buffer: ");
  // for(uint16_t i = 0; i < 1029; i++)
  // {
  //   buffer[i] = i%255;
  //   Serial.print(buffer[i]);
  //   Serial.print(" ");
  // } 
  // Serial.print("\n");
  
}

void loop()
{
  // enable updating code OTA
#ifdef ENABLE_UPLOAD_OTA  
  ArduinoOTA.handle();
#endif

  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  delay(500); //Don't pound too hard on the I2C bus

  //sendLoraPacket(buffer, 200);
}

void sendLoraPacket(uint8_t buffer[], uint16_t size)
{
  if(size < 255)  // Deal with RTCM size < 255
  {
    // append an additional byte to the end of Lora packet for detecting order of Lora packet
    // format of additional byte: | SYN (1 bit, set 1 if first packet, otherwise set 0) |  nb of remaining packets (3 bits)    | 4 bits reserved |   
    buffer[size] = 0x80; // 10000000 in binary 

    // to_send points to the first element in buffer
    uint8_t *to_send = buffer;                           
    
    //start transmitting
    if (LT.transmit(to_send, size + 1, 10000, TXpower, WAIT_TX))        //will return packet length sent if OK, otherwise 0 if transmit error
    {
      packet_is_OK();

      // for(uint8_t i = 0; i < size + 1; i++)
      // {
      //   Serial.print(*(to_send + i));
      //   Serial.print(" ");
      // }
      // Serial.println();
    }
    else
    {
      packet_is_Error();                                 //transmit packet returned 0, there was an error
    }
  }
  else // Deal with RTCM size >= 255
  {
    uint8_t numberOfPackets;
    uint8_t SYN;
    uint8_t nb_of_remaining_packets;

    // compute the nb of packets needed to be transmitted
    if(size / 254 == 0)
      numberOfPackets = size/254;
    else
      numberOfPackets = size/254 + 1;

    // start sending each packet
    for(uint8_t i=1; i <= numberOfPackets ; i++)
    {
      if(i == 1) // first packet 
      {
        SYN = 1;
        nb_of_remaining_packets = numberOfPackets - i;  

        // define packet for copying data from the original buffer
        uint8_t first_pack[255];
        memcpy(first_pack, buffer, 255 * sizeof(uint8_t));

        // append an additional byte to the end of Lora packet for detecting the order of Lora packet
        first_pack[254] = (uint8_t) (nb_of_remaining_packets << 4) | (SYN << 7);

        // to_send points to the first element in packet
        uint8_t *to_send = first_pack;                           
                                         
        //start transmitting
        if (LT.transmit(to_send, 255, 10000, TXpower, WAIT_TX))        //will return packet length sent if OK, otherwise 0 if transmit error
        {
          packet_is_OK();

          // Serial.println("First packet");
          // for(uint8_t j = 0; j < 255; j++)
          // {
          //   Serial.print(*(to_send + j));
          //   Serial.print("  ");
          // }
          // Serial.println();
        }
        else
        {
          packet_is_Error();                                 //transmit packet returned 0, there was an error
        }
      }
      else if (i == numberOfPackets) // last packet
      {
        SYN = 0;
        nb_of_remaining_packets = 0;
        uint8_t size_of_last_packet = size - 254* (numberOfPackets-1);

        // define packet for copying data from the original buffer
        uint8_t last_pack[size_of_last_packet+1];
        memcpy(last_pack, buffer + 254 * (i-1), size_of_last_packet * sizeof(uint8_t));

        // append an additional byte to the end of Lora packet for detecting the order of Lora packet
        last_pack[size_of_last_packet] = (nb_of_remaining_packets << 4) | (SYN << 7);

        // to_send points to the first element in packet
        uint8_t *to_send = last_pack;                           
                                            
        //start transmitting 
        if (LT.transmit(to_send, size_of_last_packet + 1 , 10000, TXpower, WAIT_TX))        //will return packet length sent if OK, otherwise 0 if transmit error
        {
          packet_is_OK();

          // Serial.println("Last packet");
          // for(uint8_t j = 0; j < size - 254* (numberOfPackets-1) + 1; j++)
          // {
          //   Serial.print(*(to_send + j));
          //   Serial.print("  ");
          // }
          // Serial.println();
        }
        else
        {
          packet_is_Error();                                 //transmit packet returned 0, there was an error
        }
      }
      else {
        SYN = 0;
        nb_of_remaining_packets = numberOfPackets - i;

        // define packet for copying data from the original buffer
        uint8_t middle_pack[255];
        memcpy(middle_pack, buffer + 254 * (i-1), 255 * sizeof(uint8_t));

        // append an additional byte to the end of Lora packet for detecting the order of Lora packet 
        middle_pack[254] = (nb_of_remaining_packets << 4) | (SYN << 7);

        // to_send points to the first element in packet
        uint8_t *to_send = middle_pack;                           
                                             
        //start transmitting
        if (LT.transmit(to_send, 255, 10000, TXpower, WAIT_TX))        //will return packet length sent if OK, otherwise 0 if transmit error
        {
          packet_is_OK();

          // Serial.println("Middle packet");
          // Serial.print("Remaining packets: ");
          // Serial.println(nb_of_remaining_packets);
          // for(uint8_t j = 0; j < 255; j++)
          // {
          //   Serial.print(*(to_send + j));
          //   Serial.print("  ");
          // }
          // Serial.println();
        }
        else
        {
          packet_is_Error();                                 //transmit packet returned 0, there was an error
        }
      }
    }
  }

  //delay(1000);                                 //have a delay between packets
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
