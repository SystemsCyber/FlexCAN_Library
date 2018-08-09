/*
 * Object Oriented CAN example for Teensy 3.6 with Dual CAN buses 
 * By Collin Kidder. Based upon the work of Pawelsky and Teachop
 * 
 *
 * The reception of frames in this example is done via callbacks
 * to an object rather than polling. Frames are delivered as they come in.
 */
#ifndef __MK66FX1M0__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif
 
#include "FlexCAN.h"
#include "SdFat.h"

// 32 KiB buffer.
const size_t BUF_DIM = 32768;
// 8 MiB file.

const uint32_t FILE_SIZE = 256UL*BUF_DIM;

SdFatSdioEX sdEx;
File file;
uint8_t buf[BUF_DIM];

// buffer as uint32_t
uint32_t* buf32 = (uint32_t*)buf;

bool OKtoWrite = false;
bool waiting = false;

size_t numBytes = 512;


uint8_t RxError;
elapsedMillis printTimer;
elapsedMillis errorCountTimer;

elapsedMillis recordTimer;

//// Create a new class to define functions in the CANListener class in FlexCAN.cpp
//class CANPrinter : public CANListener 
//{
//public:
//   //overrides the parent version so we can actually do something
//   void printFrame(CAN_message_t &frame, int8_t mailbox, uint8_t controller);
//   void logFrame(CAN_message_t &frame, int8_t mailbox, uint8_t controller);
//   bool frameHandler(CAN_message_t &frame, int8_t mailbox, uint8_t controller); 
//};
//
//class CANLogger : public FlexCAN 
//{
//public:
//   //overrides the parent version so we can actually do something
//   void processBuffer(ringbuffer_t &ring);
//};
//
//void CANLogger::processBuffer(ringbuffer_t &ring)
//{
//    file.write(buf, numBytes);
//    // This function can't keep up with a fully loaded CAN bus.
//    char message[19];
//    if (mailbox < 0){
//      sprintf(message, "%5d Can%d", frame.timestamp, controller);   
//    }
//    else {
//      sprintf(message, "%5d Can%d (%d)", frame.timestamp, controller, mailbox);
//    }
//    Serial.print(message);
//
//    if (frame.flags.extended){
//      sprintf(message, "  %08X   [%d] ", frame.id, frame.len);
//    }
//    else {
//      sprintf(message, "  %03X   [%d] ", frame.id, frame.len);
//    }
//    Serial.print(message);
//    char byteStr[5];
//    for (uint8_t c = 0; c < frame.len; c++) 
//    {
//       sprintf(byteStr, " %02X", frame.buf[c]); 
//       Serial.print(byteStr);
//    }
//    Serial.println();
//}
//
//void logFrame(CAN_message_t &frame, int8_t mailbox, uint8_t controller)
//{
//  if (frame.flags.extended){
//    waiting = false;
//    //Add code to log data here.
//    uint32_t dataIn = micros();
//    buf[3] = dataIn;
//    dataIn >>= 8;
//    buf[2] = dataIn;
//    dataIn >>= 8;
//    buf[1] = dataIn;
//    dataIn >>= 8;
//    buf[0] = dataIn;
//    
//    dataIn = millis();
//    buf[7] = dataIn;
//    dataIn >>= 8;
//    buf[6] = dataIn;
//    dataIn >>= 8;
//    buf[5] = dataIn;
//    dataIn >>= 8;
//    buf[4] = dataIn;
//    
//    dataIn = frame.timestamp;
//    dataIn >>= 16;
//    buf[9] = dataIn;
//    dataIn >>= 8;
//    buf[8] = dataIn;
//    
//    dataIn = frame.id;
//    buf[13] = dataIn;
//    dataIn >>= 8;
//    buf[12] = dataIn;
//    dataIn >>= 8;
//    buf[11] = dataIn;
//    dataIn >>= 8;
//    buf[10] = dataIn;
//    if (OKtoWrite){
//      file.write(buf, numBytes);  
//    }
//  }
//  
//  
//  
//}
//
//bool CANPrinter::frameHandler(CAN_message_t &frame, int8_t mailbox, uint8_t controller)
//{
//    logFrame(frame, mailbox, controller);
//    
//    // This function can't keep up with a fully loaded CAN bus
//    // A fully loaded bus may crash the Teensy when filling the serial buffer
//    if (printTimer>5){
//      printTimer = 0;
//      if (OKtoWrite){
//        printFrame(frame, mailbox, controller);
//      }
//    }
//    
//    // be sure to return true to tell the library that you've processed the frame.
//    // Otherwise, the data goes into a ringbuffer and waits to be read. 
//    return true;
//}

//CANPrinter canPrinter;

static CAN_message_t tempBuffer[SIZE_RX_BUFFER];

// -------------------------------------------------------------
void setup(void)
{
  
  while (!Serial);
  Serial.println(F("Hello Teensy 3.6 dual CAN Test With Objects."));
  if (!sdEx.begin()) {
    sdEx.initErrorHalt("SdFatSdioEX begin() failed");
  }
  // make sdEx the current volume.
  sdEx.chvol();
  if (!file.open("TeensyCANLog.bin", (O_WRITE | O_CREAT | O_TRUNC))) {
    sdEx.errorHalt("open failed");
  }
  OKtoWrite = true;
  
  CAN_filter_t standardPassFilter;
  standardPassFilter.id=0;
  standardPassFilter.flags.extended=0;

  CAN_filter_t extendedPassFilter;
  extendedPassFilter.id=0;
  extendedPassFilter.flags.extended=1;
  
  // Options for a Detroit Diesel CPC4
  Can0.begin(250000,standardPassFilter); 
  // standardPassFilter is the default option and can be omitted. 
  Can1.begin(666666);
//  Can0.attachObj(&canPrinter);
//  Can1.attachObj(&canPrinter);

  // Give a few mailboxes the ability to read extended IDs
  for (uint8_t filterNum = 0; filterNum < 4; filterNum++){
    Can0.setFilter(extendedPassFilter,filterNum); 
    Can1.setFilter(extendedPassFilter,filterNum); 
  }
//  canPrinter.attachGeneralHandler();

  while(waiting);
  recordTimer = 0;
  int previousCan0buf = Can0.currentBuffer;
  Serial.print("Can0.currentBuffer: ");
  Serial.println(Can0.currentBuffer);
  Serial.print("Can1.currentBuffer: ");
  Serial.println(Can1.currentBuffer);
  while (recordTimer<5000){
    if (Can0.currentBuffer != previousCan0buf){
      memcpy(&tempBuffer,&Can0.rxRing[previousCan0buf].buffer, sizeof(Can0.rxRing[previousCan0buf].buffer));
      file.write(tempBuffer,SIZE_RX_BUFFER);
      Can0.resetRingBuffer(Can0.rxRing[previousCan0buf]);
      previousCan0buf = Can0.currentBuffer;
      Serial.print("Can0.currentBuffer: ");
      Serial.println(Can0.currentBuffer);
    }
    
    
  }
  
  OKtoWrite = false;
  file.close();
  recordTimer = 0;
  // re-open the file for reading:
  file = sdEx.open("TeensyCANLog.bin", FILE_READ);
  if (file) {
    Serial.println("TeensyCANLog.bin:");

    // read from the file until there's nothing else in it:
    int count;
    char byteStr[4];
    Serial.println(file.available());
    while (file.available()) {
      OKtoWrite = false;
      sprintf(byteStr, " %02X", file.read()); 
      Serial.print(byteStr);
      count++;
      if (count >= 512){
        count = 0;
        delay(1);
        Serial.println();
      }
    }
    // close the file:
    file.close();
  }
  Serial.print("Done.");
}

// -------------------------------------------------------------
void loop(void)
{ 
  RxError = Can0.readRxError();
  if (RxError){
    if (errorCountTimer>5){
     errorCountTimer = 0;
     Serial.print("Can0 RxError: ");
     Serial.println(RxError);
    }
    
 
  }
  RxError = Can1.readRxError();
  if (RxError){
    Serial.print("Can1 RxError: ");
    Serial.println(RxError);
    
  }
  
}


