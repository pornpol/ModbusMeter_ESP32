#ifndef ModbusMeter_ESP32_h
#define ModbusMeter_ESP32_h

/* _____STANDARD INCLUDES____________________________________________________ */
// include types & constants of Wiring core API
#include "Arduino.h"

/* _____UTILITY MACROS_______________________________________________________ */

/* _____PROJECT INCLUDES_____________________________________________________ */
// functions to calculate Modbus Application Data Unit CRC
#include "util/crc16.h"

// functions to manipulate words
#include "util/word.h"

#include <driver/uart.h>

class ModbusMeter
{
public:
  ModbusMeter();

  typedef struct __meterData
  {
    time_t mdt;
    float watt;
    float wattHour;
    float pf;
    float varh;
    float i0;
    float i1;
    float i2;
    float v0;
    float v1;
    float v2;
  } meterData;

  meterData md[10];

  typedef struct __pqData
  {
    time_t mdt;
    float watt;
    float wattHour;
    float pf;
    float varh;
    float i0;
    float i1;
    float i2;
    float v0;
    float v1;
    float v2;

    float thdvr;
    float thdvs;
    float thdvt;
    float thdir;
    float thdis;
    float thdit;

    float vunbr;
    float vunbs;
    float vunbt;
    float chr[7];
    float chs[7];
    float cht[7];

    float freq;

  } pqData;

  pqData pd[5];

  void begin(Stream &serial);
  void begin(Stream &serial, Stream &debug);
  void preTransmission(void (*)());
  void postTransmission(void (*)());

  /*_____READ HOLDING REGISTER_____*/
  uint8_t readMeterData(uint8_t, uint8_t, uint8_t, uint8_t, time_t, float *, uint16_t *, uint8_t *);

  /*_____READ DATA FROM BUFFER_____*/
  uint16_t getResponseBuffer(uint8_t);

  static const uint8_t ku8MBIllegalFunction = 0x01;
  static const uint8_t ku8MBIllegalDataAddress = 0x02;
  static const uint8_t ku8MBIllegalDataValue = 0x03;
  static const uint8_t ku8MBSlaveDeviceFailure = 0x04;
  static const uint8_t ku8MBSuccess = 0x00;
  static const uint8_t ku8MBInvalidSlaveID = 0xE0;
  static const uint8_t ku8MBInvalidFunction = 0xE1;
  static const uint8_t ku8MBResponseTimedOut = 0xE2;
  static const uint8_t ku8MBInvalidCRC = 0xE3;

private:
  Stream *_serial;
  Stream *_debug;
  static const uint8_t ku8MaxBufferSize = 128;   ///< size of response/transmit buffers
  uint16_t _u16ResponseBuffer[ku8MaxBufferSize]; ///< buffer to store Modbus slave response; read via GetResponseBuffer()
  //uint8_t _u8ResponseBufferLength;

  // preTransmission callback function; gets called before writing a Modbus message
  void (*_preTransmission)();
  // postTransmission callback function; gets called after a Modbus message has been sent
  void (*_postTransmission)();

  uint8_t masterTransaction(uint8_t slave, uint16_t startAddress, uint16_t readQty, uint8_t fnRead);
  float wordToFloat(uint16_t h, uint16_t l);
  uint32_t u16Tou32(uint16_t h, uint16_t l);

  // Modbus function codes for bit access
  static const uint8_t ku8MBReadCoils = 0x01;          ///< Modbus function 0x01 Read Coils
  static const uint8_t ku8MBReadDiscreteInputs = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
  static const uint8_t ku8MBWriteSingleCoil = 0x05;    ///< Modbus function 0x05 Write Single Coil
  static const uint8_t ku8MBWriteMultipleCoils = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

  // Modbus function codes for 16 bit access
  static const uint8_t ku8MBReadHoldingRegisters = 0x03;       ///< Modbus function 0x03 Read Holding Registers
  static const uint8_t ku8MBReadInputRegisters = 0x04;         ///< Modbus function 0x04 Read Input Registers
  static const uint8_t ku8MBWriteSingleRegister = 0x06;        ///< Modbus function 0x06 Write Single Register
  static const uint8_t ku8MBWriteMultipleRegisters = 0x10;     ///< Modbus function 0x10 Write Multiple Registers
  static const uint8_t ku8MBMaskWriteRegister = 0x16;          ///< Modbus function 0x16 Mask Write Register
  static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers

  static const uint8_t dts353 = 0x01;
  static const uint8_t eastron = 0x02;
  static const uint8_t iem3255 = 0x03;
  static const uint8_t heyuan3 = 0x04;
  static const uint8_t heyuan1 = 0x05;
  static const uint8_t circutor = 0x06;
  static const uint8_t abbm2m = 0x07;
  static const uint8_t integra1630 = 0x08;
  static const uint8_t generic3 = 0x09;
  static const uint8_t generic1 = 0x0a;
  static const uint8_t pm800 = 0x0b;

  static const uint8_t pm2230 = 0x81;
  static const uint8_t dmg610 = 0x82;
  static const uint8_t dmg800 = 0x83;

  static const uint8_t manual = 0xff;

  //Power Meter DTS-353 Protocol Parameter
  //static const uint16_t dts353DataLength      = 0x0006; //Data Length = 38
  //static const uint16_t dts353StartAddress    = 0x000E; //Start Address = 22
  //static const uint8_t dts353ADUSize          = 0x08;   //Application Data Unit Size

  // DATA type
  /*
      0: N/A

      1: Signed
      2: Unsigned
      3: HEX
      4: Binary

      5: 32 Bit Signed Big-endian               AB CD
      6: 32 Bit Signed Little-endian            CD AB
      7: 32 Bit Signed Big-endian Byte Swap     BA DC
      8: 32 Bit Signed Little-endian Byte Swap  DC BA

      9: 32 Bit Unsigned Big-endian
      10: 32 Bit Unsigned Little-endian
      11: 32 Bit Unsigned Big-endian Byte Swap
      12: 32 Bit Unsigned Little-endian Byte Swap

      13: 64 Bit Signed Big-endian              AB CD EF GH
      14: 64 Bit Signed Little-endian           GH EF CD AB
      15: 64 Bit Signed Big-endian Byte Swap    BA DC FE HG
      16: 64 Bit Signed Little-endian Byte Swap HG FE DC BA

      17: 64 Bit Unsigned Big-endian
      18: 64 Bit Unsigned Little-endian
      19: 64 Bit Unsigned Big-endian Byte Swap
      20: 64 Bit Unsigned Little-endian Byte Swap

      21: 32 Bit Float Big-endian
      22: 32 Bit Float Little-endian
      23: 32 Bit Float Big-endian Byte Swap
      24: 32 Bit Float Little-endian Byte Swap

      25: 64 Bit Double Big-endian
      26: 64 Bit Double Little-endian
      27: 64 Bit Double Big-endian Byte Swap
      28: 64 Bit Double Little-endian Byte Swap  
    */

  // Modbus timeout [milliseconds]
  static const uint16_t ku16MBResponseTimeout = 500; ///< Modbus timeout [milliseconds]
};

#endif