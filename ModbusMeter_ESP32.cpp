#include "ModbusMeter_ESP32.h"
#include <esp_task_wdt.h>

ModbusMeter::ModbusMeter(void)
{
  _preTransmission = 0;
  _postTransmission = 0;
}

void ModbusMeter::begin(Stream &serial)
{
  _serial = &serial;
}

void ModbusMeter::begin(Stream &serial, Stream &debug)
{
  _serial = &serial;
  _debug = &debug;
  _debug->println("Init Modbus Meter");
}

void ModbusMeter::preTransmission(void (*preTransmission)())
{
  _preTransmission = preTransmission;
}

void ModbusMeter::postTransmission(void (*postTransmission)())
{
  _postTransmission = postTransmission;
}

uint16_t ModbusMeter::getResponseBuffer(uint8_t u8Index)
{
  if (u8Index < ku8MaxBufferSize)
  {
    return _u16ResponseBuffer[u8Index];
  }
  else
  {
    return 0xFFFF;
  }
}

uint8_t ModbusMeter::masterTransaction(uint8_t slave, uint16_t startAddress, uint16_t readQty, uint8_t fnRead)
{
  uint16_t u16CRC;
  uint8_t u8ModbusADU[256];
  uint8_t u8ModbusADUSize = 0;
  uint8_t i;

  uint32_t u32StartTime;
  uint8_t u8BytesLeft = 8;
  uint8_t u8MBStatus = ku8MBSuccess;

  uint8_t u8MBFunction = fnRead;
  //uint8_t u8MBFunction = ku8MBReadHoldingRegisters;

  u8ModbusADU[u8ModbusADUSize++] = slave;
  // MODBUS function = readHoldingRegister
  u8ModbusADU[u8ModbusADUSize++] = u8MBFunction;
  // MODBUS Address
  u8ModbusADU[u8ModbusADUSize++] = highByte(startAddress);
  u8ModbusADU[u8ModbusADUSize++] = lowByte(startAddress);
  // MODBUS Data Size
  u8ModbusADU[u8ModbusADUSize++] = highByte(readQty);
  u8ModbusADU[u8ModbusADUSize++] = lowByte(readQty);

  // calculate CRC
  u16CRC = 0xFFFF;
  for (i = 0; i < (u8ModbusADUSize); i++)
  {
    u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
  }
  u8ModbusADU[u8ModbusADUSize++] = lowByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize++] = highByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize] = 0;

  // flush receive buffer before transmitting request
  while (_serial->read() != -1)
    ; //////////////////////////////////////////////
  // while(_serial->available())
  // {
  //   _serial->read();
  // }

  // transmit request
  if (_preTransmission)
  {
    _preTransmission();
  }
  uint8_t *data = (uint8_t *)"test";
  for (i = 0; i < u8ModbusADUSize; i++)
  {
    _serial->write(u8ModbusADU[i]);
    // uart_write_bytes(UART_NUM_2, (const char*)&u8ModbusADU[i], 1);
    if (_debug)
    {
      _debug->write(u8ModbusADU[i]);
    }
  }

  u8ModbusADUSize = 0;
  _serial->flush(); // flush transmit buffer ///////////////////////////////

  if (_postTransmission)
  {
    delay(10);
    _postTransmission();
  }

  // loop until we run out of time or bytes, or an error occurs
  u32StartTime = millis();
  while (u8BytesLeft && !u8MBStatus)
  {
    if (_serial->available())
    {
      u8ModbusADU[u8ModbusADUSize++] = _serial->read();
      // if(uart_read_bytes(UART_NUM_2, &u8ModbusADU[u8ModbusADUSize++], 1, 1000))
      // {
      u8BytesLeft--;
      // }
      //_debug->printf("Char %c \r\n", u8ModbusADU[u8ModbusADUSize-1]);
    }

    // evaluate slave ID, function code once enough bytes have been read
    if (u8ModbusADUSize == 5)
    {
      // verify response is for correct Modbus slave
      if (u8ModbusADU[0] != slave)
      {
        u8MBStatus = ku8MBInvalidSlaveID;
        break;
      }

      // verify response is for correct Modbus function code (mask exception bit 7)
      if ((u8ModbusADU[1] & 0x7F) != u8MBFunction)
      {
        u8MBStatus = ku8MBInvalidFunction;
        break;
      }

      // check whether Modbus exception occurred; return Modbus Exception Code
      if (bitRead(u8ModbusADU[1], 7))
      {
        u8MBStatus = u8ModbusADU[2];
        break;
      }

      // evaluate returned Modbus function code
      switch (u8ModbusADU[1])
      {
      case ku8MBReadCoils:
      case ku8MBReadDiscreteInputs:
      case ku8MBReadInputRegisters:
      case ku8MBReadHoldingRegisters:
      case ku8MBReadWriteMultipleRegisters:
        u8BytesLeft = u8ModbusADU[2];
        break;

      case ku8MBWriteSingleCoil:
      case ku8MBWriteMultipleCoils:
      case ku8MBWriteSingleRegister:
      case ku8MBWriteMultipleRegisters:
        u8BytesLeft = 3;
        break;

      case ku8MBMaskWriteRegister:
        u8BytesLeft = 5;
        break;
      }
    }

    if ((millis() - u32StartTime) > ku16MBResponseTimeout)
    {
      u8MBStatus = ku8MBResponseTimedOut;
    }
  }
  // verify response is large enough to inspect further
  if (!u8MBStatus && u8ModbusADUSize >= 5)
  {
    // calculate CRC
    u16CRC = 0xFFFF;
    for (i = 0; i < (u8ModbusADUSize - 2); i++)
    {
      u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
    }

    // verify CRC
    if (!u8MBStatus && (lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2] ||
                        highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1]))
    {
      u8MBStatus = ku8MBInvalidCRC;
    }
  }

  // disassemble ADU into words
  if (!u8MBStatus)
  {
    // evaluate returned Modbus function code
    switch (u8ModbusADU[1])
    {
    case ku8MBReadCoils:
    case ku8MBReadDiscreteInputs:
      // load bytes into word; response bytes are ordered L, H, L, H, ...
      for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
      {
        if (i < ku8MaxBufferSize)
        {
          _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 4], u8ModbusADU[2 * i + 3]);
        }

        //_u8ResponseBufferLength = i;
      }

      // in the event of an odd number of bytes, load last byte into zero-padded word
      if (u8ModbusADU[2] % 2)
      {
        if (i < ku8MaxBufferSize)
        {
          _u16ResponseBuffer[i] = word(0, u8ModbusADU[2 * i + 3]);
        }

        //_u8ResponseBufferLength = i + 1;
      }
      break;

    case ku8MBReadInputRegisters:
    case ku8MBReadHoldingRegisters:
    case ku8MBReadWriteMultipleRegisters:
      // load bytes into word; response bytes are ordered H, L, H, L, ...
      for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
      {
        if (i < ku8MaxBufferSize)
        {
          _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 3], u8ModbusADU[2 * i + 4]);
        }

        //_u8ResponseBufferLength = i;
      }
      break;
    }
  }
  return u8MBStatus;
}

uint8_t ModbusMeter::readMeterData(uint8_t index, uint8_t slave, uint8_t slaveIndex, uint8_t mType, time_t mdt, float *adj, uint16_t *mt, uint8_t *dt)
{
  uint8_t result = 0x00;
  // Select Meter Type
  switch (mType)
  {
  case dts353: // 3 Phase Meter
    result = masterTransaction(slave, 0x000e, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[7];
    md[index].v1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[8];
    md[index].v2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[9];

    result |= masterTransaction(slave, 0x0016, 8, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[4];
    md[index].i1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[5];
    md[index].i2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[6];
    md[index].watt = wordToFloat(getResponseBuffer(6), getResponseBuffer(7)) * adj[0];

    result |= masterTransaction(slave, 0x0034, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].pf = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[2];

    result |= masterTransaction(slave, 0x00100, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].wattHour = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[1];

    result |= masterTransaction(slave, 0x00118, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].varh = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[3];

    md[index].mdt = mdt;

    break;

  case eastron:
    result = masterTransaction(slave, 0x0000 + (2000 * slaveIndex), 12, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].v0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[7];
    md[index].v1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[8];
    md[index].v2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[9];
    md[index].i0 = wordToFloat(getResponseBuffer(6), getResponseBuffer(7)) * adj[4];
    md[index].i1 = wordToFloat(getResponseBuffer(8), getResponseBuffer(9)) * adj[5];
    md[index].i2 = wordToFloat(getResponseBuffer(10), getResponseBuffer(11)) * adj[6];

    result |= masterTransaction(slave, 0x0034 + (2000 * slaveIndex), 2, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].watt = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[0];

    result |= masterTransaction(slave, 0x003E + (2000 * slaveIndex), 2, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].pf = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[2];

    result |= masterTransaction(slave, 0x0156 + (2000 * slaveIndex), 4, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].wattHour = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[1];
    md[index].varh = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[3];

    md[index].mdt = mdt;

    break;

  case iem3255:
    result |= masterTransaction(slave, 2999, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[4];
    md[index].i1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[5];
    md[index].i2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[6];

    delay(10);

    result = masterTransaction(slave, 3027, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[7];
    md[index].v1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[8];
    md[index].v2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[9];

    delay(10);

    result |= masterTransaction(slave, 3059, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].watt = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[0];

    delay(10);

    result |= masterTransaction(slave, 3083, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].pf = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[2];
    if (isnan(md[index].pf))
      md[index].pf = 0;
    if (md[index].pf < -1.00)
      md[index].pf = (-2.0) - md[index].pf;
    if (md[index].pf > 1.00)
      md[index].pf = (2.0) - md[index].pf;

    delay(10);

    result |= masterTransaction(slave, 3203, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].wattHour = (((int64_t)getResponseBuffer(0) << 48) | ((int64_t)getResponseBuffer(1) << 32) | ((int64_t)getResponseBuffer(2) << 16) | ((int64_t)getResponseBuffer(3))) * adj[1];

    delay(10);

    result |= masterTransaction(slave, 3219, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].varh = (((int64_t)getResponseBuffer(0) << 48) | ((int64_t)getResponseBuffer(1) << 32) | ((int64_t)getResponseBuffer(2) << 16) | ((int64_t)getResponseBuffer(3))) * adj[3];

    md[index].mdt = mdt;

    break;

  case heyuan3: // 3-Phase
    result |= masterTransaction(slave, mt[0], 1, mt[10]);
    if (result)
      return result;
    md[index].watt = (getResponseBuffer(0) / 1000.00) * adj[0];

    delay(5);

    result |= masterTransaction(slave, mt[1], 2, mt[10]);
    if (result)
      return result;
    md[index].wattHour = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00) * adj[1];

    delay(5);

    result |= masterTransaction(slave, mt[2], 1, mt[10]);
    if (result)
      return result;
    md[index].pf = (getResponseBuffer(0) / 1000.00) * adj[2];

    delay(5);

    result |= masterTransaction(slave, mt[3], 2, mt[10]);
    if (result)
      return result;
    md[index].varh = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00) * adj[3];

    delay(5);

    result |= masterTransaction(slave, mt[4], 1, mt[10]);
    if (result)
      return result;
    md[index].i0 = (getResponseBuffer(0) / 100.00) * adj[4];

    delay(5);

    result |= masterTransaction(slave, mt[5], 1, mt[10]);
    if (result)
      return result;
    md[index].i1 = (getResponseBuffer(0) / 100.00) * adj[5];

    delay(5);

    result |= masterTransaction(slave, mt[6], 1, mt[10]);
    if (result)
      return result;
    md[index].i2 = (getResponseBuffer(0) / 100.00) * adj[6];

    delay(5);

    result = masterTransaction(slave, mt[7], 1, mt[10]);
    if (result)
      return result;
    md[index].v0 = (getResponseBuffer(0) / 100.00) * adj[7];

    delay(5);

    result = masterTransaction(slave, mt[8], 1, mt[10]);
    if (result)
      return result;
    md[index].v1 = (getResponseBuffer(0) / 100.00) * adj[8];

    delay(5);

    result = masterTransaction(slave, mt[9], 1, mt[10]);
    if (result)
      return result;
    md[index].v2 = (getResponseBuffer(0) / 100.00) * adj[9];

    md[index].mdt = mdt;

    break;

  case heyuan1: // 1-Phase
    result |= masterTransaction(slave, mt[0], 1, mt[10]);
    if (result)
      return result;
    md[index].watt = (getResponseBuffer(0) / 1000.00) * adj[0];

    delay(5);

    result |= masterTransaction(slave, mt[1], 2, mt[10]);
    if (result)
      return result;
    md[index].wattHour = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00) * adj[1];

    delay(5);

    result |= masterTransaction(slave, mt[2], 1, mt[10]);
    if (result)
      return result;
    md[index].pf = (getResponseBuffer(0) / 1000.00) * adj[2];

    delay(5);

    result |= masterTransaction(slave, mt[3], 2, mt[10]);
    if (result)
      return result;
    md[index].varh = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00) * adj[3];

    delay(5);

    result |= masterTransaction(slave, mt[4], 1, mt[10]);
    if (result)
      return result;
    md[index].i0 = (getResponseBuffer(0) / 100.00) * adj[4];
    md[index].i1 = 0;
    md[index].i2 = 0;

    delay(5);

    result = masterTransaction(slave, mt[7], 1, mt[10]);
    if (result)
      return result;
    md[index].v0 = (getResponseBuffer(0) / 100.00) * adj[7];
    md[index].v1 = 0;
    md[index].v2 = 0;

    md[index].mdt = mdt;

    break;

  case circutor: // 3-Phase
    result |= masterTransaction(slave, 0x1e, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].watt = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[0];

    delay(5);

    result |= masterTransaction(slave, 0x3c, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].wattHour = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[1];

    delay(5);

    result |= masterTransaction(slave, 0x26, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].pf = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00) * adj[2];

    delay(5);

    result |= masterTransaction(slave, 0x3c, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].varh = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[3];

    delay(5);

    result |= masterTransaction(slave, 0x02, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i0 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[4];

    delay(5);

    result |= masterTransaction(slave, 0x0c, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i1 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[5];

    delay(5);

    result |= masterTransaction(slave, 0x16, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i2 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[6];

    delay(5);

    result = masterTransaction(slave, 0x00, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v0 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 10.00) * adj[7];

    delay(5);

    result = masterTransaction(slave, 0x0a, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v1 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 10.00) * adj[8];

    delay(5);

    result = masterTransaction(slave, 0x14, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v2 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 10.00) * adj[9];

    md[index].mdt = mdt;

    break;

  case abbm2m: // 3-Phase
    result |= masterTransaction(slave, 0x102e, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].watt = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[0];

    delay(5);

    result |= masterTransaction(slave, 0x103e, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].wattHour = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100000.00) * adj[1];

    delay(5);

    result |= masterTransaction(slave, 0x1016, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].pf = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[2];

    delay(5);

    result |= masterTransaction(slave, 0x1040, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].varh = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100000.00) * adj[3];

    delay(5);

    result |= masterTransaction(slave, 0x1010, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i0 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[4];

    delay(5);

    result |= masterTransaction(slave, 0x1012, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i1 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[5];

    delay(5);

    result |= masterTransaction(slave, 0x1014, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i2 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[6];

    delay(5);

    result = masterTransaction(slave, 0x1002, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v0 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1.00) * adj[7];

    delay(5);

    result = masterTransaction(slave, 0x1004, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v1 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1.00) * adj[8];

    delay(5);

    result = masterTransaction(slave, 0x1006, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v2 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1.00) * adj[9];

    md[index].mdt = mdt;

    break;

  case integra1630: // 3 Phase Meter
    result = masterTransaction(slave, 0x0000, 6, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].v0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[7];
    md[index].v1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[8];
    md[index].v2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[9];

    result |= masterTransaction(slave, 0x0006, 6, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].i0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[4];
    md[index].i1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[5];
    md[index].i2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[6];

    result |= masterTransaction(slave, 0x0034, 2, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].watt = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[0];

    result |= masterTransaction(slave, 0x0048, 2, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].wattHour = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[1];

    result |= masterTransaction(slave, 0x004c, 2, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].varh = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[3];

    result |= masterTransaction(slave, 0x00fe, 2, ku8MBReadInputRegisters);
    if (result)
      return result;
    md[index].pf = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[2];

    md[index].mdt = mdt;

    break;

  case generic3: // 3-Phase
    result |= masterTransaction(slave, mt[0], 2, mt[10]);
    if (result)
      return result;
    md[index].watt = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[0];

    result |= masterTransaction(slave, mt[1], 2, mt[10]);
    if (result)
      return result;
    md[index].wattHour = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[1];

    result |= masterTransaction(slave, mt[2], 2, mt[10]);
    if (result)
      return result;
    md[index].pf = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[2];

    result |= masterTransaction(slave, mt[3], 2, mt[10]);
    if (result)
      return result;
    md[index].varh = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[3];

    result |= masterTransaction(slave, mt[4], 2, mt[10]);
    if (result)
      return result;
    md[index].i0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[4];

    result |= masterTransaction(slave, mt[5], 2, mt[10]);
    if (result)
      return result;
    md[index].i1 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[5];

    result |= masterTransaction(slave, mt[6], 2, mt[10]);
    if (result)
      return result;
    md[index].i2 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[6];

    result = masterTransaction(slave, mt[7], 2, mt[10]);
    if (result)
      return result;
    md[index].v0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[7];

    result = masterTransaction(slave, mt[8], 2, mt[10]);
    if (result)
      return result;
    md[index].v1 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[8];

    result = masterTransaction(slave, mt[9], 2, mt[10]);
    if (result)
      return result;
    md[index].v2 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[9];

    md[index].mdt = mdt;

    break;

  case generic1: // 1-Phase
    result |= masterTransaction(slave, mt[0], 2, mt[10]);
    if (result)
      return result;
    md[index].watt = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[0];

    result |= masterTransaction(slave, mt[1], 2, mt[10]);
    if (result)
      return result;
    md[index].wattHour = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[1];

    result |= masterTransaction(slave, mt[2], 2, mt[10]);
    if (result)
      return result;
    md[index].pf = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[2];

    result |= masterTransaction(slave, mt[3], 2, mt[10]);
    if (result)
      return result;
    md[index].varh = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[3];

    result |= masterTransaction(slave, mt[4], 2, mt[10]);
    if (result)
      return result;
    md[index].i0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[4];
    md[index].i1 = 0;
    md[index].i2 = 0;

    result = masterTransaction(slave, mt[7], 2, mt[10]);
    if (result)
      return result;
    md[index].v0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[7];
    md[index].v1 = 0;
    md[index].v2 = 0;

    md[index].mdt = mdt;

    break;

  case pm800: // 3 Phase Meter
    result |= masterTransaction(slave, 1099, 3, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i0 = getResponseBuffer(0) * adj[4];
    md[index].i1 = getResponseBuffer(1) * adj[5];
    md[index].i2 = getResponseBuffer(2) * adj[6];

    delay(5);

    result = masterTransaction(slave, 1123, 3, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v0 = getResponseBuffer(0) * adj[7];
    md[index].v1 = getResponseBuffer(1) * adj[8];
    md[index].v2 = getResponseBuffer(2) * adj[9];

    delay(5);

    result |= masterTransaction(slave, 1142, 1, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].watt = getResponseBuffer(0) * adj[0];

    delay(5);

    result |= masterTransaction(slave, 1715, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].wattHour = (((int64_t)getResponseBuffer(0)) + ((int64_t)getResponseBuffer(1) * 10000) + ((int64_t)getResponseBuffer(2) * 10000 * 10000) + ((int64_t)getResponseBuffer(3) * 10000 * 10000 * 10000)) * adj[1];

    delay(5);

    result |= masterTransaction(slave, 1719, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].varh = (((int64_t)getResponseBuffer(0)) + ((int64_t)getResponseBuffer(1) * 10000) + ((int64_t)getResponseBuffer(2) * 10000 * 10000) + ((int64_t)getResponseBuffer(3) * 10000 * 10000 * 10000)) * adj[3];

    delay(5);

    result |= masterTransaction(slave, 1166, 1, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].pf = (getResponseBuffer(0) / 1000.00) * adj[2];

    delay(5);

    md[index].mdt = mdt;

    break;

    /////////
    ///////// pq
  case pm2230: // PQ
    result |= masterTransaction(slave, 2999, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].i0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[4];
    pd[index].i1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[5];
    pd[index].i2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[6];

    delay(10);

    result = masterTransaction(slave, 3027, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].v0 = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[7];
    pd[index].v1 = wordToFloat(getResponseBuffer(2), getResponseBuffer(3)) * adj[8];
    pd[index].v2 = wordToFloat(getResponseBuffer(4), getResponseBuffer(5)) * adj[9];

    delay(10);

    result |= masterTransaction(slave, 3059, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].watt = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[0];

    delay(10);

    result |= masterTransaction(slave, 3083, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].pf = wordToFloat(getResponseBuffer(0), getResponseBuffer(1)) * adj[2];
    if (isnan(pd[index].pf))
      pd[index].pf = 0;
    if (pd[index].pf < -1.00)
      pd[index].pf = (-2.0) - pd[index].pf;
    if (pd[index].pf > 1.00)
      pd[index].pf = (2.0) - pd[index].pf;

    delay(10);

    result |= masterTransaction(slave, 3203, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].wattHour = (((int64_t)getResponseBuffer(0) << 48) | ((int64_t)getResponseBuffer(1) << 32) | ((int64_t)getResponseBuffer(2) << 16) | ((int64_t)getResponseBuffer(3))) * adj[1];

    delay(10);

    result |= masterTransaction(slave, 3219, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].varh = (((int64_t)getResponseBuffer(0) << 48) | ((int64_t)getResponseBuffer(1) << 32) | ((int64_t)getResponseBuffer(2) << 16) | ((int64_t)getResponseBuffer(3))) * adj[3];

    delay(10);

    // THDV
    result |= masterTransaction(slave, 21329, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].thdvr = wordToFloat(getResponseBuffer(0), getResponseBuffer(1));
    pd[index].thdvs = wordToFloat(getResponseBuffer(2), getResponseBuffer(3));
    pd[index].thdvt = wordToFloat(getResponseBuffer(4), getResponseBuffer(5));

    delay(10);

    // THDI
    result |= masterTransaction(slave, 21299, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].thdir = wordToFloat(getResponseBuffer(0), getResponseBuffer(1));
    pd[index].thdis = wordToFloat(getResponseBuffer(2), getResponseBuffer(3));
    pd[index].thdit = wordToFloat(getResponseBuffer(4), getResponseBuffer(5));

    delay(10);

    // VUNB
    result |= masterTransaction(slave, 3045, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].vunbr = wordToFloat(getResponseBuffer(0), getResponseBuffer(1));
    pd[index].vunbs = wordToFloat(getResponseBuffer(2), getResponseBuffer(3));
    pd[index].vunbt = wordToFloat(getResponseBuffer(4), getResponseBuffer(5));

    delay(10);

    // CHR
    for (uint8_t i = 0; i < 7; i++)
    {
      result |= masterTransaction(slave, 22887 + (i * 12), 2, ku8MBReadHoldingRegisters);
      if (result)
        return result;
      pd[index].chr[i] = wordToFloat(getResponseBuffer(0), getResponseBuffer(1));
      delay(15);
    }

    // CHS
    for (uint8_t i = 0; i < 7; i++)
    {
      result |= masterTransaction(slave, 23275 + (i * 12), 2, ku8MBReadHoldingRegisters);
      if (result)
        return result;
      pd[index].chs[i] = wordToFloat(getResponseBuffer(0), getResponseBuffer(1));
      delay(15);
    }

    // CHT
    for (uint8_t i = 0; i < 7; i++)
    {
      result |= masterTransaction(slave, 23663 + (i * 12), 2, ku8MBReadHoldingRegisters);
      if (result)
        return result;
      pd[index].cht[i] = wordToFloat(getResponseBuffer(0), getResponseBuffer(1));
      delay(15);
    }

    result |= masterTransaction(slave, 3109, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].freq = wordToFloat(getResponseBuffer(0), getResponseBuffer(1));

    pd[index].mdt = mdt;

    break;

  case dmg610: // PQ
    result |= masterTransaction(slave, 0x0008 - 1, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].i0 = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 10000.00 * adj[4];
    pd[index].i1 = u16Tou32(getResponseBuffer(3), getResponseBuffer(2)) / 10000.00 * adj[5];
    pd[index].i2 = u16Tou32(getResponseBuffer(5), getResponseBuffer(4)) / 10000.00 * adj[6];

    delay(10);

    result = masterTransaction(slave, 0x0002 - 1, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].v0 = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00 * adj[7];
    pd[index].v1 = u16Tou32(getResponseBuffer(3), getResponseBuffer(2)) / 100.00 * adj[8];
    pd[index].v2 = u16Tou32(getResponseBuffer(5), getResponseBuffer(4)) / 100.00 * adj[9];

    delay(10);

    result |= masterTransaction(slave, 0x003a - 1, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].watt = ((int32_t)u16Tou32(getResponseBuffer(1), getResponseBuffer(0))) / 100.00 * adj[0];

    delay(10);

    result |= masterTransaction(slave, 0x0040 - 1, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].pf = ((int32_t)u16Tou32(getResponseBuffer(1), getResponseBuffer(0))) / 10000.00 * adj[2];

    delay(10);

    result |= masterTransaction(slave, 0x1b20 - 1, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].wattHour = (((uint64_t)getResponseBuffer(0) << 48) | ((uint64_t)getResponseBuffer(1) << 32) | ((uint64_t)getResponseBuffer(2) << 16) | ((uint64_t)getResponseBuffer(3))) / 100.00 * adj[1];

    delay(10);

    result |= masterTransaction(slave, 0x1b28 - 1, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].varh = (((uint64_t)getResponseBuffer(0) << 48) | ((uint64_t)getResponseBuffer(1) << 32) | ((uint64_t)getResponseBuffer(2) << 16) | ((uint64_t)getResponseBuffer(3))) / 100.00 * adj[3];

    delay(10);

    // THDV
    result |= masterTransaction(slave, 0x0054 - 1, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].thdvr = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00;
    pd[index].thdvs = u16Tou32(getResponseBuffer(3), getResponseBuffer(2)) / 100.00;
    pd[index].thdvt = u16Tou32(getResponseBuffer(5), getResponseBuffer(4)) / 100.00;

    delay(10);

    // THDI
    result |= masterTransaction(slave, 0x005a - 1, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].thdir = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00;
    pd[index].thdis = u16Tou32(getResponseBuffer(3), getResponseBuffer(2)) / 100.00;
    pd[index].thdit = u16Tou32(getResponseBuffer(5), getResponseBuffer(4)) / 100.00;

    delay(10);

    // VUNB
    // result |= masterTransaction(slave, 3045, 6, ku8MBReadHoldingRegisters);
    // if (result)
    //   return result;
    pd[index].vunbr = 0;
    pd[index].vunbs = 0;
    pd[index].vunbt = 0;

    delay(10);

    // CHR
    for (uint8_t i = 0; i < 7; i++)
    {
      // result |= masterTransaction(slave, 0x31c0 + (i * 2) - 1, 2, ku8MBReadHoldingRegisters);
      // if (result)
      //   return result;
      pd[index].chr[i] = 0;
      delay(15);
    }

    // CHS
    for (uint8_t i = 0; i < 7; i++)
    {
      // result |= masterTransaction(slave, 0x3202 + (i * 2) - 1, 2, ku8MBReadHoldingRegisters);
      // if (result)
      //   return result;
      pd[index].chs[i] = 0;
      delay(15);
    }

    // CHT
    for (uint8_t i = 0; i < 7; i++)
    {
      // result |= masterTransaction(slave, 0x3242 + (i * 2) - 1, 2, ku8MBReadHoldingRegisters);
      // if (result)
      //   return result;
      pd[index].cht[i] = 0;
      delay(15);
    }

    // FREQ
    result |= masterTransaction(slave, 0x0032 - 1, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].freq = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00;

    pd[index].mdt = mdt;

    break;

  case dmg800: // PQ
    result |= masterTransaction(slave, 0x0008 - 1, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].i0 = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 10000.00 * adj[4];
    pd[index].i1 = u16Tou32(getResponseBuffer(3), getResponseBuffer(2)) / 10000.00 * adj[5];
    pd[index].i2 = u16Tou32(getResponseBuffer(5), getResponseBuffer(4)) / 10000.00 * adj[6];

    delay(10);

    result = masterTransaction(slave, 0x0002 - 1, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].v0 = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00 * adj[7];
    pd[index].v1 = u16Tou32(getResponseBuffer(3), getResponseBuffer(2)) / 100.00 * adj[8];
    pd[index].v2 = u16Tou32(getResponseBuffer(5), getResponseBuffer(4)) / 100.00 * adj[9];

    delay(10);

    result |= masterTransaction(slave, 0x003a - 1, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].watt = ((int32_t)u16Tou32(getResponseBuffer(1), getResponseBuffer(0))) / 100.00 * adj[0];

    delay(10);

    result |= masterTransaction(slave, 0x0040 - 1, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].pf = ((int32_t)u16Tou32(getResponseBuffer(1), getResponseBuffer(0))) / 10000.00 * adj[2];

    delay(10);

    result |= masterTransaction(slave, 0x1b20 - 1, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].wattHour = (((uint64_t)getResponseBuffer(0) << 48) | ((uint64_t)getResponseBuffer(1) << 32) | ((uint64_t)getResponseBuffer(2) << 16) | ((uint64_t)getResponseBuffer(3))) / 100.00 * adj[1];

    delay(10);

    result |= masterTransaction(slave, 0x1b28 - 1, 4, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].varh = (((uint64_t)getResponseBuffer(0) << 48) | ((uint64_t)getResponseBuffer(1) << 32) | ((uint64_t)getResponseBuffer(2) << 16) | ((uint64_t)getResponseBuffer(3))) / 100.00 * adj[3];

    delay(10);

    // THDV
    result |= masterTransaction(slave, 0x0054 - 1, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].thdvr = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00;
    pd[index].thdvs = u16Tou32(getResponseBuffer(3), getResponseBuffer(2)) / 100.00;
    pd[index].thdvt = u16Tou32(getResponseBuffer(5), getResponseBuffer(4)) / 100.00;

    delay(10);

    // THDI
    result |= masterTransaction(slave, 0x005a - 1, 6, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].thdir = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100.00;
    pd[index].thdis = u16Tou32(getResponseBuffer(3), getResponseBuffer(2)) / 100.00;
    pd[index].thdit = u16Tou32(getResponseBuffer(5), getResponseBuffer(4)) / 100.00;

    delay(10);

    // VUNB
    // result |= masterTransaction(slave, 3045, 6, ku8MBReadHoldingRegisters);
    // if (result)
    //   return result;
    pd[index].vunbr = 0;
    pd[index].vunbs = 0;
    pd[index].vunbt = 0;

    delay(10);

    // CHR
    for (uint8_t i = 0; i < 7; i++)
    {
      result |= masterTransaction(slave, 0x0c02 + (i * 2) - 1, 2, ku8MBReadHoldingRegisters);
      if (result)
        return result;
      pd[index].chr[i] = 0;
      delay(15);
    }

    // CHS
    for (uint8_t i = 0; i < 7; i++)
    {
      result |= masterTransaction(slave, 0x0c42 + (i * 2) - 1, 2, ku8MBReadHoldingRegisters);
      if (result)
        return result;
      pd[index].chs[i] = 0;
      delay(15);
    }

    // CHT
    for (uint8_t i = 0; i < 7; i++)
    {
      result |= masterTransaction(slave, 0x0c82 + (i * 2) - 1, 2, ku8MBReadHoldingRegisters);
      if (result)
        return result;
      pd[index].cht[i] = 0;
      delay(15);
    }

    // FREQ
    result |= masterTransaction(slave, 0x0032 - 1, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    pd[index].freq = u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00;

    pd[index].mdt = mdt;

    break;
    /////////
    /////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  case manual: // Manual

    uint8_t num = 0;
    float value = 0;

    for (uint8_t k = 0; k < 10; k++)
    {

      switch (dt[k])
      {
      case 1:
        result |= masterTransaction(slave, mt[k], 2, mt[10]);
        if (result)
          return result;
        value = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)));
        break;
      }

      switch (k)
      {
      case 0:
        md[index].watt = value * adj[0];
        break;
      case 1:
        md[index].watt = value * adj[1];
        break;
      }
    }

    result |= masterTransaction(slave, 0x102e, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].watt = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[0];

    delay(5);

    result |= masterTransaction(slave, 0x103e, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].wattHour = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100000.00) * adj[1];

    delay(5);

    result |= masterTransaction(slave, 0x1016, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].pf = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[2];

    delay(5);

    result |= masterTransaction(slave, 0x1040, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].varh = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 100000.00) * adj[3];

    delay(5);

    result |= masterTransaction(slave, 0x1010, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i0 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[4];

    delay(5);

    result |= masterTransaction(slave, 0x1012, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i1 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[5];

    delay(5);

    result |= masterTransaction(slave, 0x1014, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].i2 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1000.00) * adj[6];

    delay(5);

    result = masterTransaction(slave, 0x1002, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v0 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1.00) * adj[7];

    delay(5);

    result = masterTransaction(slave, 0x1004, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v1 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1.00) * adj[8];

    delay(5);

    result = masterTransaction(slave, 0x1006, 2, ku8MBReadHoldingRegisters);
    if (result)
      return result;
    md[index].v2 = (u16Tou32(getResponseBuffer(1), getResponseBuffer(0)) / 1.00) * adj[9];

    md[index].mdt = mdt;

    break;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  return result;
}

float ModbusMeter::wordToFloat(uint16_t h, uint16_t l)
{
  typedef union {
    float number;
    uint16_t wd[2];
  } FLOATUNION_t;

  FLOATUNION_t myFloat;

  myFloat.wd[0] = l;
  myFloat.wd[1] = h;

  return myFloat.number;
}

uint32_t ModbusMeter::u16Tou32(uint16_t h, uint16_t l)
{
  typedef union {
    uint32_t number;
    uint16_t wd[2];
  } FLOATUNION_t;

  FLOATUNION_t myU32;

  myU32.wd[0] = h;
  myU32.wd[1] = l;

  return myU32.number;
}