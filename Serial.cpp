
// ######################################################################################
// Include Libraries:

#include "Serial.h"

#include <cstdio>
#include <cstring>

namespace
{
  bool isTimedOut(unsigned long startTick, unsigned long timeout)
  {
    if(timeout == HAL_MAX_DELAY)
    {
      return false;
    }

    return (HAL_GetTick() - startTick) >= timeout;
  }
}

// ######################################################################################
// Serial class:

// -----------------------------------------------------------------------------------
// base and initial methods:

Serial::Serial(char* txBuffer, size_t txBufferSize, char* rxBuffer, size_t rxBufferSize, BufferType txType, BufferType rxType)
{
	_baudRate = 9600;
  _huart = nullptr;
  _timeout = HAL_MAX_DELAY;
  _txMode = PROGRAM_MODE_BLOCK;
  _rxMode = PROGRAM_MODE_INTERRUPT;
  _isTransmitting = false;
  _isReceiving = false;
  setTxBuffer(txBuffer, txBufferSize, txType);
  setRxBuffer(rxBuffer, rxBufferSize, rxType);
}

bool Serial::begin(unsigned long baudRate)
{
  return begin(_huart, baudRate);
}

bool Serial::begin(UART_HandleTypeDef* huart, unsigned long baudRate)
{
  bool state = ( (baudRate == 9600) || (baudRate == 57600) || (baudRate == 115200) ) && 
               (huart != nullptr);

  if(state == false)
  {
    // sprintf(errorMessage, "Parameters");
    std::snprintf(errorMessage, sizeof(errorMessage), "Parameters");
    return false;
  }

  _huart = huart;
  _baudRate = baudRate;

  _huart->Init.BaudRate = _baudRate;

  if (HAL_UART_Init(_huart) != HAL_OK)
  {
    // sprintf(errorMessage, "HAL_UAER_Init()");
    std::snprintf(errorMessage, sizeof(errorMessage), "HAL_UART_Init()");
    return false;
  }
	
  if(_rxMode == PROGRAM_MODE_INTERRUPT)
  {
    _isReceiving = true;
    if(HAL_UART_Receive_IT(_huart, (uint8_t*)&_rxBuffer, 1) != HAL_OK)
    {
      return false;
    }
  }

  if(_txMode == PROGRAM_MODE_INTERRUPT)
  {
    _startTxIfIdle();
  }
	return true;
}

// ------------------------------------------------------------------------
// Set methods:

void Serial::setUart(UART_HandleTypeDef* huart)
{
  _huart = huart;
}

void Serial::setTimeout(unsigned long timeout)
{
  _timeout = timeout;
}

void Serial::setTxBuffer(char* txBuffer, uint16_t txBufferSize, BufferType txType)
{
  stream.setTxBuffer(txBuffer, txBufferSize, txType);
}

void Serial::setRxBuffer(char* rxBuffer, uint16_t rxBufferSize, BufferType rxType)
{
  stream.setRxBuffer(rxBuffer, rxBufferSize, rxType);
}

void Serial::setBufferTypes(BufferType txType, BufferType rxType)
{
  stream.setBufferTypes(txType, rxType);
}

bool Serial::setTxMode(uint8_t mode)
{
  if((mode != PROGRAM_MODE_BLOCK) && (mode != PROGRAM_MODE_INTERRUPT) && (mode != PROGRAM_MODE_DMA))
  {
    return false;
  }

  // If user wants interrupt/DMA, TX must be ring to avoid memmove while HAL reads
  if (mode == PROGRAM_MODE_INTERRUPT /* || mode == PROGRAM_MODE_DMA */)
  {
      // If you can read current tx type from Stream, check it.
      // Otherwise: document requirement or force it via stream.setBufferTypes().
      // stream.setBufferTypes(BUFFER_RING, /*keep rx*/ BUFFER_RING or current);
  }

  _txMode = mode;

  return true;
}

bool Serial::setRxMode(uint8_t mode)
{
  if((mode != PROGRAM_MODE_BLOCK) && (mode != PROGRAM_MODE_INTERRUPT) && (mode != PROGRAM_MODE_DMA))
  {
    return false;
  }

  _rxMode = mode;

  return true;
}

// -------------------------------------------------------------------------
// get/read methods:

uint16_t Serial::available(void)
{
  return stream.availableRx();
}

size_t Serial::availableForWrite(void)
{
  return stream.availableTx();
} 

void Serial::clearTxBuffer() 
{
  stream.clearTxBuffer();
}

void Serial::clearRxBuffer() 
{
  stream.clearRxBuffer();
}

bool Serial::removeFrontTxBuffer(uint32_t dataSize)
{
  return stream.removeFrontTxBuffer(dataSize);
}

bool Serial::removeFrontRxBuffer(uint32_t dataSize)
{
  return stream.removeFrontRxBuffer(dataSize);
}

bool Serial::find(const char* target, size_t length)
{
    if(target == nullptr || length == 0)
    {
      return false;
    }

    size_t targetIndex = 0;  // Track the index in the target string
    const unsigned long startTick = HAL_GetTick();

    // Loop to read characters from the serial buffer
    while (!isTimedOut(startTick, _timeout)) 
    {
        int16_t data = read();  // Read one character from serial
        
        // If no data is available or read() fails, continue checking
        if (data == -1) 
        {
            continue;
        }

        // Check if the current character matches the target character
        if (static_cast<char>(data) == target[targetIndex]) 
        {
            targetIndex++;  // Move to the next character in the target string

            // If the entire target string is matched, return true
            if (targetIndex == length) 
            {
                return true;
            }
        } else {
            // If there's a mismatch, reset target index to 0 and start over
            targetIndex = 0;
        }
    }
    
    // Timeout reached, return false if the target was not found
    return false;
}

bool Serial::find(const std::string& target)
{
  return find(target.c_str(), target.length());
}

bool Serial::findUntil(const char* target, size_t length, const char terminate)
{
  if(target == nullptr || length == 0)
  {
    return false;
  }

  size_t targetIndex = 0;
  const unsigned long startTick = HAL_GetTick();

  while(!isTimedOut(startTick, _timeout))
  {
    const int16_t data = read();

    if(data == -1)
    {
      continue;
    }

    const char ch = static_cast<char>(data);
    if(ch == terminate)
    {
      return false;
    }

    if(ch == target[targetIndex])
    {
      ++targetIndex;
      if(targetIndex == length)
      {
        return true;
      }
    }
    else
    {
      targetIndex = 0;
    }
  }

  return false;
}

bool Serial::findUntil(const std::string target, const char terminate)
{
  return findUntil(target.c_str(), target.length(), terminate);
}

int16_t Serial::peek(void)
{
  if(stream.availableRx() == 0)
  {
    return -1;
  }

  const char* data = stream.getRxBuffer();

  return (uint8_t)*data;
}

int16_t Serial::read(void)
{
  if(stream.availableRx() == 0)
  {
    return -1;
  }

  char data = 0;

  stream.popFrontRxBuffer(&data, 1);

  return data;
}

size_t Serial::readBytes(char* buffer, size_t length)
{
  if(buffer == nullptr || length == 0)
  {
    return 0;
  }

  switch(_rxMode)
  {
    case PROGRAM_MODE_BLOCK:
      if(HAL_UART_Receive(_huart, (uint8_t*)buffer, length, _timeout) == HAL_OK)
      {
        return length;
      }
      else
      {
        return 0;
      }
    break;
    case PROGRAM_MODE_INTERRUPT:
      {
        const size_t availableBytes = stream.availableRx();
        if(availableBytes == 0)
        {
          return 0;
        }

        const size_t bytesToRead = (availableBytes < length) ? availableBytes : length;

        if(stream.popFrontRxBuffer(buffer, bytesToRead) != true)
        {
          return 0;
        }
        return bytesToRead;
      }
    break;
    case PROGRAM_MODE_DMA:

    break;
    default:
      return 0;
  }

  return length;
}

size_t Serial::readBytesUntil(char character, char* buffer, size_t length)
{
    buffer[0] = '\0';  // Initialize buffer as an empty string
    size_t bytesRead = 0;

    for(size_t i = 0; i < length - 1; i++)  // Loop stops before reaching the last index
    {
        int16_t data = read();  // Read data from serial
        if (data == -1 || data == character)  // Check if end condition is met
        {
            break;
        }
        buffer[bytesRead++] = static_cast<char>(data);  // Add the read character to buffer
    }
    
    buffer[bytesRead] = '\0';  // Null-terminate the string
    return bytesRead;  // Return the number of bytes read
}

size_t Serial::readAll(char* buffer, size_t maxLength)
{
  size_t length = stream.availableRx();

  if(length > maxLength)
  {
    length = maxLength;
  }
  
  if(stream.popFrontRxBuffer(buffer, length) == false)
  {
    return 0;
  }

  return length;
}

#if defined(__linux__)
  std::string Serial::readAll(void)
  {
    size_t length = stream.availableRx();
    std::string buffer;

    if(stream.popAllRxBuffer(&buffer) == false)
    {
      buffer.assign("");
      return buffer;
    }

    return buffer;
  }
#endif

// ---------------------------------------------------------------------------
// write, flush method

bool Serial::flush(uint32_t timeoutMs)
{
  uint32_t start = HAL_GetTick();
  while (stream.availableTx() > 0 || _huart->gState != HAL_UART_STATE_READY)
  {
      if ((HAL_GetTick() - start) > timeoutMs)
          return false;
  }
  return true;
}

bool Serial::kickTx()
{
    if (_huart == nullptr) return false;

    // If HAL says busy, do nothing
    if (_huart->gState != HAL_UART_STATE_READY)
        return true;

    uint16_t n = (uint16_t)stream.txContiguousSize();
    if (n == 0)
        return true;

    const uint8_t* p = (const uint8_t*)stream.txReadPtr();
    _txBufferSize2Transmitting = n;

    return (HAL_UART_Transmit_IT(_huart, (uint8_t*)p, n) == HAL_OK);
}

uint16_t Serial::write(uint8_t data)
{
  return write(&data, 1);
}

uint16_t Serial::write(uint8_t* data, uint16_t length)
{
  if (_huart == nullptr || data == nullptr || length == 0) return 0;

  HAL_StatusTypeDef status = HAL_OK;

  switch(_txMode)
  {
    case PROGRAM_MODE_BLOCK:
      status = HAL_UART_Transmit(_huart, data, length, _timeout);
    break;
    case PROGRAM_MODE_INTERRUPT:
      // Always queue into internal buffer first
      if (!stream.pushBackTxBuffer((char*)data, length))
        return 0;

      // Just attempt. If busy it will do nothing.
      if (!kickTx())
          return 0;
      status = HAL_OK;
    break;
    case PROGRAM_MODE_DMA:

    break;
    default:
      return 0;
  }

  if(status != HAL_OK)
  {
    _isTransmitting = false;
    return 0;
  }

  return length;
}

// --------------------------------------------------------------------------
// Interrupts callback functions:

void Serial::TxCpltCallback(void)
{   
  // remove the bytes we just sent
  stream.removeFrontTxBuffer(_txBufferSize2Transmitting);

  _txBufferSize2Transmitting = 0;

  // immediately start next chunk if exists
  kickTx();
}

void Serial::RxCpltCallback(void)
{
  stream.pushBackRxBuffer(&_rxBuffer, 1);
  HAL_UART_Receive_IT(_huart, (uint8_t*)&_rxBuffer, 1);
}

// ------------------------------------------------------------------------
// print/println methods:

uint16_t Serial::print(const char* data)
{
  if(data == nullptr)
  {
    return 0;
  }

  uint16_t dataSize = strlen(data);

  return write((uint8_t*)data, dataSize);
}

uint16_t Serial::print(const std::string& data)
{
  return print(data.c_str());
}

uint16_t Serial::print(uint32_t data)
{
    char buffer[12]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = std::snprintf(buffer, sizeof(buffer), "%lu", static_cast<unsigned long>(data)); // Convert the uint8_t to string

    if(length <= 0 || length >= (int)sizeof(buffer))
    {
      return 0;
    }

    return print(buffer);
}

uint16_t Serial::print(int32_t data)
{
    char buffer[12]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = std::snprintf(buffer, sizeof(buffer), "%ld", static_cast<long>(data)); // Convert the uint8_t to string

    if(length <= 0 || length >= (int)sizeof(buffer))
    {
      return 0;
    }

    return print(buffer);
}

uint16_t Serial::print(uint64_t data)
{
    char buffer[30]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = std::snprintf(buffer, sizeof(buffer), "%llu", data); // Convert the uint8_t to string

    if(length <= 0 || length >= (int)sizeof(buffer))
    {
      return 0;
    }

    return print(buffer);
}

uint16_t Serial::print(int64_t data)
{
    char buffer[30]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = std::snprintf(buffer, sizeof(buffer), "%lld", data); // Convert the uint8_t to string

    if(length <= 0 || length >= (int)sizeof(buffer))
    {
      return 0;
    }

    return print(buffer);
}

uint16_t Serial::print(double data, uint8_t precision)
{
    if (precision > 10) 
    { // Limit precision to avoid excessive output
        precision = 10;
    }

    char buffer[30];  // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = std::snprintf(buffer, sizeof(buffer), "%.*lf", precision, data);  // Convert the uint8_t to string

     // Ensure snprintf was successful and print the result
    if (length <= 0 || length >= (int)sizeof(buffer)) 
    {
        return 0;
    } 

    return print(buffer);
}

uint16_t Serial::println(const char* data)
{
  size_t dataSize = print(data);
  print("\n");
  return dataSize + 1;
}

uint16_t Serial::println(const std::string& data)
{
  return print(data + "\n");
}

uint16_t Serial::println(uint32_t data)
{
  size_t dataSize = print(data);
  print("\n");
  return dataSize + 1;
}

uint16_t Serial::println(int32_t data)
{
  size_t dataSize = print(data);
  print("\n");
  return dataSize + 1;
}

uint16_t Serial::println(uint64_t data)
{
  size_t dataSize = print(data);
  print("\n");
  return dataSize + 1;
}

uint16_t Serial::println(int64_t data)
{
  size_t dataSize = print(data);
  print("\n");
  return dataSize + 1;
}

uint16_t Serial::println(double data, uint8_t precision)
{
  size_t dataSize = print(data, precision);
  print("\n");
  return dataSize + 1;
}

// -------------------------------------------------------------------------

bool Serial::_startTxIfIdle()
{
    if (_isTransmitting) return true;

    uint32_t n = stream.txContiguousSize();   // IMPORTANT
    if (n == 0) return true;                  // nothing to send

    if (n > 0xFFFF) n = 0xFFFF;               // HAL uses uint16_t size

    _txBufferSize2Transmitting = (uint16_t)n;
    _isTransmitting = true;

    const char* p = stream.txReadPtr();       // IMPORTANT (tail for ring)
    return (HAL_UART_Transmit_IT(_huart, (uint8_t*)p, _txBufferSize2Transmitting) == HAL_OK);
}

void Serial::_EnableIRQ(void)
{
  // HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);  // Set priority
  // HAL_NVIC_EnableIRQ(USART1_IRQn);          // Enable IRQ in NVIC
}

// ########################################################################################################