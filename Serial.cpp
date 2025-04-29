
// ######################################################################################
// Include Libraries:

#include "Serial.h"

// ######################################################################################
// Serial class:

// -----------------------------------------------------------------------------------
// base and initial methods:

Serial::Serial(char* txBuffer, size_t txBufferSize, char* rxBuffer, size_t rxBufferSize)
{
	_baudRate = 9600;
  _huart = nullptr;
  _timeout = HAL_MAX_DELAY;
  _txMode = PROGRAM_MODE_BLOCK;
  _rxMode = PROGRAM_MODE_INTERRUPT;
  _isTransmitting = false;
  _isReceiving = false;
  setTxBuffer(txBuffer, txBufferSize);
  setRxBuffer(rxBuffer, rxBufferSize);
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
    sprintf(errorMessage, "Parameters");
    return false;
  }

  _huart = huart;
  _baudRate = baudRate;

  _huart->Init.BaudRate = _baudRate;

  if (HAL_UART_Init(_huart) != HAL_OK)
  {
    sprintf(errorMessage, "HAL_UAER_Init()");
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

void Serial::setTxBuffer(char* txBuffer, uint16_t txBufferSize)
{
  stream.setTxBuffer(txBuffer, txBufferSize);
}

void Serial::setRxBuffer(char* rxBuffer, uint16_t rxBufferSize)
{
  stream.setRxBuffer(rxBuffer, rxBufferSize);
}

bool Serial::setTxMode(uint8_t mode)
{
  if((mode != PROGRAM_MODE_BLOCK) && (mode != PROGRAM_MODE_INTERRUPT) && (mode != PROGRAM_MODE_DMA))
  {
    return false;
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
    size_t targetIndex = 0;  // Track the index in the target string
    
    // Loop to read characters from the serial buffer
    while (true) {
        int16_t data = read();  // Read one character from serial
        
        // If no data is available or read() fails, continue checking
        if (data == -1) {
            continue;
        }

        // Check if the current character matches the target character
        if (static_cast<char>(data) == target[targetIndex]) {
            targetIndex++;  // Move to the next character in the target string

            // If the entire target string is matched, return true
            if (targetIndex == length) {
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
  size_t length = stream.availableRx();
  return find(target.c_str(), length);
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
  switch(_rxMode)
  {
    case PROGRAM_MODE_BLOCK:
      if(HAL_UART_Receive(_huart, (uint8_t*)&buffer, length, _timeout) != HAL_OK)
      {
        return length;
      }
      else
      {
        return 0;
      }
    break;
    case PROGRAM_MODE_INTERRUPT:
      if(stream.availableRx() == 0)
      {
        return 0;
      }

      if(stream.popFrontRxBuffer(buffer, length) != true)
      {
        return 0;
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

// ---------------------------------------------------------------------------
// write, flush method

void Serial::flush(void)
{
  while(_txBufferSize2Transmitting > 0);
}

uint16_t Serial::write(uint8_t data)
{
  return write(&data, 1);
}

uint16_t Serial::write(uint8_t* data, uint16_t length)
{
  HAL_StatusTypeDef status;

  switch(_txMode)
  {
    case PROGRAM_MODE_BLOCK:
      status = HAL_UART_Transmit(_huart, data, length, _timeout);
    break;
    case PROGRAM_MODE_INTERRUPT:
      if(_isTransmitting == true)
      {
        stream.pushBackTxBuffer((char*)data, length);
      }
      else
      {
        _isTransmitting = true;
        status = HAL_UART_Transmit_IT(_huart, data, length);
      }
    break;
    case PROGRAM_MODE_DMA:

    break;
    default:
      return 0;
  }

  if(status != HAL_OK)
  {
    return 0;
  }

  return length;
}

// --------------------------------------------------------------------------
// Interrupts callback functions:

void Serial::TxCpltCallback(void)
{
  stream.removeFrontTxBuffer(_txBufferSize2Transmitting);
  _txBufferSize2Transmitting = stream.availableTx();
  if(_txBufferSize2Transmitting > 0)
  {
    _isTransmitting = true;
    HAL_UART_Transmit_IT(_huart, (uint8_t*)stream.getTxBuffer(), _txBufferSize2Transmitting);
    return;
  }
  _isTransmitting = false;
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
  HAL_StatusTypeDef status;
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
    int length = snprintf(buffer, sizeof(buffer), "%d", data); // Convert the uint8_t to string
  
    return print(buffer);
}

uint16_t Serial::print(int32_t data)
{
    char buffer[12]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = snprintf(buffer, sizeof(buffer), "%d", data); // Convert the uint8_t to string
  
    return print(buffer);
}

uint16_t Serial::print(uint64_t data)
{
    char buffer[30]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = snprintf(buffer, sizeof(buffer), "%llu", data); // Convert the uint8_t to string
  
    return print(buffer);
}

uint16_t Serial::print(int64_t data)
{
    char buffer[30]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = snprintf(buffer, sizeof(buffer), "%lld", data); // Convert the uint8_t to string
  
    return print(buffer);
}

uint16_t Serial::print(double data, uint8_t precision)
{
    if (precision > 10) 
    { // Limit precision to avoid excessive output
        precision = 10;
    }
    char buffer[30]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = snprintf(buffer, sizeof(buffer), "%.*lf", precision, data); // Convert the uint8_t to string
  
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
  size_t dataSize = print(data);
  print("\n");
  return dataSize + 1;
}

// -------------------------------------------------------------------------

void Serial::_EnableIRQ(void)
{
  // HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);  // Set priority
  // HAL_NVIC_EnableIRQ(USART1_IRQn);          // Enable IRQ in NVIC
}

// ########################################################################################################