
// ######################################################################################
// Include Libraries:

#include "Serial.h"

Serial::Serial()
{
	_baudRate = 9600;
  _huart = nullptr;
  _timeout = HAL_MAX_DELAY;
}


bool Serial::begin(UART_HandleTypeDef* huart, unsigned long baudRate)
{
  bool state = ( (baudRate == 9600) || (baudRate == 115200) ) && 
               (huart != nullptr);

  if(state == false)
  {
    errorMessage = "Error Serial: One or some parameters are not correct.";
    return false;
  }

  _huart = huart;
  _baudRate = baudRate;

  _huart->Init.BaudRate = _baudRate;

  if (HAL_UART_Init(_huart) != HAL_OK)
  {
    errorMessage = "Error Serial: HAL_UAER_Init() is not succeeded.";
    return false;
  }
	
	return true;
}

void Serial::setTimeout(unsigned long timeout)
{
  _timeout = timeout;
}

size_t Serial::print(const std::string& data)
{
  return print(data.c_str());
}

size_t Serial::println(const std::string& data)
{
  return print(data + "\n");
}

size_t Serial::print(const char* data)
{
  if(HAL_UART_Transmit(_huart, (uint8_t*)data, strlen(data), _timeout) == HAL_OK)
  {
    return 0;
  }

  return 0;
}

size_t Serial::println(const char* data)
{
  print(data);
  print("\n");
  return 0;
}

size_t Serial::print(uint32_t data)
{
    char buffer[11]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = snprintf(buffer, sizeof(buffer), "%d", data); // Convert the uint8_t to string
  
    return print(buffer);
}

size_t Serial::print(int32_t data)
{
    char buffer[11]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = snprintf(buffer, sizeof(buffer), "%d", data); // Convert the uint8_t to string
  
    return print(buffer);
}

size_t Serial::print(double data, int = 2)
{
    char buffer[30]; // Buffer to hold the ASCII representation of the number (up to 3 digits + null terminator)
    int length = snprintf(buffer, sizeof(buffer), "%lf", data); // Convert the uint8_t to string
  
    return print(buffer);
}