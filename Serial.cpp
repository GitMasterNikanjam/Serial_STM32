
// ######################################################################################
// Include Libraries:

#include "Serial.h"

Serial::Serial()
{
	parameters.BAUDRATE = 9600;
  parameters.INSTANCE = nullptr;
}


bool Serial::init(void)
{
  if(_checkParameters() == false)
  {
    return false;
  }

	_huart->Instance = parameters.INSTANCE;
  _huart->Init.BaudRate = parameters.BAUDRATE;
  _huart->Init.WordLength = UART_WORDLENGTH_8B;
  _huart->Init.StopBits = UART_STOPBITS_1;
  _huart->Init.Parity = UART_PARITY_NONE;
  _huart->Init.Mode = UART_MODE_TX_RX;
  _huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  _huart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(_huart) != HAL_OK)
  {
    errorMessage = "Error Serial: HAL_UAER_Init() is not succeeded.";
    return false;
		// __disable_irq();
		// while (1)
		// {
		// }
  }
	
	return true;
}

bool Serial::_checkParameters(void)
{
  bool state = ( (parameters.BAUDRATE == 9600) || (parameters.BAUDRATE == 115200) ) && 
               (parameters.INSTANCE != nullptr);

  if(state == false)
  {
    errorMessage = "Error Serial: One or some parameters are not correct.";
    return false;
  }

  return true;
}

void Serial::print(std::string data)
{
  HAL_UART_Transmit(_huart, (uint8_t*)data.c_str(), data.size(), 1000);
}