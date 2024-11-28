#ifndef SERIAL_H
#define SERIAL_H

// Define the target MCU family here
#define STM32F4
// #define STM32F1
// #define STM32H7

// ##############################################################
// Include libraries:

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#endif

#include <string>

// ###############################################################

/**
	@class Serial
*/
class Serial
{
	public:
		
		/// @brief The last error occurred for the object.
		std::string errorMessage;

		/**
		 * @struct ParametersStructure
		 * @brief Parameters structure
		 */
		struct ParametersStructure
		{
			/**
			 * @brief Serial baude rate.
			 * @note The value can be: 9600, 115200
			 */
			uint32_t BAUDRATE;
			
			/**
			 * @brief Universal Synchronous Asynchronous Receiver Transmitter instance pointer
			 * @note The value can be for example: UART1, UART2M UART3, ... .
			 */
			USART_TypeDef *INSTANCE;
		}parameters;
		
		/**
		 * @brief Default constructor. Init some variables and parameters.
		 */
		Serial();
		
		bool init(void);

		void print(std::string data);
	
	private:
		
		UART_HandleTypeDef *_huart;

		bool _checkParameters(void);
	
};


#endif



