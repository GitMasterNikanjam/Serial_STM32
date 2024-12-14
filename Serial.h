#ifndef SERIAL_H
#define SERIAL_H

// Define the target MCU family here
// #define STM32F4
#define STM32F1
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
#include <stdio.h>

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
		 * @brief Default constructor. Init some variables and parameters.
		 */
		Serial();

		bool begin(UART_HandleTypeDef* huart, unsigned long baudRate);
    
        void end();

		int available(void);

		int peek(void);

		int read(void);

		int availableForWrite(void);

		void flush(void);

		size_t write(uint8_t);
	
		/**
		 * @brief Set maximum milliseconds to wait for stream data, default is HAL_MAX_DELAY
		 */
	    void setTimeout(unsigned long timeout);  
  		
		unsigned long getTimeout(void) { return _timeout; }

		size_t print(const std::string& data);
		size_t print(const char* data);
		size_t print(uint32_t data);
		size_t print(int32_t data);
		size_t print(double, int = 2);

		size_t println(const std::string& data);
		size_t println(const char* data);
		size_t println(uint32_t data);
		size_t println(int32_t data);
		size_t println(double, int = 2);

	private:
		
		UART_HandleTypeDef *_huart;

		unsigned long _timeout;

		unsigned long _baudRate;
};


#endif



