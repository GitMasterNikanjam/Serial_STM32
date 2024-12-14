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

		void begin(unsigned long baud);
    
        void end();

		int available(void);

		int peek(void);

		int read(void);

		int availableForWrite(void);

		void flush(void);

		size_t write(uint8_t);

		void print(std::string data);
	
	    void setTimeout(unsigned long timeout);  // sets maximum milliseconds to wait for stream data, default is 1 second
  		
		unsigned long getTimeout(void) { return _timeout; }

		size_t print(const std::string &);
		size_t print(const char[]);
		size_t print(char);
		size_t print(unsigned char, int);
		size_t print(int, int);
		size_t print(unsigned int, int);
		size_t print(long, int);
		size_t print(unsigned long, int);
		size_t print(double, int = 2);

		size_t println(const std::string &s);
		size_t println(const char[]);
		size_t println(char);
		size_t println(unsigned char, int);
		size_t println(int, int);
		size_t println(unsigned int, int);
		size_t println(long, int);
		size_t println(unsigned long, int);
		size_t println(double, int = 2);
		size_t println(void);

	private:
		
		UART_HandleTypeDef *_huart;

		unsigned long _timeout;

		bool _checkParameters(void);
	
};


#endif



