#pragma once

// ##############################################################################################
// MCU Select:

#include "mcu_select.h"

/*
    If there is not exist mcu_select.h at beside of this header file, Create it and put this bellow following content. 
    Then select your desired MCU that want work with.
*/
// ----------------------------------------------------------------
// mcu_select.h file:

// Define the target MCU family here
// Uncomment the desired MCU family definition below:

// #define STM32F1
// #define STM32F4
// #define STM32H7

// ----------------------------------------------------------------
// ##############################################################################################
// Include libraries:

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#endif

#include <string>
#include "Stream.h"
// #include <stdio.h>			// Standard Input/Output library

// ##############################################################################################
// Define Public Macros:

#ifndef PROGRAM_MODE_BLOCK
	#define PROGRAM_MODE_BLOCK       	0
#endif

#ifndef PROGRAM_MODE_INTERRUPT
	#define PROGRAM_MODE_INTERRUPT		1
#endif

#ifndef PROGRAM_MODE_DMA
	#define PROGRAM_MODE_DMA			2
#endif

// ##############################################################################################

/**
 * @brief @class Serial
 * @brief This class can handle data transmit and receive for UART communication ports.
 * @note - Default transmitting mode is blocking mode.
 * @note - Default recieving mode is interrupt mode.
 * @note - Default baudrate is 9600.
 */
class Serial
{
	public:
		
		/**
		 * @brief The last error occurred for the object.
		 * @warning Message length must be lower than 32 characters.
		 *  */ 
		char errorMessage[32];
		
		/**
		 * @brief Default constructor. Init some variables and parameters to Default value.
		 * @note - Default transmitting mode is blocking mode.
	     * @note - Default recieving mode is interrupt mode.
	     * @note - Default baudrate is 9600.
		 * @param txBuffer: is the buffer for serial transmitting.
		 * @param rxBuffer: is the buffer for serial recieving.
		 */
		Serial(char* txBuffer = nullptr, size_t txBufferSize = 0, char* rxBuffer = nullptr, size_t rxBufferSize = 0);

		/**
		 * @brief Sets the data rate in bits per second (baud) for serial data transmission. 
		 * For communicating with Serial Monitor, make sure to use one of the baud rates 9600, 56700 or 115200.
		 * @param huart is The HAL UART handle pointer. 
		 * @param baudRate is the UART speed baudrate. It can just be 9600, 57600 or 115200.
		 * @note For the Serial object to function correctly, the HAL UART (huart) must be set and configured beforehand.
		 * @return true if succeeded.
		 */
		bool begin(UART_HandleTypeDef* huart, unsigned long baudRate = 9600);

		/**
		 * @brief Sets the data rate in bits per second (baud) for serial data transmission. 
		 * For communicating with Serial Monitor, make sure to use one of the baud rates 9600, 56700 or 115200.
		 * @param baudRate is the UART speed baudrate. It can just be 9600, 57600 or 115200.
		 * @note For the Serial object to function correctly, the HAL UART (huart) must be set and configured beforehand.
		 * @return true if succeeded.
		 */
		bool begin(unsigned long baudRate = 9600);

		/**
		 * @brief Set the HAL UART handle pointer.
		 */
		void setUart(UART_HandleTypeDef* huart);

		/**
		 * @brief Set UART transmit mode that can be Block mode, Interrupt mode, DMA mode.
		 * @param mode: Can be 0: Block mode, 1: Interrupt mode, 2: DMA mode.
		 * @return true if succeeded.
		 * @warning TxMode should be set before calling the begin() method.
		 */
		bool setTxMode(uint8_t mode);

		/**
		 * @brief Set UART receive mode that can be Block mode, Interrupt mode, DMA mode.
		 * @param mode: Can be 0: Block mode, 1: Interrupt mode, 2: DMA mode.
		 * @return true if succeeded.
		 * @warning RxMode should be set before calling the begin() method.
		 */
		bool setRxMode(uint8_t mode);

		/**
		 * @brief Set transmit buffer.
		 * @param txBuffer: Transmit buffer pointer.
		 * @param txBufferSize: Transmit buffer size.
		 */
		void setTxBuffer(char* txBuffer, uint16_t txBufferSize);

		/**
		 * @brief Set receive buffer.
		 * @param rxBuffer: Recieve buffer pointer.
		 * @param rxBufferSize: Recieve buffer size.
		 */
		void setRxBuffer(char* rxBuffer, uint16_t rxBufferSize);
		
		/**
		 * @brief Set maximum milliseconds to wait for stream data, default is HAL_MAX_DELAY
		 * @note Timeout used just in blocking mode.
		 */
	    void setTimeout(unsigned long timeout);  
  		
		/**
		 * @brief Get maximum milliseconds to wait for stream data, default is HAL_MAX_DELAY
		 * @note Timeout used just in blocking mode.
		 */
		unsigned long getTimeout(void) { return _timeout; }

		/**
		 * @brief Get the number of bytes (characters) available for reading from the serial port. 
		 * This is data that’s already arrived and stored in the serial receive buffer
		 */
		uint16_t available(void);

		/**
		 * @brief Get the number of bytes (characters) available for writing in the serial buffer without blocking the write operation.
		 */
		size_t availableForWrite(void);

		/**
		 * @brief Clear all data on the TxBuffer.
		 */
		void clearTxBuffer();

		/**
		 * @brief Clear all data on the RxBuffer.
		 */
		void clearRxBuffer();

		/**
		 * @brief Remove certain number elements from front of TX buffer.
		 * @return true if succeeded.
		 * @note - Error code be 1 if: "Not enough data in the buffer to remove"
		 *  */
		bool removeFrontTxBuffer(uint32_t dataSize = 1);

		/**
		 * @brief Remove certain number elements from front of RX buffer.
		 * @return true if succeeded.
		 * @note - Error code be 1 if: "Not enough data in the buffer to remove"
		 *  */
		bool removeFrontRxBuffer(uint32_t dataSize = 1);

		/**
		 * @brief Returns the next byte (character) of incoming serial data without removing it from the internal serial buffer. 
		 * That is, successive calls to peek() will return the same character, as will the next call to read().
		 * @return The first byte of incoming serial data available (or -1 if no data is available).
		 */
		int16_t peek(void);

		/**
		 * @brief Reads incoming serial data.
		 * @return The first byte of incoming serial data available (or -1 if no data is available).
		 * @note It removes data from buffer after read operation.
		 */
		int16_t read(void);

		/**
		 * @brief reads characters from the serial port into a buffer. 
		 * The function terminates if the determined length has been read, or it times out.
		 * returns the number of characters placed in the buffer. A 0 means no valid data was.
		 * @param buffer: the buffer to store the bytes in. Allowed data types: array of char or byte.
		 * @param length: the number of bytes to read.
		 * @return The number of bytes placed in the buffer.
		 */
		size_t readBytes(char* buffer, size_t length);

		/**
		 * @brief Serial.readBytesUntil() reads characters from the serial buffer into an array. 
		 * The function terminates (checks being done in this order) if the determined length has been read, 
		 * if it times out (see Serial.setTimeout()), or if the terminator character is detected 
		 * (in which case the function returns the characters up to the last character before the supplied terminator). 
		 * The terminator itself is not returned in the buffer.
		 * @param character: the character to search for.
		 * @param buffer: the buffer to store the bytes in. Allowed data types: array of char or byte.
		 * @param length: the number of bytes to read.
		 * @return The number of bytes placed in the buffer.
		 * @warning The terminator character is discarded from the serial buffer, 
		 * unless the number of characters read and copied into the buffer equals length.
		 */
		size_t readBytesUntil(char character, char* buffer, size_t length);

		/**
		 * @brief Read all data on RXBuffer and store it in input buffer.
		 * @param maxLength: the max number of bytes to read.
		 * @return The number of bytes placed in the buffer.
		 */
		size_t readAll(char* buffer, size_t maxLength);
		
		/**
		 * @brief Read all data on RXBuffer and return to string data.
		 */
		std::string readAll(void);

		/**
		 * @brief Tx Transfer completed interrupt callbacks.
		 */
		void TxCpltCallback(void);

		/**
		 * @brief Rx Transfer completed interrupt callbacks.
		 */
		void RxCpltCallback(void);

		/**
		 * @brief reads data from the serial buffer until the target is found. The function returns true if target is found, 
		 * false if it times out.
		 * @param target: the string to search for.
		 * @param length:  length of the target.
		 */
		bool find(const char* target, size_t length);

		/**
		 * @brief reads data from the serial buffer until the target is found. The function returns true if target is found, 
		 * false if it times out.
		 * @param target: the string to search for.
		 */
		bool find(const std::string& target);

		/**
		 * @brief reads data from the serial buffer until a target string of given length or terminator string is found.
         * The function returns true if the target string is found, false if it times out.
		 * @param target: the string to search for.
		 * @param length:  length of the target.
		 * @param terminate: the terminal character in the search.
		 */
		bool findUntil(const char* target, size_t length, const char terminate);

		/**
		 * @brief reads data from the serial buffer until a target string of given length or terminator string is found.
         * The function returns true if the target string is found, false if it times out.
		 * @param target: the string to search for.
		 * @param terminate: the terminal character in the search.
		 */
		bool findUntil(const std::string target, const char terminate);

		/**
		 * @brief Waits for the transmission of outgoing serial data to complete.
		 */
		void flush(void);

		/**
		 * @brief Writes binary data to the serial port.
		 * @return The number of bytes written, though reading that number is optional.
		 */
		uint16_t write(uint8_t data);

		/**
		 * @brief Writes binary data to the serial port.
		 * @return The number of bytes written, though reading that number is optional.
		 */
		uint16_t write(uint8_t* data, uint16_t length);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t print(const char* data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t print(const std::string& data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t print(uint32_t data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t print(int32_t data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t print(uint64_t data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t print(int64_t data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t print(double data, uint8_t precision= 2);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @note It sends newline character automatically end of data.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t println(const char* data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @note It sends newline character automatically end of data.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t println(const std::string& data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @note It sends newline character automatically end of data.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t println(uint32_t data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @note It sends newline character automatically end of data.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t println(int32_t data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @note It sends newline character automatically end of data.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t println(uint64_t data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @note It sends newline character automatically end of data.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t println(int64_t data);

		/**
		 * @brief Prints data to the serial port as human-readable ASCII text. This command can take many forms. 
		 * Numbers are printed using an ASCII character for each digit. Floats are similarly printed as ASCII digits, 
		 * defaulting to two decimal places.
		 * @note It sends newline character automatically end of data.
		 * @return number of characters that written.
		 * @note Return value can be 0 or number of character of data. If any error for transmitting data occurred it returns 0.
		 */
		uint16_t println(double data, uint8_t precision= 2);

	private:
		
		/**
		 * @brief HAL UART handle pointer.
		 */
		UART_HandleTypeDef *_huart;

		/**
		 * @brief Stream object for manage receive and transmit data on UART communication.
		 * @note Stream object has advance buffer management for transmit and recieve data.
		 */
		Stream stream;

		/// @brief Rx buffer for recieve data.
		char _rxBuffer;

		/// @brief Tx buffer size that is transmitting.
		uint16_t _txBufferSize2Transmitting;

		/**
		 * @brief The maximum milliseconds to wait for stream data, default is HAL_MAX_DELAY
		 */
		unsigned long _timeout;

		/**
		 * @brief The baudrate- speed for UART communication. Default value is 9600.
		 * @note Its value can be just: 9600, 57600 or 115200.
		 */
		unsigned long _baudRate;

		/**
		 * @brief UART transmit mode that can be Block mode, Interrupt mode, DMA mode.
		 * @note - Can be 0: Block mode, 1: Interrupt mode, 3: DMA mode.
		 * @return true if succeeded.
		 */
		volatile uint8_t _txMode;

		/**
		 * @brief UART receive mode that can be Block mode, Interrupt mode, DMA mode.
		 * @note - Can be 0: Block mode, 1: Interrupt mode, 3: DMA mode.
		 * @return true if succeeded.
		 */
		volatile uint8_t _rxMode;

		/// @brief The flag indicate transmitting process is running or finished.
		volatile bool _isTransmitting;

		/// @brief The flag indicate receiving process is running or finished.
		volatile bool _isReceiving;

		void _EnableIRQ(void);
};






