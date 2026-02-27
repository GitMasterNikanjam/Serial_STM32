#pragma once

/**
 * @file Serial.h
 * @brief Arduino-like UART serial wrapper built on STM32Cube HAL.
 *
 * This header declares the ::Serial class, which combines:
 * - STM32Cube HAL UART (blocking / interrupt modes; DMA reserved for future use)
 * - A Stream-based TX/RX software buffer (see Stream.h)
 *
 * MCU selection
 * -------------
 * This header expects a `mcu_select.h` next to it that defines exactly one MCU family:
 * @code{.c}
 * // mcu_select.h
 * // #define STM32F1
 * // #define STM32F4
 * // #define STM32H7
 * @endcode
 *
 * @note `mcu_select.h` is not part of this library file; it must be provided by the user/project.
 */

#include "mcu_select.h"

// ##############################################################################################
// STM32 HAL includes (selected by mcu_select.h)

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#endif

#include <string>				// Provides the std::string class for working with dynamic strings in C++
#include "Stream.h"

// ##############################################################################################
// Public macros (UART programming mode)

/**
 * @def PROGRAM_MODE_BLOCK
 * @brief Blocking mode (HAL_UART_Transmit / HAL_UART_Receive).
 */
#ifndef PROGRAM_MODE_BLOCK
	#define PROGRAM_MODE_BLOCK       	0
#endif

/**
 * @def PROGRAM_MODE_INTERRUPT
 * @brief Interrupt mode (HAL_UART_Transmit_IT / HAL_UART_Receive_IT).
 *
 * @note When using interrupt mode, this class requires **ring** buffers in ::Stream for the
 *       corresponding direction (TX and/or RX). Linear buffers rely on memmove() and can corrupt
 *       data when used concurrently with HAL interrupt transfers.
 */
#ifndef PROGRAM_MODE_INTERRUPT
	#define PROGRAM_MODE_INTERRUPT		1
#endif

/**
 * @def PROGRAM_MODE_DMA
 * @brief DMA mode (reserved).
 *
 * DMA mode is declared for API completeness but is currently not implemented in Serial.cpp.
 * Calls that request DMA may fail and/or set ::Serial::errorMessage accordingly.
 */
#ifndef PROGRAM_MODE_DMA
	#define PROGRAM_MODE_DMA			2
#endif

// ##############################################################################################

/**
 * @class Serial
 * @brief UART serial port helper with optional TX/RX software buffering.
 *
 * The interface is inspired by Arduino's `HardwareSerial` (e.g. `begin()`, `available()`, `read()`,
 * `print()`), but it targets STM32Cube HAL.
 *
 * Default configuration
 * ---------------------
 * - TX mode: ::PROGRAM_MODE_BLOCK
 * - RX mode: ::PROGRAM_MODE_INTERRUPT
 * - Baud rate: 9600
 *
 * Interrupt callbacks
 * -------------------
 * If you use interrupt-based RX/TX, connect the HAL callbacks to the matching methods:
 * @code{.c}
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *   if (huart == Serial1.getUart()) { Serial1.TxCpltCallback(); }
 * }
 *
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *   if (huart == Serial1.getUart()) { Serial1.RxCpltCallback(); }
 * }
 * @endcode
 *
 * @warning In interrupt mode you must provide ring buffers via setTxBuffer()/setRxBuffer()
 *          (or the constructor). ::Serial::begin() will fail if interrupt mode is selected but the
 *          corresponding Stream buffer type is not ::BUFFER_RING or its size is < 2.
 */
class Serial
{
	public:
		
		/**
		 * @brief Human-readable description of the last error.
		 *
		 * The string is always null-terminated and truncated to fit this buffer.
		 *
		 * @warning Keep messages shorter than 32 characters including the terminator.
		 */
		char errorMessage[32];
		
		/**
		 * @brief Construct a Serial instance and optionally attach Stream buffers.
		 *
		 * @param txBuffer        Pointer to TX buffer storage (may be nullptr).
		 * @param txBufferSize    Size of the TX buffer in bytes.
		 * @param rxBuffer        Pointer to RX buffer storage (may be nullptr).
		 * @param rxBufferSize    Size of the RX buffer in bytes.
		 * @param txType          TX buffer type (see ::BufferType in Stream.h).
		 * @param rxType          RX buffer type (see ::BufferType in Stream.h).
		 *
		 * @note Buffers can be configured later using setTxBuffer()/setRxBuffer().
		 */
		Serial(char* txBuffer = nullptr, size_t txBufferSize = 0, char* rxBuffer = nullptr, size_t rxBufferSize = 0, BufferType txType = BUFFER_LINEAR, BufferType rxType = BUFFER_LINEAR);

		/**
		 * @brief Initialize the UART peripheral and start RX/TX according to the selected modes.
		 *
		 * @param huart     Pointer to a pre-initialized HAL UART handle.
		 * @param baudRate  UART baud rate. Supported values: 9600, 57600, 115200.
		 * @return true on success, false otherwise (see ::errorMessage).
		 *
		 * @note The HAL UART handle must be configured (instance, word length, stop bits, parity, etc.)
		 *       before calling begin(); this function updates `huart->Init.BaudRate` and calls
		 *       HAL_UART_Init().
		 *
		 * @warning If RX mode is ::PROGRAM_MODE_INTERRUPT, begin() will arm an initial
		 *          HAL_UART_Receive_IT() for 1 byte and continuously re-arm it in RxCpltCallback().
		 * @warning If TX mode is ::PROGRAM_MODE_INTERRUPT, begin() will try to start an initial
		 *          interrupt-driven transmit if TX data is already queued.
		 */
		bool begin(UART_HandleTypeDef* huart, unsigned long baudRate = 9600);

		/**
		 * @brief Initialize the UART peripheral using the handle previously set via setUart().
		 *
		 * @param baudRate  UART baud rate. Supported values: 9600, 57600, 115200.
		 * @return true on success, false otherwise (see ::errorMessage).
		 *
		 * @warning This overload requires a valid UART handle to have been set earlier with setUart().
		 */
		bool begin(unsigned long baudRate = 9600);

		/**
		 * @brief Set the HAL UART handle used by this Serial instance.
		 * @param huart Pointer to a HAL UART handle (may be nullptr).
		 *
		 * @note The Serial instance does not take ownership of the handle.
		 */
		void setUart(UART_HandleTypeDef* huart);

		/**
		 * @brief Get the current HAL UART handle.
		 * @return Pointer to the HAL UART handle, or nullptr if not set.
		 */
		UART_HandleTypeDef* getUart(void) {return _huart;};

		/**
		 * @brief Set UART transmit (TX) mode.
		 *
		 * @param mode One of ::PROGRAM_MODE_BLOCK, ::PROGRAM_MODE_INTERRUPT, ::PROGRAM_MODE_DMA.
		 * @return true if the value is accepted; false if @p mode is invalid.
		 *
		 * @note Set the mode before calling begin().
		 * @warning Interrupt TX requires a ring TX buffer (see setTxBuffer()).
		 */
		bool setTxMode(uint8_t mode);

		/**
		 * @brief Set UART receive (RX) mode.
		 *
		 * @param mode One of ::PROGRAM_MODE_BLOCK, ::PROGRAM_MODE_INTERRUPT, ::PROGRAM_MODE_DMA.
		 * @return true if the value is accepted; false if @p mode is invalid.
		 *
		 * @note Set the mode before calling begin().
		 * @warning Interrupt RX requires a ring RX buffer (see setRxBuffer()).
		 */
		bool setRxMode(uint8_t mode);

		/**
		 * @brief Configure the TX software buffer used by the internal ::Stream.
		 *
		 * @param txBuffer      Pointer to TX buffer storage.
		 * @param txBufferSize  Size of the TX buffer in bytes.
		 * @param txType        Buffer type (e.g. ::BUFFER_LINEAR or ::BUFFER_RING).
		 *
		 * @warning If TX mode is ::PROGRAM_MODE_INTERRUPT, @p txType must be ::BUFFER_RING and the
		 *          buffer size must be at least 2.
		 */
		void setTxBuffer(char* txBuffer, uint16_t txBufferSize, BufferType txType = BUFFER_LINEAR);

		/**
		 * @brief Configure the RX software buffer used by the internal ::Stream.
		 *
		 * @param rxBuffer      Pointer to RX buffer storage.
		 * @param rxBufferSize  Size of the RX buffer in bytes.
		 * @param rxType        Buffer type (e.g. ::BUFFER_LINEAR or ::BUFFER_RING).
		 *
		 * @warning If RX mode is ::PROGRAM_MODE_INTERRUPT, @p rxType must be ::BUFFER_RING and the
		 *          buffer size must be at least 2.
		 */
		void setRxBuffer(char* rxBuffer, uint16_t rxBufferSize, BufferType rxType = BUFFER_LINEAR);

		/**
		 * @brief Set both TX and RX buffer types on the internal ::Stream.
		 *
		 * This is a convenience wrapper around Stream::setBufferTypes().
		 *
		 * @param txType TX buffer type.
		 * @param rxType RX buffer type.
		 */
		void setBufferTypes(BufferType txType, BufferType rxType);

		/**
		 * @brief Set the maximum time to wait for blocking operations (in milliseconds).
		 * @param timeout Timeout in milliseconds, or HAL_MAX_DELAY to wait indefinitely.
		 *
		 * @note This value is used by blocking-mode HAL calls (e.g. HAL_UART_Transmit / HAL_UART_Receive)
		 *       and by helpers like find()/findUntil() that implement timeouts via HAL_GetTick().
		 */
	    void setTimeout(unsigned long timeout);  
  		
		/**
		 * @brief Get the timeout configured by setTimeout().
		 * @return Timeout in milliseconds, or HAL_MAX_DELAY.
		 */
		unsigned long getTimeout(void) { return _timeout; }

		/**
		 * @brief Number of bytes available in the RX software buffer.
		 *
		 * @return Count of bytes currently buffered and ready to read.
		 *
		 * @note This count reflects the internal Stream RX buffer (typically filled in RxCpltCallback()).
		 */
		uint16_t available(void);

		/**
		 * @brief Number of bytes that can be queued for TX without blocking.
		 *
		 * @return Free space in the TX software buffer when using interrupt TX; 0 in blocking mode.
		 *
		 * @note In ::PROGRAM_MODE_BLOCK, write() transmits directly through HAL and may block on the
		 *       peripheral, so this function returns 0 (Arduino-style "non-blocking capacity" semantics).
		 */
		size_t availableForWrite(void);

		/**
		 * @brief Clear the internal TX software buffer (does not cancel an active HAL transfer).
		 */
		void clearTxBuffer();

		/**
		 * @brief Clear the internal RX software buffer.
		 */
		void clearRxBuffer();

		/**
		 * @brief Remove bytes from the front of the TX buffer.
		 * @param dataSize Number of bytes to remove.
		 * @return true on success, false if fewer than @p dataSize bytes are available.
		 */
		bool removeFrontTxBuffer(uint32_t dataSize = 1);

		/**
		 * @brief Remove bytes from the front of the RX buffer.
		 * @param dataSize Number of bytes to remove.
		 * @return true on success, false if fewer than @p dataSize bytes are available.
		 */
		bool removeFrontRxBuffer(uint32_t dataSize = 1);

		/**
		 * @brief Peek the next received byte without removing it from the RX buffer.
		 * @return The next byte (0..255), or -1 if no data is available.
		 */
		int16_t peek(void);

		/**
		 * @brief Read one byte from the RX buffer.
		 * @return The next byte (0..255), or -1 if no data is available.
		 *
		 * @note This function reads from the internal RX software buffer. In blocking RX mode, prefer
		 *       readBytes() or HAL_UART_Receive().
		 */
		int16_t read(void);

		/**
		 * @brief Read up to @p length bytes into @p buffer.
		 *
		 * @param buffer Destination buffer.
		 * @param length Maximum number of bytes to read.
		 * @return Number of bytes actually read (0 on timeout / error).
		 *
		 * Behavior by RX mode:
		 * - ::PROGRAM_MODE_BLOCK: reads exactly @p length bytes using HAL_UART_Receive() (or returns 0).
		 * - ::PROGRAM_MODE_INTERRUPT: reads from the internal RX buffer up to the currently available
		 *   bytes (min(available(), @p length)).
		 * - ::PROGRAM_MODE_DMA: not implemented (returns 0 and sets ::errorMessage).
		 */
		size_t readBytes(char* buffer, size_t length);

		/**
		 * @brief Read bytes into @p buffer until @p character is encountered, @p length is reached, or timeout occurs.
		 *
		 * The terminator character is not copied into @p buffer. The resulting buffer is always
		 * null-terminated.
		 *
		 * @param character Terminator to stop at.
		 * @param buffer    Destination buffer.
		 * @param length    Size of @p buffer in bytes (including space for the null terminator).
		 * @return Number of bytes copied (excluding the null terminator).
		 *
		 * @warning This function consumes bytes via read() and therefore depends on the internal RX buffer.
		 *          It is intended for interrupt RX mode.
		 */
		size_t readBytesUntil(char character, char* buffer, size_t length);

		/**
		 * @brief Read all currently buffered RX data into @p buffer.
		 *
		 * @param buffer     Destination buffer.
		 * @param maxLength  Maximum bytes to copy.
		 * @return Number of bytes copied.
		 *
		 * @note Data is removed from the RX buffer.
		 */
		size_t readAll(char* buffer, size_t maxLength);
		
		#if defined(__linux__)
			/**
			 * @brief Read and return all currently buffered RX data as a std::string.
			 * @return String containing all buffered bytes (may be empty).
			 *
			 * @note Data is removed from the RX buffer.
			 */
			std::string readAll(void);
		#endif

		/**
		 * @brief TX transfer complete callback (interrupt mode).
		 *
		 * Call this from your HAL_UART_TxCpltCallback() when the callback corresponds to this instance.
		 * It advances the TX buffer and starts the next chunk if more data is queued.
		 */
		void TxCpltCallback(void);

		/**
		 * @brief RX transfer complete callback (interrupt mode).
		 *
		 * Call this from your HAL_UART_RxCpltCallback() when the callback corresponds to this instance.
		 * It pushes the received byte into the RX buffer and re-arms HAL_UART_Receive_IT() for the next byte.
		 */
		void RxCpltCallback(void);

		/**
		 * @brief Search the RX stream for a byte sequence.
		 *
		 * Consumes bytes from the RX buffer until:
		 * - the sequence @p target is found (returns true), or
		 * - the operation times out (returns false).
		 *
		 * @param target Byte sequence to search for.
		 * @param length Number of bytes in @p target.
		 * @return true if found, false on timeout or invalid input.
		 *
		 * @note Timeout is controlled by setTimeout().
		 */
		bool find(const char* target, size_t length);

		/**
		 * @brief Convenience overload of find(const char*, size_t) for std::string.
		 */
		bool find(const std::string& target);

		/**
		 * @brief Search the RX stream for a byte sequence, aborting early on a terminator character.
		 *
		 * Consumes bytes from the RX buffer until:
		 * - the sequence @p target is found (returns true), or
		 * - @p terminate is read (returns false), or
		 * - the operation times out (returns false).
		 *
		 * @param target    Byte sequence to search for.
		 * @param length    Number of bytes in @p target.
		 * @param terminate Terminator character that aborts the search.
		 * @return true if found, false otherwise.
		 */
		bool findUntil(const char* target, size_t length, const char terminate);

		/**
		 * @brief Convenience overload of findUntil(const char*, size_t, char) for std::string.
		 */
		bool findUntil(const std::string target, const char terminate);

		/**
		 * @brief Wait for all queued TX data to be transmitted.
		 *
		 * This waits until both:
		 * - the internal TX buffer is empty, and
		 * - the HAL UART state indicates it is ready (no active transfer).
		 *
		 * @param timeoutMs Maximum time to wait in milliseconds.
		 * @return true if transmission completed within the timeout, false otherwise.
		 */
		bool flush(uint32_t timeoutMs);

		/**
		 * @brief Write one byte.
		 * @param data Byte to write.
		 * @return Number of bytes written (1) on success, 0 on failure.
		 */
		uint16_t write(uint8_t data);

		/**
		 * @brief Write a byte array to the serial port.
		 *
		 * @param data   Pointer to data to send.
		 * @param length Number of bytes to send.
		 * @return Number of bytes written on success; 0 on failure.
		 *
		 * Behavior by TX mode:
		 * - ::PROGRAM_MODE_BLOCK: calls HAL_UART_Transmit() and may block up to setTimeout().
		 * - ::PROGRAM_MODE_INTERRUPT: enqueues data in the TX buffer and starts an interrupt-driven
		 *   transfer if the peripheral is idle.
		 * - ::PROGRAM_MODE_DMA: not implemented (returns 0 and sets ::errorMessage).
		 */
		uint16_t write(uint8_t* data, uint16_t length);

		/**
		 * @brief Print a null-terminated C-string.
		 * @param data Text to send (must be null-terminated).
		 * @return Number of characters written, or 0 on failure.
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
		
		/// @brief HAL UART handle used for all low-level operations (not owned).
		UART_HandleTypeDef *_huart;

		/**
		 * @brief Internal Stream instance managing TX/RX buffers.
		 * @note Stream provides the buffer types (linear/ring) and push/pop helpers.
		 */
		Stream stream;

		/// @brief Single-byte staging buffer used with HAL_UART_Receive_IT().
		char _rxBuffer;

		/// @brief Number of bytes currently being transmitted by HAL (interrupt TX).
		volatile uint16_t _txBufferSize2Transmitting;

		/// @brief Timeout used for blocking operations and helper methods (ms).
		unsigned long _timeout;

		/// @brief UART baud rate last passed to begin().
		unsigned long _baudRate;

		/// @brief Current TX mode (one of ::PROGRAM_MODE_*).
		volatile uint8_t _txMode;

		/// @brief Current RX mode (one of ::PROGRAM_MODE_*).
		volatile uint8_t _rxMode;

		/// @brief Indicates whether a transmit operation is considered active.
		volatile bool _isTransmitting;

		/// @brief Indicates whether reception via interrupt has been started.
		volatile bool _isReceiving;

		/**
		 * @brief Start an interrupt-driven TX if the UART is idle.
		 * @return true if a transfer was started or the UART is busy; false on error.
		 */
		bool _startTxIfIdle(void);

		/**
		 * @brief Enable UART interrupt in NVIC (currently a stub).
		 *
		 * NVIC configuration is highly application-specific (IRQ number and priority),
		 * therefore this function is intentionally left as a placeholder.
		 */
		void _EnableIRQ(void);

		/**
		 * @brief Try to kick off (or continue) an interrupt-driven TX transfer.
		 *
		 * If the UART is idle and there is queued TX data, this will start a HAL_UART_Transmit_IT()
		 * for the next contiguous chunk in the TX buffer.
		 *
		 * @return true on success (or if nothing needs to be done), false on failure.
		 */
		bool kickTx();
};






