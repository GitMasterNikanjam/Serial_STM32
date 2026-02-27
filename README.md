# Serial (STM32Cube HAL) — Arduino-like UART wrapper

An Arduino-inspired `Serial` class built on **STM32Cube HAL UART**, with optional **software TX/RX buffering** via a `Stream` helper (see `Stream.h`). It provides familiar methods like `begin()`, `available()`, `read()`, `print()`, `println()`, plus a small set of convenience helpers (`readBytesUntil()`, `find()`, etc.).

> **Programming modes**
> - **Blocking**: `HAL_UART_Transmit()` / `HAL_UART_Receive()`
> - **Interrupt**: `HAL_UART_Transmit_IT()` / `HAL_UART_Receive_IT()`
> - **DMA**: declared for completeness, **not implemented yet** in `Serial.cpp`

---

## Features

- Simple Arduino-like API: `begin()`, `available()`, `read()`, `peek()`, `write()`, `print()/println()`
- Configurable TX/RX modes: blocking or interrupt
- TX/RX software buffers through `Stream` (linear or ring)
- Helper methods:
  - `readBytes()`, `readBytesUntil()`, `readAll()`
  - `find()`, `findUntil()`
  - `flush(timeoutMs)`
- Error reporting via `Serial::errorMessage`

---

## Supported MCU families

`Serial.h` expects you to select a HAL header by defining **exactly one** MCU family macro:

- `STM32F1` → `stm32f1xx_hal.h`
- `STM32F4` → `stm32f4xx_hal.h`
- `STM32H7` → `stm32h7xx_hal.h`

This selection is done via a small user-provided header:

### `mcu_select.h` (required)
Create a `mcu_select.h` next to `Serial.h` (or in your include path):

```c
// mcu_select.h
#define STM32F4
// or: #define STM32F1
// or: #define STM32H7
```

---

## Files / dependencies

At minimum you need:

* `Serial.h`
* `Serial.cpp`
* `Stream.h` (required by `Serial.h`) and its implementation (if separate)
* STM32Cube HAL UART enabled in your project (CubeMX or manual HAL init)

---

## Important note about buffers (read this if you use interrupts)

### Ring buffer requirement in interrupt mode

If you select `PROGRAM_MODE_INTERRUPT` for TX and/or RX, you **must** use a **ring buffer** in `Stream` for that direction.

`begin()` will **fail** (returns `false`) if:

* interrupt RX is enabled but RX buffer type is not `BUFFER_RING`, or size `< 2`
* interrupt TX is enabled but TX buffer type is not `BUFFER_RING`, or size `< 2`

Check `errorMessage` for the reason (e.g. `"Rx ring buf"`, `"Tx buf size"`).

---

## Quick start

### 1) Create a Serial instance with buffers

**Example A — RX interrupt (default), TX blocking (default TX mode)**
You still need a **ring RX buffer** because default RX mode is `PROGRAM_MODE_INTERRUPT`.

```cpp
#include "Serial.h"

extern UART_HandleTypeDef huart1;

// Provide only RX ring buffer
static char rxBuf[128];

Serial Serial1(
  nullptr, 0,                 // no TX buffer needed for blocking TX
  rxBuf, sizeof(rxBuf),
  BUFFER_LINEAR,              // TX buffer type doesn't matter for blocking TX
  BUFFER_RING                 // required for interrupt RX
);

void init_uart_serial()
{
  // Optional: Serial1.setTxMode(PROGRAM_MODE_BLOCK);        // default
  // Optional: Serial1.setRxMode(PROGRAM_MODE_INTERRUPT);    // default

  if (!Serial1.begin(&huart1, 115200))
  {
    // Serial1.errorMessage contains a short error string
    // e.g. "Rx ring buf", "Parameters", "HAL_UART_Init()"
    while (1) {}
  }
}
```

**Example B — TX + RX interrupt**
Provide **both** ring buffers and hook HAL callbacks.

```cpp
#include "Serial.h"

extern UART_HandleTypeDef huart1;

static char txBuf[256];
static char rxBuf[256];

// Ring buffers for both directions
Serial Serial1(txBuf, sizeof(txBuf), rxBuf, sizeof(rxBuf), BUFFER_RING, BUFFER_RING);

void init_uart_serial()
{
  Serial1.setTxMode(PROGRAM_MODE_INTERRUPT);
  Serial1.setRxMode(PROGRAM_MODE_INTERRUPT);

  if (!Serial1.begin(&huart1, 115200))
  {
    while (1) {}
  }
}
```

---

## Hooking HAL callbacks (required for interrupt mode)

If you use interrupt-driven RX/TX, forward the HAL callbacks to the instance:

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == Serial1.getUart()) { Serial1.TxCpltCallback(); }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == Serial1.getUart()) { Serial1.RxCpltCallback(); }
}
```

---

## Basic usage examples

### Printing

```cpp
Serial1.print("Value = ");
Serial1.println(1234);
Serial1.println(3.14159, 3);   // precision = 3
```

### Reading bytes

```cpp
if (Serial1.available() > 0)
{
  int c = Serial1.read();
  if (c >= 0) { /* ... */ }
}
```

### Read a line (until `\n`)

```cpp
char line[64];
size_t n = Serial1.readBytesUntil('\n', line, sizeof(line));
if (n > 0)
{
  // line is null-terminated
}
```

### Finding a token with timeout

```cpp
Serial1.setTimeout(1000);  // 1 second

if (Serial1.find("OK", 2))
{
  // found "OK" before timeout
}

if (Serial1.findUntil("DATA", '\n'))
{
  // found "DATA" before a newline arrived
}
```

---

## Configuration

### Baud rate

`begin()` only accepts these baud rates by default:

* `9600`
* `57600`
* `115200`

If you need others, adjust the check in `Serial.cpp` (`Serial::begin()`).

### Timeouts

* `setTimeout(ms)` controls blocking reads and helper methods (like `find()` and `readBytesUntil()`).
* Default timeout is `HAL_MAX_DELAY`.

### Flush

```cpp
bool ok = Serial1.flush(50); // wait up to 50ms for TX buffer + UART to become ready
```

---

## API overview (most-used)

* Initialization

  * `bool begin(UART_HandleTypeDef* huart, unsigned long baudRate = 9600);`
  * `bool begin(unsigned long baudRate = 9600);`
  * `void setUart(UART_HandleTypeDef* huart);`
* Modes / buffers

  * `bool setTxMode(uint8_t mode);`
  * `bool setRxMode(uint8_t mode);`
  * `void setTxBuffer(char* buf, uint16_t size, BufferType type);`
  * `void setRxBuffer(char* buf, uint16_t size, BufferType type);`
  * `void setBufferTypes(BufferType txType, BufferType rxType);`
* IO

  * `uint16_t available();`
  * `int16_t read();`
  * `int16_t peek();`
  * `uint16_t write(uint8_t data);`
  * `uint16_t write(uint8_t* data, uint16_t length);`
  * `uint16_t print(...)`, `uint16_t println(...)`
* Helpers

  * `size_t readBytes(...)`, `readBytesUntil(...)`, `readAll(...)`
  * `bool find(...)`, `bool findUntil(...)`
  * `bool flush(uint32_t timeoutMs);`
* Interrupt forwarding

  * `void TxCpltCallback();`
  * `void RxCpltCallback();`

> Note: `std::string readAll()` is only available when compiling with `__linux__` (see `Serial.h`).

---

## Troubleshooting

* **`begin()` returns false**

  * Check `Serial1.errorMessage`
  * Common causes:

    * `"Parameters"`: invalid baud rate or `huart == nullptr`
    * `"Rx ring buf"` / `"Tx ring buf"`: interrupt mode requires ring buffers
    * `"Rx buf size"` / `"Tx buf size"`: buffer size must be ≥ 2 in interrupt mode
    * `"HAL_UART_Init()"`: HAL init failed

* **No RX data in interrupt mode**

  * Ensure `HAL_UART_RxCpltCallback()` forwards to `Serial::RxCpltCallback()`
  * Ensure RX mode is `PROGRAM_MODE_INTERRUPT`
  * Ensure RX ring buffer exists and is large enough

* **TX stalls in interrupt mode**

  * Ensure `HAL_UART_TxCpltCallback()` forwards to `Serial::TxCpltCallback()`
  * Ensure TX ring buffer exists and TX mode is `PROGRAM_MODE_INTERRUPT`

---

## Notes / limitations

* DMA mode (`PROGRAM_MODE_DMA`) is currently **not implemented** in `Serial.cpp`.
* Interrupt mode relies on `Stream` buffer behavior. If your `Stream` implementation is not ISR-safe,
  protect buffer operations with critical sections.

---

## Contributing

PRs welcome: improvements to DMA support, broader baud rates, additional MCU families, or expanded examples/tests.

---

```

If you want, I can tailor the README to your repo layout (e.g., PlatformIO vs CubeIDE vs CMake) and add a “How to add to CubeMX project” section.
```
