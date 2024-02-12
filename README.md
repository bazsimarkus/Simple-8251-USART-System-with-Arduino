# Simple 8251 USART System

This circuit provides a straightforward test environment for an **8251 Universal Synchronous/Asynchronous Receiver-Transmitter (USART)** chip. Specifically, in this example, an *D8251AFC* chip is utilized, yet the software is designed to be forward compatible with all serial ICs based on the 8251 architecture.

This system serves as an introductory circuit to the world of UARTs/USARTs, aiming to minimize complexity while keeping the component count low. I created similar circuits using other popular UART/USART chips, like the 8250 and the 6850. Feel free to check them out!

## Breadboard Photo

![Breadboard Photo](https://raw.githubusercontent.com/bazsimarkus/Simple-8251-USART-System-with-Arduino/main/images/8251_breadboard.jpg)

## Components Needed

- 8251 USART chip
- Arduino board (Uno, Nano, both works, the important thing is that it should operate on 5V voltage)
- USB-TTL Serial converter for testing (FTDI will also suffice)

## How to use

1. Connect your Arduino board to the 8251 USART chip based on the included schematics.
2. Upload the 8251.ino source code to the Arduino board.
3. Connect the USB-TTL Serial converter to the PC and to the TxD/RxD pins of the 8251.
4. Open a serial terminal program (for example, RealTerm), select the COM port of your USB-Serial converter, and set up the terminal to use 9600 baud, 8-bit data, no parity, and 1 stop bit.
5. You should be receiving the string "TESTABCD" once every second.

## How it works

The Arduino sends 8-bit ASCII characters to the USART chip, then the chip converts it to serial data (9600 baud, 8-bit data, no parity, and 1 stop bit).
The parameters like the baud rate, parity bit, etc., are initialized by the Arduino at the beginning of the procedure.
The text to send is stored in the PROGMEM section of the microcontroller memory, allowing the Arduino to send long messages without any stack problems.
No external crystal is needed, as the program utilizes Timer1 and Timer2 of the ATmega328P microcontroller on the Arduino UNO board to generate clock signals, and achieve the desired standard 9600 Baud rate.

### Register Initialization for Basic 9600 Baud 8N1 Serial Transmission

To initialize the UART registers for communication, the Arduino program follows these steps:
  
  1. Set control pins:
     - Set CTSN, RESET, and CDN pins to LOW.
     - Set CSN, WRN, and RDN pins to HIGH.
     - Apply short delays between pin operations for stabilization.
  
  2. Reset the 8251 chip:
     - Set RESET pin to HIGH for a short duration.
     - Set RESET pin to LOW to reset the chip.
     - Apply short delays between reset operations.
  
  3. Write to the mode register:
     - Set CND pin to HIGH and CSN pin to LOW to select the chip for writing.
     - Write the desired mode configuration value (0b01001101) to the data bus.
     - Apply a short delay between setting up the data bus and writing.
     - Toggle the WRN pin to initiate the write operation.
     - Apply a medium delay after the write operation.
     - Toggle the WRN pin again to complete the write operation.
     - Set CND and CSN pins to LOW to deselect the chip.
  
  4. Write to the command register:
     - Set CND pin to HIGH and CSN pin to LOW to select the chip for writing.
     - Write the desired command configuration value (0b00000001) to the data bus.
     - Apply a short delay between setting up the data bus and writing.
     - Toggle the WRN pin to initiate the write operation.
     - Apply a medium delay after the write operation.
     - Toggle the WRN pin again to complete the write operation.
     - Set CND and CSN pins to LOW to deselect the chip.
  
  5. Indicate initialization completion by blinking the onboard LED.

For further register initialization details, refer to the `initUsart()` function.

## Schematic

![Schematic](https://github.com/bazsimarkus/Simple-8251-USART-System-with-Arduino/raw/main/images/8251_sch.png)

## 8251 Properties

The 8251 Universal Synchronous/Asynchronous Receiver/Transmitter (USART) is a popular integrated circuit commonly used in microcontroller-based systems for serial communication. Developed by Intel, it serves as a versatile interface between a microprocessor/microcontroller and serial communication peripherals such as modems, terminals, and other devices.

This chip supports both synchronous and asynchronous serial communication protocols, making it highly adaptable to various applications. It features a built-in baud rate generator, which enables precise timing control for communication speed adjustment. The 8251 also provides hardware support for data formatting, including start and stop bit generation, parity checking, and error detection, relieving the microcontroller of these tasks and simplifying the software overhead.

In this section, some hardware properties of the 8251 USART chip are provided.

### Pinout of the 8251 USART chip (DIP-28)

```
          ┌───────○───────┐
       D2 │1            28│ D1
       D3 │2            27│ D0
      RxD │3            26│ Vcc
      GND │4            25│ RxCN
       D4 │5            24│ DTRN
       D5 │6    8251A   23│ RTSN
       D6 │7    USART   22│ DSRN
       D7 │8            21│ RESET
     TxCN │9            20│ CLK
      WRN │10           19│ TxD
      CSN |11           18| TxEMPTY
      CDN |12           17| CTSN
      RDN |13           16| SYNDET/BD
    RxRDY |14           15| TxRDY
          └───────────────┘
```

### Pin Descriptions of the 8251 USART chip


  | Pin Number | Name      | Type   | Description                                |
  |------------|-----------|--------|--------------------------------------------|
  | 1          | D2        | In/Out | Data Bit 2                                 |
  | 2          | D3        | In/Out | Data Bit 3                                 |
  | 3          | RxD       | In     | Receive                                    |
  | 4          | GND       | -      | Ground                                     |
  | 5          | D4        | In/Out | Data Bit 4                                 |
  | 6          | D5        | In/Out | Data Bit 5                                 |
  | 7          | D6        | In/Out | Data Bit 6                                 |
  | 8          | D7        | In/Out | Data Bit 7                                 |
  | 9          | TxCN      | In     | Transmit Clock Input (Active Low)          |
  | 10         | WRN       | In     | Write (Active Low)                         |
  | 11         | CSN       | In     | Chip Select (Active Low)                   |
  | 12         | CDN       | In     | Command/Data Select                        |
  | 13         | RDN       | In     | Read (Active Low)                          |
  | 14         | RxRDY     | Out    | Read Register Ready                        |
  | 15         | TxRDY     | Out    | Transmitter Register Ready                 |
  | 16         | SYNDET/BD | In/Out | Sync Detect/Break Detect (See Data Sheet)  |
  | 17         | CTSN      | In     | Clear To Send (Active Low)                 |
  | 18         | TxEMPTY   | Out    | Transmitter Register Empty                 |
  | 19         | TxD       | Out    | Transmit Output                            |
  | 20         | CLK       | In     | Clock                                      |
  | 21         | RESET     | In     | Reset                                      |
  | 22         | DSRN      | In     | Data Set Ready (Active Low)                |
  | 23         | RTSN      | Out    | Request to Send (Active Low)               |
  | 24         | DTRN      | Out    | Data Terminal Ready (Active Low)           |
  | 25         | RXCN      | In     | Receive Clock (Active Low)                 |
  | 26         | Vcc       | -      | Positive Supply                            |
  | 27         | D0        | In/Out | Data Bit 0                                 |
  | 28         | D1        | In/Out | Data Bit 1                                 |

### Register Descriptions for 8251 USART

The chip contains seven user-visible registers, five of which can only be written, and two of which can only be read. These are the following:

- **Mode Register**: Sync/async operation mode and parameters
- **Command Register**: Enable/disable and error resetting
- **Sync 1 Character**: 8-bit dataword (sync mode only)
- **Sync 2 Character**: 8-bit dataword (sync mode only)
- **Transmit Buffer**: 8-bit register for outgoing data
- **Receive Buffer**: 8-bit register with incoming data
- **Status Register**: Several status and error bits

The meanings of the bits in the mode register are as follows:

- **D7D6**: Stop bits selection:
  - 11 = 2 stop bits
  - 10 = 1.5 stop bits
  - 01 = 1 stop bit
  - 00 = Invalid (at least 1 stop bit required)
- **D5D4**: Parity selection:
  - 11 = Even parity
  - 10 = Parity disabled
  - 01 = Odd parity
  - 00 = Parity disabled
- **D3D2**: Character length:
  - 11 = 8 data bits
  - 10 = 7 data bits
  - 01 = 6 data bits
  - 00 = 5 data bits
- **D1D0**: Mode / Baud-rate factor:
  - 11 = Async mode, 64x TXC/RXC prescaler
  - 10 = Async mode, 16x TXC/RXC prescaler
  - 01 = Async mode, no clock prescaler
  - 00 = Sync mode

For example, writing the binary value 11001101 or hex 0xCD to the mode register selects async mode without prescaler (D1=0 and D0=1), eight data bits (D3=1 and D2=1), no parity (D5=0 and D4=0), and two stop bits (D7=1 and D6=1).

The meanings of the bits in the command register are as follows:

- **D7**: EH    1=hunt mode 0=normal operation
- **D6**: IR    1=internal reset 0=normal operation
- **D5**: RTS   Set nRTS output value:
  - 1: nRTS='0'
  - 0: nRTS='1'
- **D4**: ER    1=reset error flags 0=keep error flags
- **D3**: SBRK  1=send break character 0=normal operation
- **D2**: RXE   1=enable receiver 0=disable receiver
- **D1**: DTR   Set nDTR output value:
  - 1: nDTR='0'
  - 0: nDTR='1'
- **D0**: TXEN  1=enable transmitter 0=disable transmitter

The USART chip integrates various features such as receiver and transmitter logic, status output signals, and modem-control capabilities. This function primarily focuses on setting up the chip's mode and command registers for basic operation.

The bus-interface of the 8251 is asynchronous. The CDN (command/not-data) control input selects between command and data transfers. The resulting behavior is as follows (Source of table: [link](https://tams.informatik.uni-hamburg.de/applets/hades/webdemos/50-rtlib/65-usart8251/usart.html)):

| RESET | CSN | RDN | WRN | CDN | DATA | Behavior                                     |
|-------|-----|-----|-----|-----|------|----------------------------------------------|
|   1   |  *  |  *  |  *  |  *  |   *  | device reset                                 |
|   0   |  1  |  *  |  *  |  *  |   *  | device passive                               |
|   0   |  0  |  1  |  1  |  *  |   *  | device selected but inactive                 |
|   0   |  0  |  0  |  1  |  1  | read | read status register                         |
|   0   |  0  |  1  |  0  |  1  | write| write mode/command/sync registers            |
|   0   |  0  |  0  |  1  |  0  | read | read receive buffer                          |
|   0   |  0  |  1  |  0  |  0  | write| write transmit buffer                        |

As shown in the above table, the receive buffer and status register can be selected via the CDN input for reading, and the transmit buffer can be selected via CDN=0 for writing. However, it is impossible to directly select any one of the four control (mode/command/sync1/sync2) registers for writing when CDN=1.
Instead, a state-machine inside the 8251 chip selects which control register is to be written depending on its current state. The first control (CDN=1) write operation following a chip-reset is interpreted as a write to the mode register.
If synchronous mode has been selected by this write operation, the next (or next two) write operations are used to initialize the sync1 (or sync1 and sync2) registers.
All following write operations are interpreted to write the command register. If the "internal reset" bit is set during one of the command register write operations, the chip returns to the reset-state, and the first subsequent write operation is again targeted at the mode register.

## Development

The project was developed using the [Arduino IDE](https://www.arduino.cc/), and testing was conducted with [RealTerm](https://realterm.sourceforge.io/).

Here you can see a screenshot of the result:

![Screenshot](https://github.com/bazsimarkus/Simple-8251-USART-System-with-Arduino/raw/main/images/8251_screenshot.png)

## Notes

- The baud rate, parity bit, etc., are initialized by the Arduino at the beginning of the procedure.
- The text to send is stored in the PROGMEM section of the microcontroller memory, allowing the Arduino to send long messages without any stack problems.
- No external crystal is needed, as the program utilizes Timer1 and Timer2 of the ATmega328P microcontroller on the Arduino UNO board to generate clock signals, and achieve the desired standard 9600 Baud rate.
- After some testing, I found that the 8251 chip, particularly my specific model (D8251AFC), is very sensitive to proper clock signals. For example, I used a clock signal of 9600 Hz for the TxC (no prescaler mode) and around 1.6 MHz for the system clock, although a value around 1 MHz would also suffice. I noticed that sometimes, when I moved the jumper cables on the breadboard during testing, some characters were sent multiple times. However, when testing on a stable prototype PCB, this behavior disappeared. So, pay attention to proper timings! The signals generated by the Arduino are perfectly good for this circuit, but if you are using a good crystal, or a function generator anyway, you can just disconnect the clock input pins from the Arduino, and provide the clock signals from your own source. For the TxC, I am using 9600 Hz, as I am in no prescaler mode, but you can also use 153600 Hz; in this case, you have to set the prescaler to 16 in the initUart function (10 instead of 01). For the system clock, you can use any frequency between 1-4 MHz, I guess (according to the datasheet it has to be at least 30x the baud clock, but my experience is that it works with less, too).

Check out my other serial test circuit projects, developed concurrently with this circuit!
