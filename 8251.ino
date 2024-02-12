/**
 * @file 8251.ino
 * @author Balazs Markus
 * @brief Arduino code for interacting with the 8251 USART chip and transmitting a test string with 9600 baud, 8-bit data, no parity, and 1 stop bit, once every second.
 * @details This code provides a simple test environment for an 8251 Universal Synchronous/Asynchronous Receiver-Transmitter (USART) chip. It is specifically tested with a D8251AFC chip but is forward compatible with all serial ICs based on the 8251 architecture.
 *
 * Pinout of the 8251 USART chip (DIP-28):
 *
 *         ┌───────○───────┐
 *      D2 │1            28│ D1
 *      D3 │2            27│ D0
 *     RxD │3            26│ Vcc
 *     GND │4            25│ RxCN
 *      D4 │5            24│ DTRN
 *      D5 │6    8251A   23│ RTSN
 *      D6 │7    USART   22│ DSRN
 *      D7 │8            21│ RESET
 *    TxCN │9            20│ CLK
 *     WRN │10           19│ TxD
 *     CSN |11           18| TxEMPTY
 *     CDN |12           17| CTSN
 *     RDN |13           16| SYNDET/BD
 *   RxRDY |14           15| TxRDY
 *         └───────────────┘
 *
 * Pin descriptions of the 8251 USART chip:
 *
 * --------------------------------------------------------------------------------
 * | Pin Number | Name      | Type   | Description                                |
 * |------------|-----------|--------|--------------------------------------------|
 * | 1          | D2        | In/Out | Data Bit 2                                 |
 * | 2          | D3        | In/Out | Data Bit 3                                 |
 * | 3          | RxD       | In     | Receive                                    |
 * | 4          | GND       | -      | Ground                                     |
 * | 5          | D4        | In/Out | Data Bit 4                                 |
 * | 6          | D5        | In/Out | Data Bit 5                                 |
 * | 7          | D6        | In/Out | Data Bit 6                                 |
 * | 8          | D7        | In/Out | Data Bit 7                                 |
 * | 9          | TxCN      | In     | Transmit Clock Input (Active Low)          |
 * | 10         | WRN       | In     | Write (Active Low)                         |
 * | 11         | CSN       | In     | Chip Select (Active Low)                   |
 * | 12         | CDN       | In     | Command/Data Select                        |
 * | 13         | RDN       | In     | Read (Active Low)                          |
 * | 14         | RxRDY     | Out    | Read Register Ready                        |
 * | 15         | TxRDY     | Out    | Transmitter Register Ready                 |
 * | 16         | SYNDET/BD | In/Out | Sync Detect/Break Detect (See Data Sheet)  |
 * | 17         | CTSN      | In     | Clear To Send (Active Low)                 |
 * | 18         | TxEMPTY   | Out    | Transmitter Register Empty                 |
 * | 19         | TxD       | Out    | Transmit Output                            |
 * | 20         | CLK       | In     | Clock                                      |
 * | 21         | RESET     | In     | Reset                                      |
 * | 22         | DSRN      | In     | Data Set Ready (Active Low)                |
 * | 23         | RTSN      | Out    | Request to Send (Active Low)               |
 * | 24         | DTRN      | Out    | Data Terminal Ready (Active Low)           |
 * | 25         | RXCN      | In     | Receive Clock (Active Low)                 |
 * | 26         | Vcc       | -      | Positive Supply                            |
 * | 27         | D0        | In/Out | Data Bit 0                                 |
 * | 28         | D1        | In/Out | Data Bit 1                                 |
 * --------------------------------------------------------------------------------
 *
 * Components needed for the test circuit:
 * - 8251 USART chip
 * - Arduino board (Uno, Nano, both works, the important thing is that it should operate on 5V voltage)
 * - USB-TTL Serial converter for testing (FTDI will do as well)
 *
 * The circuit works as follows:
 * The Arduino sends 8-bit ASCII characters to the USART chip, then the chip converts it to serial data (9600 baud, 8-bit data, no parity, and 1 stop bit).
 * The parameters like the baud rate, parity bit, etc., are initialized by the Arduino at the beginning of the procedure.
 * The text to send is stored in the PROGMEM section of the microcontroller memory, allowing the Arduino to send long messages without any stack problems.
 * No external crystal is needed, as the program utilizes Timer1 and Timer2 of the ATmega328P microcontroller on the Arduino UNO board to generate clock signals, and achieve the desired standard 9600 Baud rate.
 *
 * How to use:
 * 1. Connect your Arduino board to the 8251 USART chip based on the included schematics.
 * 2. Upload the 8251.ino source code to the Arduino board.
 * 3. Connect the USB-TTL Serial converter to the PC and to the TxD/RxD pins of the 8251
 * 4. Open a serial terminal program (for example, RealTerm), select the COM port of your USB-Serial converter, and set up the terminal to use 9600 baud, 8-bit data, no parity, and 1 stop bit.
 * 5. You should be receiving the string "TESTABCD" once every second.
 *
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>

#define CDN A0    // Command/Data (CDN) line for selecting between command and data transfers
#define WRN A1    // Write Enable (WRN) line for enabling write operations
#define RDN A2    // Read Enable (RDN) line for enabling read operations
#define CSN A3    // Chip Select (CSN) line for selecting the 8251 USART chip
#define RESET A4  // Reset line for resetting the 8251 USART chip
#define CTSN A5   // Clear To Send (CTSN) input signal line for modem control

#define D0 2   // Data bus line 0
#define D1 3   // Data bus line 1
#define D2 4   // Data bus line 2
#define D3 5   // Data bus line 3
#define D4 6   // Data bus line 4
#define D5 7   // Data bus line 5
#define D6 8   // Data bus line 6
#define D7 10  // Data bus line 7

#define BAUDCLK 9     // BAUDCLK output pin for generating baud rate clock
#define SYSTEMCLK 11  // SYSTEMCLK output pin for generating system clock

#define LED_OUTPUT 13  // Arduino onboard LED pin to indicate status

// Timing constants
#define DELAY_SHORT 1    // Short delay time in milliseconds
#define DELAY_MEDIUM 50  // Medium delay time in milliseconds
#define DELAY_LONG 200   // Long delay time in milliseconds

#define BAUDRATE 9600  // Baud rate for serial communication

// Test string definition stored in PROGMEM (program memory)
const char testString[] PROGMEM = {"UARTTEST"};

/**
 * @brief Arduino setup function.
 *
 * @details This function is called once when the microcontroller is powered on or reset. It is used to initialize
 * the microcontroller's hardware and peripherals, such as pins, timers, serial communication, etc.
 *
 * In this project, the setup function initializes the USART (Universal Synchronous/Asynchronous Receiver-Transmitter)
 * registers for serial communication and sets the pin modes for input and output.
 *
 * @note The setup function runs only once after power-up or reset. It should be used to perform
 * initialization tasks and configuration settings.
 */
void setup() {
    pinMode(LED_OUTPUT, OUTPUT);
    setUsartPinsToOutput();
    initClocks();
    initUsart();
}

/**
 * @brief Arduino main loop function.
 *
 * @details The loop function is called repeatedly as long as the microcontroller is powered on.
 * It is used to implement the main program logic, perform tasks, and handle events.
 *
 * In this project, the loop function continuously a string
 * with the 8251 USART using the printStringToUsart function.
 *
 * @note The loop function runs in an infinite loop and should be used to execute the main
 * program logic or perform tasks that need to be repeated continuously.
 */
void loop() {
    // Print the string with the 8251 USART
    printStringToUsart(testString);

    // Blink onboard LED
    digitalWrite(LED_OUTPUT, HIGH);
    delay(DELAY_MEDIUM);
    digitalWrite(LED_OUTPUT, LOW);

    // Wait one second
    delay(1000);
}

/**
 * @brief Set all USART pins to output mode.
 *
 * @details This function sets all USART pins to output mode.
 * It configures the pins for controlling the serial communication.
 */
void setUsartPinsToOutput() {
    // Control pins
    pinMode(CDN, OUTPUT);
    pinMode(WRN, OUTPUT);
    pinMode(RDN, OUTPUT);
    pinMode(CSN, OUTPUT);
    pinMode(RESET, OUTPUT);
    pinMode(CTSN, OUTPUT);
    // Data bus
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);
    pinMode(D7, OUTPUT);
}

/**
 * @brief Set data bus pins to high impedance (not used).
 *
 * @details This function sets the data bus pins to high impedance,
 * but it is not used in the current implementation.
 */
void setDataBusToHighZ() {
    pinMode(D0, INPUT);  // Data Bus D0 - Input (High-Z)
    pinMode(D1, INPUT);  // Data Bus D1 - Input (High-Z)
    pinMode(D2, INPUT);  // Data Bus D2 - Input (High-Z)
    pinMode(D3, INPUT);  // Data Bus D3 - Input (High-Z)
    pinMode(D4, INPUT);  // Data Bus D4 - Input (High-Z)
    pinMode(D5, INPUT);  // Data Bus D5 - Input (High-Z)
    pinMode(D6, INPUT);  // Data Bus D6 - Input (High-Z)
    pinMode(D7, INPUT);  // Data Bus D7 - Input (High-Z)
}

/**
 * @brief Write data to the data bus.
 *
 * @details This function sets the data bus pins in bulk. It uses the bitRead
 * function to retrieve each bit of the input parameter.
 *
 * @param data The data to be written to the data bus.
 */
void writeDataBus(int data) {
    digitalWrite(D0, bitRead(data, 0));
    digitalWrite(D1, bitRead(data, 1));
    digitalWrite(D2, bitRead(data, 2));
    digitalWrite(D3, bitRead(data, 3));
    digitalWrite(D4, bitRead(data, 4));
    digitalWrite(D5, bitRead(data, 5));
    digitalWrite(D6, bitRead(data, 6));
    digitalWrite(D7, bitRead(data, 7));
}

/**
 * @brief Write a character to USART.
 *
 * @details This function writes an 8-bit character to the USART for transmission. This function can only be called after initialization!
 *
 * @param character The character to be transmitted.
 *
 * The function begins by writing the character to the USART's data register for transmission. It follows a series of steps to ensure proper data transmission:
 *
 * 1. Set the WRN (Write Enable) line HIGH to enable writing to the USART's registers.
 *    - This step prepares the USART for receiving data.
 *
 * 2. Set the CDN (Command/Data Not) line LOW to indicate that data is being written (not a command).
 *    - By setting the CDN line LOW, the function specifies that the following data transmission is not a command but actual data being written to the USART.
 *
 * 3. Set the CSN (Chip Select Not) line LOW to select the USART chip for communication.
 *    - Bringing the CSN line LOW enables the USART chip to receive the data being transmitted.
 *
 * 4. Provide a short delay (DELAY_SHORT) to ensure proper synchronization and stability of signals.
 *    - Introducing a delay allows time for the USART and associated circuitry to settle before data transmission.
 *
 * 5. Write the character to the data bus using the writeDataBus() function.
 *    - The character to be transmitted is written onto the data bus for communication with the USART.
 *
 * 6. Provide another short delay (DELAY_SHORT) for signal stability.
 *    - This delay ensures that the transmitted data is properly captured by the USART before proceeding.
 *
 * 7. Set the WRN line LOW to complete the data write operation.
 *    - Bringing WRN LOW indicates the end of the data write operation.
 *
 * 8. Provide a short delay (DELAY_SHORT) before setting WRN HIGH again.
 *    - This delay ensures proper signal timing and adherence to USART communication protocols.
 *
 * 9. Set the WRN line HIGH to disable further writing to the USART registers.
 *    - Once the data is transmitted, WRN is set HIGH to prevent accidental data writes and maintain the integrity of the USART configuration.
 *
 * Each step in this function is essential for proper USART communication, ensuring that data is correctly transmitted and received.
 */
void writeCharToUsart(char character) {
    // Write data register
    digitalWrite(WRN, HIGH);
    digitalWrite(CDN, LOW);
    digitalWrite(CSN, LOW);
    delay(DELAY_SHORT);
    writeDataBus(character);
    delay(DELAY_SHORT);
    digitalWrite(WRN, LOW);
    delay(DELAY_SHORT);
    digitalWrite(WRN, HIGH);
    delay(DELAY_SHORT);
}

/**
 * @brief Print a string to USART.
 *
 * @details This function prints a string to the USART for transmission.
 *
 * @param str The string to be transmitted. The string has to be zero terminated!
 */
void printStringToUsart(const char* str) {
    char myChar;
    for (int j = 0; j < strlen(str); j++) {
        myChar = pgm_read_byte_near(str + j);
        writeCharToUsart(myChar);
    }
}

/**
 * @brief Blink the onboard LED to indicate status.
 *
 * @details This function blinks the onboard LED to indicate status.
 * It is used for visual indication in the circuit.
 */
void blinkLed() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_OUTPUT, HIGH);
        delay(DELAY_LONG);
        digitalWrite(LED_OUTPUT, LOW);
        delay(DELAY_LONG);
    }
}

/**
 * @brief Initializes clocks for the USART chip, utilizing Timer1 and Timer2 of the ATmega328P microcontroller on the Arduino UNO board.
 *
 * This function configures the clocks for BAUDCLK and SYSTEMCLK.
 * BAUDCLK is generated using Timer1 with a frequency of 9600 Hz (BAUDRATE with no prescaler (1x) mode - while 16x prescaler mode is recommended for stability, for this initial test, the 1x mode suffices),
 * and SYSTEMCLK is generated using Timer2 with a frequency of 1.6 MHz (according to the datasheet, SYSTEMCLK should be at least 30 times the BAUDCLK, but lower frequencies may suffice).
 */
void initClocks() {
    // Set pin 9 as output for BAUDCLK and pin 11 as output for SYSTEMCLK
    pinMode(BAUDCLK, OUTPUT);
    pinMode(SYSTEMCLK, OUTPUT);

    // Timer1 configuration for BAUDCLK - 9600 Hz
    // Toggle OC1A (connected to pin 9) on compare match
    TCCR1A = 0 | (1 << COM1A0);
    // Set Timer1 to CTC mode and set prescaler to 1
    TCCR1B = 0 | (1 << WGM12) | (1 << CS10);
    // Set compare match register to desired frequency for BAUDCLK
    OCR1A = 16000000UL / (2 * BAUDRATE) - 1;

    // Timer2 configuration for SYSTEMCLK - 1.6 MHz
    // Bits 7:6 mean toggle pin 11 on a compare match
    // Bits 1:0 and bit 3 in TCCR2B select Fast PWM mode, with OCRA for the TOP value
    TCCR2A = 0b01000011;
    // Bits 2:0 are the prescaler setting, this is a 1 prescaler
    TCCR2B = 0b00001001;
    // Value for the timer to toggle the output pin and restart for SYSTEMCLK
    OCR2A = 4;
}

/**
 * @brief Initializes the 8251 USART (Universal Synchronous/Asynchronous Receiver-Transmitter) chip.
 *
 * @details This function initializes the USART chip, configuring its registers for proper operation. The USART chip integrates a standard (8-bit) microprocessor bus interface, one serial transmitter, and one serial receiver. It also provides additional control lines for modem-control and efficient handshaking or interrupts.
 *
 * The bus-interface consists of the bidirectional 8-bit data-bus (lines D7..D0) and the read/write control-logic with the following inputs:
 * - DATA (D7..D0): 8-bit bidirectional data-bus
 * - RESET: Active-high reset input
 * - CSN: Chip select input (active low)
 * - CDN: Command (high) or data (low) selection input
 * - RDN: Read enable input (active low)
 * - WRN: Write enable input (active low)
 *
 * The function sets up the chip's control lines and registers according to the following sequence:
 * 1. Reset the chip by pulling the RESET line LOW momentarily and then HIGH.
 * 2. Write to the mode register to configure the operation mode and parameters. The mode register controls stop bits selection, parity selection, character length, and mode/baud-rate factor.
 * 3. Write to the command register to enable/disable various functions and reset error flags.
 *
 * The meanings of the bits in the mode register are as follows:
 * - D7D6: Stop bits selection:
 *          11 = 2 stop bits
 *          10 = 1.5 stop bits
 *          01 = 1 stop bit
 *          00 = Invalid (at least 1 stop bit required)
 * - D5D4: Parity selection:
 *          11 = Even parity
 *          10 = Parity disabled
 *          01 = Odd parity
 *          00 = Parity disabled
 * - D3D2: Character length:
 *          11 = 8 data bits
 *          10 = 7 data bits
 *          01 = 6 data bits
 *          00 = 5 data bits
 * - D1D0: Mode / Baud-rate factor:
 *          11 = Async mode, 64x TXC/RXC prescaler
 *          10 = Async mode, 16x TXC/RXC prescaler
 *          01 = Async mode, no clock prescaler
 *          00 = Sync mode
 *
 * For example, writing the binary value 11001101 or hex 0xCD to the mode register selects async mode without prescaler (D1=0 and D0=1), eight data bits (D3=1 and D2=1), no parity (D5=0 and D4=0), and two stop bits (D7=1 and D6=1).
 *
 * The meanings of the bits in the command register are as follows:
 * - D7:   EH    1=hunt mode 0=normal operation
 * - D6:   IR    1=internal reset 0=normal operation
 * - D5:   RTS   Set nRTS output value:
 *               1: nRTS='0'
 *               0: nRTS='1'
 * - D4:   ER    1=reset error flags 0=keep error flags
 * - D3:   SBRK  1=send break character 0=normal operation
 * - D2:   RXE   1=enable receiver 0=disable receiver
 * - D1:   DTR   Set nDTR output value:
 *               1: nDTR='0'
 *               0: nDTR='1'
 * - D0:   TXEN  1=enable transmitter 0=disable transmitter
 *
 * The USART chip integrates various features such as receiver and transmitter logic, status output signals, and modem-control capabilities. This function primarily focuses on setting up the chip's mode and command registers for basic operation.
 *
 * The bus-interface of the 8251 is asynchronous. The CDN (command/not-data) control input selects between command and data transfers. The resulting behavior is as follows (Source of table: https://tams.informatik.uni-hamburg.de/applets/hades/webdemos/50-rtlib/65-usart8251/usart.html):
 *
 * | RESET | CSN | RDN | WRN | CDN | DATA | Behavior                                     |
 * |-------|-----|-----|-----|-----|------|----------------------------------------------|
 * |   1   |  *  |  *  |  *  |  *  |   *  | device reset                                 |
 * |   0   |  1  |  *  |  *  |  *  |   *  | device passive                               |
 * |   0   |  0  |  1  |  1  |  *  |   *  | device selected but inactive                 |
 * |   0   |  0  |  0  |  1  |  1  | read | read status register                         |
 * |   0   |  0  |  1  |  0  |  1  | write| write mode/command/sync registers            |
 * |   0   |  0  |  0  |  1  |  0  | read | read receive buffer                          |
 * |   0   |  0  |  1  |  0  |  0  | write| write transmit buffer                        |
 *
 * The chip contains seven user-visible registers, five of which can only be written, and two of which can only be read. These are the following:
 * - mode register: sync/async operation mode and parameters
 * - command register: enable/disable and error resetting
 * - sync 1 character: 8-bit dataword (sync mode only)
 * - sync 2 character: 8-bit dataword (sync mode only)
 * - transmit buffer: 8-bit register for outgoing data
 * - receive buffer: 8-bit register with incoming data
 * - status register: several status and error bits
 *
 * As shown in the above table, the receive buffer and status register can be selected via the CDN input for reading, and the transmit buffer can be selected via CDN=0 for writing.
 * However, it is impossible to directly select any one of the four control (mode/command/sync1/sync2) registers for writing when CDN=1.
 * Instead, a state-machine inside the 8251 chip selects which control register is to be written depending on its current state. The first control (CDN=1) write operation following a chip-reset is interpreted as a write to the mode register.
 * If synchronous mode has been selected by this write operation, the next (or next two) write operations are used to initialize the sync1 (or sync1 and sync2) registers.
 * All following write operations are interpreted to write the command register. If the "internal reset" bit is set during one of the command register write operations, the chip returns to the reset-state, and the first subsequent write operation is again targeted at the mode register.
 *
 * The initialization steps are as follows:
 *
 * 1. Set control pins:
 *    - Set CTSN, RESET, and CDN pins to LOW.
 *    - Set CSN, WRN, and RDN pins to HIGH.
 *    - Apply short delays between pin operations for stabilization.
 *
 * 2. Reset the 8251 chip:
 *    - Set RESET pin to HIGH for a short duration.
 *    - Set RESET pin to LOW to reset the chip.
 *    - Apply short delays between reset operations.
 *
 * 3. Write to the mode register:
 *    - Set CND pin to HIGH and CSN pin to LOW to select the chip for writing.
 *    - Write the desired mode configuration value (0b01001101) to the data bus.
 *    - Apply a short delay between setting up the data bus and writing.
 *    - Toggle the WRN pin to initiate the write operation.
 *    - Apply a medium delay after the write operation.
 *    - Toggle the WRN pin again to complete the write operation.
 *    - Set CND and CSN pins to LOW to deselect the chip.
 *
 * 4. Write to the command register:
 *    - Set CND pin to HIGH and CSN pin to LOW to select the chip for writing.
 *    - Write the desired command configuration value (0b00000001) to the data bus.
 *    - Apply a short delay between setting up the data bus and writing.
 *    - Toggle the WRN pin to initiate the write operation.
 *    - Apply a medium delay after the write operation.
 *    - Toggle the WRN pin again to complete the write operation.
 *    - Set CND and CSN pins to LOW to deselect the chip.
 *
 * 5. Indicate initialization completion by blinking the onboard LED.
 *
 */
void initUsart() {
    digitalWrite(LED_OUTPUT, HIGH);  // Turn on the onboard LED to indicate the start of the initialization

    digitalWrite(CTSN, LOW);  // Set the CTSN pin to LOW
    digitalWrite(CDN, LOW);   // Set the CDN pin to LOW
    digitalWrite(CSN, HIGH);  // Set the CSN pin to HIGH
    digitalWrite(WRN, HIGH);  // Set the WRN pin to HIGH
    digitalWrite(RDN, HIGH);  // Set the RDN pin to HIGH
    delay(DELAY_SHORT);       // Apply a short delay for stabilization

    // Reset the chip
    digitalWrite(RESET, HIGH);  // Set the RESET pin to HIGH
    delay(DELAY_SHORT);         // Apply a short delay
    digitalWrite(RESET, LOW);   // Set the RESET pin to LOW to reset the chip
    delay(DELAY_SHORT);         // Apply a short delay

    // Write mode register
    digitalWrite(CDN, HIGH);  // Set the CDN pin to HIGH
    delay(DELAY_SHORT);       // Apply a short delay
    digitalWrite(CSN, LOW);   // Set the CSN pin to LOW to select the chip for writing
    delay(DELAY_SHORT);       // Apply a short delay

    writeDataBus(0b01001101);  // Write the desired mode configuration value (0b01001101) to the data bus
    delay(DELAY_SHORT);        // Apply a short delay

    digitalWrite(WRN, LOW);   // Toggle the WRN pin to initiate the write operation
    delay(DELAY_MEDIUM);      // Apply a medium delay after the write operation
    digitalWrite(WRN, HIGH);  // Toggle the WRN pin again to complete the write operation
    delay(DELAY_SHORT);       // Apply a short delay

    digitalWrite(CDN, LOW);   // Set the CDN pin to LOW
    digitalWrite(CSN, HIGH);  // Set the CSN pin to HIGH to deselect the chip
    delay(DELAY_SHORT);       // Apply a short delay

    // Write command register
    digitalWrite(CDN, HIGH);  // Set the CDN pin to HIGH
    delay(DELAY_SHORT);       // Apply a short delay
    digitalWrite(CSN, LOW);   // Set the CSN pin to LOW to select the chip for writing
    delay(DELAY_SHORT);       // Apply a short delay

    writeDataBus(0b00000001);  // Write the desired command configuration value (0b00000001) to the data bus
    delay(DELAY_SHORT);        // Apply a short delay

    digitalWrite(WRN, LOW);   // Toggle the WRN pin to initiate the write operation
    delay(DELAY_MEDIUM);      // Apply a medium delay after the write operation
    digitalWrite(WRN, HIGH);  // Toggle the WRN pin again to complete the write operation
    delay(DELAY_SHORT);       // Apply a short delay

    digitalWrite(CDN, LOW);   // Set the CDN pin to LOW
    digitalWrite(CSN, HIGH);  // Set the CSN pin to HIGH to deselect the chip
    delay(DELAY_SHORT);       // Apply a short delay

    // Indicate the end of initialization by blinking the onboard LED
    blinkLed();
}
