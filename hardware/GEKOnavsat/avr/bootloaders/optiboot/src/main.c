/**
* \file
*
* \brief Empty user application template
*
*/

/**
* \mainpage User Application template doxygen documentation
*
* \par Empty user application template
*
* Bare minimum empty user application template
*
* \par Content
*
* -# Include the ASF header files (through asf.h)
* -# "Insert system clock initialization code here" comment
* -# Minimal main function that starts with a call to board_init()
* -# "Insert application code here" comment
*
*/

/*
* Include header files for all drivers that have been imported from
* Atmel Software Framework (ASF).
*/
#include <asf.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#define FUNC_READ 1
#define FUNC_WRITE 1
#define OPTIBOOT_MAJVER 6
#define OPTIBOOT_MINVER 2

/*
* OPTIBOOT_CUSTOMVER should be defined (by the makefile) for custom edits
* of optiboot.  That way you don't wind up with very different code that
* matches the version number of a "released" optiboot.
*/

/*
* Miscellaneous: -std=gnu99 -fno-strict-aliasing -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax
*/

#if !defined(OPTIBOOT_CUSTOMVER)
#define OPTIBOOT_CUSTOMVER 0
#endif

unsigned const int __attribute__((section(".version")))
optiboot_version = 256*(OPTIBOOT_MAJVER + OPTIBOOT_CUSTOMVER) + OPTIBOOT_MINVER;

/*
* Note that we use our own version of "boot.h"
* <avr/boot.h> uses sts instructions, but this version uses out instructions
* This saves cycles and program memory.  Sorry for the name overlap.
*/
#include "boot.h"

/*
* stk500.h contains the constant definitions for the stk500v1 comm protocol
*/
#include "stk500.h"

//#ifndef LED_START_FLASHES
//#define LED_START_FLASHES 0
//#endif

/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
#define BAUD_RATE   57600L // Highest rate Avrdude win32 will support
#elif F_CPU >= 1000000L
#define BAUD_RATE   9600L   // 19200 also supported, but with significant error
#elif F_CPU >= 128000L
#define BAUD_RATE   4800L   // Good for 128kHz internal RC
#else
#define BAUD_RATE 1200L     // Good even at 32768Hz
#endif
#endif

#ifndef UART
#define UART 0
#endif

#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 4L) / ((BAUD_RATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((BAUD_SETTING)+1)))
#define BAUD_ERROR (( 100*(BAUD_RATE - BAUD_ACTUAL) ) / BAUD_RATE)

#if BAUD_ERROR >= 5
#error BAUD_RATE error greater than 5%
#elif BAUD_ERROR <= -5
#error BAUD_RATE error greater than -5%
#elif BAUD_ERROR >= 2
#warning BAUD_RATE error greater than 2%
#elif BAUD_ERROR <= -2
#warning BAUD_RATE error greater than -2%
#endif

#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 > 250
#error Unachievable baud rate (too slow) BAUD_RATE
#endif // baud rate slow check
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 < 3
#if BAUD_ERROR != 0 // permit high bitrates (ie 1Mbps@16MHz) if error is zero
#error Unachievable baud rate (too fast) BAUD_RATE
#endif
#endif // baud rate fastn check

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))

/*
* We can never load flash with more than 1 page at a time, so we can save
* some code space on parts with smaller pagesize by using a smaller int.
*/
#if SPM_PAGESIZE > 255
typedef uint16_t pagelen_t ;
#define GETLENGTH(len) len = getch()<<8; len |= getch()
#else
typedef uint8_t pagelen_t;
#define GETLENGTH(len) (void) getch() /* skip high byte */; len = getch()
#endif


/* Function Prototypes
* The main() function is in init9, which removes the interrupt vector table
* we don't need. It is also 'OS_main', which means the compiler does not
* generate any entry or exit code itself (but unlike 'naked', it doesn't
* supress some compile-time options we want.)
*/

int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));

void __attribute__((noinline)) putch(char);
uint8_t __attribute__((noinline)) getch(void);
void __attribute__((noinline)) verifySpace();
void __attribute__((noinline)) watchdogConfig(uint8_t x);

static inline void getNch(uint8_t);
//static inline void flash_led(uint8_t);
static inline void watchdogReset();
static inline void writebuffer(int8_t memtype, uint8_t *mybuff, uint16_t address, pagelen_t len);
static inline void read_mem(uint8_t memtype, uint16_t address, pagelen_t len);

#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
void appStart(uint8_t rstFlags) __attribute__ ((naked));


/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(RAMSTART))
#ifdef VIRTUAL_BOOT_PARTITION
#define rstVect (*(uint16_t*)(RAMSTART+SPM_PAGESIZE*2+4))
#define wdtVect (*(uint16_t*)(RAMSTART+SPM_PAGESIZE*2+6))
#endif


/* main program starts here */
int main(void) {
	uint8_t ch;

	/*
	* Making these local and in registers prevents the need for initializing
	* them, and also saves space because code no longer stores to memory.
	* (initializing address keeps the compiler happy, but isn't really
	*  necessary, and uses 4 bytes of flash.)
	*/
	register uint16_t address = 0;
	register pagelen_t  length;

	// After the zero init loop, this is the first code to run.
	//
	// This code makes the following assumptions:
	//  No interrupts will execute
	//  SP points to RAMEND
	//  r1 contains zero
	//
	// If not, uncomment the following instructions:
	// cli();
	asm volatile ("clr __zero_reg__");

	/*
	* modified Adaboot no-wait mod.
	* Pass the reset reason to app.  Also, it appears that an Uno poweron
	* can leave multiple reset flags set; we only want the bootloader to
	* run on an 'external reset only' status
	*/
	ch = MCUSR;
	MCUSR = 0;
	if (ch & (_BV(WDRF) | _BV(BORF) | _BV(PORF))){
		appStart(ch);
	}
	
	//#if LED_START_FLASHES > 0
	//// Set up Timer 1 for timeout counter
	//TCCR1B = _BV(CS12) | _BV(CS10); // div 1024
	//#endif

	#ifndef SOFT_UART
	UART_SRA = _BV(U2X0); //Double speed mode USART0
	UART_SRB = _BV(RXEN0) | _BV(TXEN0);
	UART_SRC = _BV(UCSZ00) | _BV(UCSZ01);
	//UART_SRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
	UART_SRL = BAUD_SETTING;
	#endif

	// Set up watchdog to trigger after 500ms
	watchdogConfig(WATCHDOG_1S);


	//#if (LED_START_FLASHES > 0) || defined(LED_DATA_FLASH)
	///* Set LED pin as output */
	//LED_DDR |= _BV(LED);
	//#endif
	//
	//#ifdef SOFT_UART
	///* Set TX pin as output */
	//UART_DDR |= _BV(UART_TX_BIT);
	//#endif
	//
	//#if LED_START_FLASHES > 0
	///* Flash onboard LED to signal entering of bootloader */
	//flash_led(LED_START_FLASHES * 2);
	//#endif

	/* Forever loop: exits by causing WDT reset */
	for (;;) {
		/* get character from UART */
		ch = getch();

		if(ch == STK_GET_PARAMETER) {
			unsigned char which = getch();
			verifySpace();
			/*
			* Send optiboot version as "SW version"
			* Note that the references to memory are optimized away.
			*/
			if (which == 0x82) {
				putch(optiboot_version & 0xFF);
			} 
			else if (which == 0x81) {
				putch(optiboot_version >> 8);
			} 
			else {
				/*
				* GET PARAMETER returns a generic 0x03 reply for
				* other parameters - enough to keep Avrdude happy
				*/
				putch(0x03);
			}
		}
		else if(ch == STK_SET_DEVICE) {
			// SET DEVICE is ignored
			getNch(20);
		}
		else if(ch == STK_SET_DEVICE_EXT) {
			// SET DEVICE EXT is ignored
			getNch(5);
		}
		else if(ch == STK_LOAD_ADDRESS) {
			// LOAD ADDRESS
			uint16_t newAddress;
			newAddress = getch();
			newAddress = (newAddress & 0xff) | (getch() << 8);
			#ifdef RAMPZ
			// Transfer top bit to RAMPZ
			RAMPZ = (newAddress & 0x8000) ? 1 : 0;
			#endif
			newAddress += newAddress; // Convert from word address to byte address
			address = newAddress;
			verifySpace();
		}
		else if(ch == STK_UNIVERSAL) {
			// UNIVERSAL command is ignored
			getNch(4);
			putch(0x00);
		}
		/* Write memory, length is big endian and is in bytes */
		else if(ch == STK_PROG_PAGE) {
			// PROGRAM PAGE - we support flash programming only, not EEPROM
			uint8_t desttype;
			uint8_t *bufPtr;
			pagelen_t savelength;

			GETLENGTH(length);
			savelength = length;
			desttype = getch();

			// read a page worth of contents
			bufPtr = buff;
			do *bufPtr++ = getch();
			while (--length);

			// Read command terminator, start reply
			verifySpace();

			#ifdef VIRTUAL_BOOT_PARTITION
			if ((uint16_t)(void*)address == 0) {
				// This is the reset vector page. We need to live-patch the code so the
				// bootloader runs.
				//
				// Move RESET vector to WDT vector
				uint16_t vect = buff[0] | (buff[1]<<8);
				rstVect = vect;
				wdtVect = buff[8] | (buff[9]<<8);
				vect -= 4; // Instruction is a relative jump (rjmp), so recalculate.
				buff[8] = vect & 0xff;
				buff[9] = vect >> 8;

				// Add jump to bootloader at RESET vector
				buff[0] = 0x7f;
				buff[1] = 0xce; // rjmp 0x1d00 instruction
			}
			#endif

			writebuffer(desttype, buff, address, savelength);


		}
		/* Read memory block mode, length is big endian.  */
		else if(ch == STK_READ_PAGE) {
			uint8_t desttype;
			GETLENGTH(length);

			desttype = getch();

			verifySpace();
			
			read_mem(desttype, address, length);
		}

		/* Get device signature bytes  */
		else if(ch == STK_READ_SIGN) {
			// READ SIGN - return what Avrdude wants to hear
			verifySpace();
			putch(SIGNATURE_0);
			putch(SIGNATURE_1);
			putch(SIGNATURE_2);
		}
		else if (ch == STK_LEAVE_PROGMODE) { /* 'Q' */
			// Adaboot no-wait mod
			watchdogConfig(WATCHDOG_16MS);
			verifySpace();
		}
		else {
			// This covers the response to commands like STK_ENTER_PROGMODE
			verifySpace();
		}
		putch(STK_OK);
	}
}

void putch(char ch) {
	#ifndef SOFT_UART
	while (!(UART_SRA & _BV(UDRE0)));
	UART_UDR = ch;
	#else
	__asm__ __volatile__ (
	"   com %[ch]\n" // ones complement, carry set
	"   sec\n"
	"1: brcc 2f\n"
	"   cbi %[uartPort],%[uartBit]\n"
	"   rjmp 3f\n"
	"2: sbi %[uartPort],%[uartBit]\n"
	"   nop\n"
	"3: rcall uartDelay\n"
	"   rcall uartDelay\n"
	"   lsr %[ch]\n"
	"   dec %[bitcnt]\n"
	"   brne 1b\n"
	:
	:
	[bitcnt] "d" (10),
	[ch] "r" (ch),
	[uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
	[uartBit] "I" (UART_TX_BIT)
	:
	"r25"
	);
	#endif
}

uint8_t getch(void) {
	uint8_t ch;

	//#ifdef LED_DATA_FLASH
	//#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
	//LED_PORT ^= _BV(LED);
	//#else
	//LED_PIN |= _BV(LED);
	//#endif
	//#endif

	#ifdef SOFT_UART
	__asm__ __volatile__ (
	"1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
	"   rjmp  1b\n"
	"   rcall uartDelay\n"          // Get to middle of start bit
	"2: rcall uartDelay\n"              // Wait 1 bit period
	"   rcall uartDelay\n"              // Wait 1 bit period
	"   clc\n"
	"   sbic  %[uartPin],%[uartBit]\n"
	"   sec\n"
	"   dec   %[bitCnt]\n"
	"   breq  3f\n"
	"   ror   %[ch]\n"
	"   rjmp  2b\n"
	"3:\n"
	:
	[ch] "=r" (ch)
	:
	[bitCnt] "d" (9),
	[uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
	[uartBit] "I" (UART_RX_BIT)
	:
	"r25"
	);
	#else
	while(!(UART_SRA & _BV(RXC0)))
	;
	if (!(UART_SRA & _BV(FE0))) {
		/*
		* A Framing Error indicates (probably) that something is talking
		* to us at the wrong bit rate.  Assume that this is because it
		* expects to be talking to the application, and DON'T reset the
		* watchdog.  This should cause the bootloader to abort and run
		* the application "soon", if it keeps happening.  (Note that we
		* don't care that an invalid char is returned...)
		*/
		watchdogReset();
	}
	
	ch = UART_UDR;
	#endif

	//#ifdef LED_DATA_FLASH
	//#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
	//LED_PORT ^= _BV(LED);
	//#else
	//LED_PIN |= _BV(LED);
	//#endif
	//#endif

	return ch;
}

#ifdef SOFT_UART
// AVR305 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
	__asm__ __volatile__ (
	"ldi r25,%[count]\n"
	"1:dec r25\n"
	"brne 1b\n"
	"ret\n"
	::[count] "M" (UART_B_VALUE)
	);
}
#endif

void getNch(uint8_t count) {
	do getch(); while (--count);
	verifySpace();
}

void verifySpace() {
	if (getch() != CRC_EOP) {
		watchdogConfig(WATCHDOG_16MS);    // shorten WD timeout
		while (1)			      // and busy-loop so that WD causes
		;				      //  a reset and app start.
	}
	putch(STK_INSYNC);
}

//#if LED_START_FLASHES > 0
//void flash_led(uint8_t count) {
	//do {
		//TCNT1 = -(F_CPU/(1024*16));
		//TIFR1 = _BV(TOV1);
		//while(!(TIFR1 & _BV(TOV1)));
		//#if defined(__AVR_ATmega8__)  || defined (__AVR_ATmega32__)
		//LED_PORT ^= _BV(LED);
		//#else
		//LED_PIN |= _BV(LED);
		//#endif
		//watchdogReset();
	//} while (--count);
//}
//#endif

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
	__asm__ __volatile__ (
	"wdr\n"
	);
}

void watchdogConfig(uint8_t x) {
	WDTCSR = _BV(WDCE) | _BV(WDE);
	WDTCSR = x;
}

void appStart(uint8_t rstFlags) {
	// save the reset flags in the designated register
	//  This can be saved in a main program by putting code in .init0 (which
	//  executes before normal c init code) to save R2 to a global variable.
	__asm__ __volatile__ ("mov r2, %0\n" :: "r" (rstFlags));

	watchdogConfig(WATCHDOG_OFF);
	__asm__ __volatile__ (
	#ifdef VIRTUAL_BOOT_PARTITION
	// Jump to WDT vector
	"ldi r30,4\n"
	"clr r31\n"
	#else
	// Jump to RST vector
	"clr r30\n"
	"clr r31\n"
	#endif
	"ijmp\n"
	);
}

/*
* void writebuffer(memtype, buffer, address, length)
*/
static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
uint16_t address, pagelen_t len)
{
	switch (memtype) {
		case 'E': // EEPROM
		#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
		while(len--) {
			eeprom_write_byte((uint8_t *)(address++), *mybuff++);
		}
		#else
		/*
		* On systems where EEPROM write is not supported, just busy-loop
		* until the WDT expires, which will eventually cause an error on
		* host system (which is what it should do.)
		*/
		while (1)
		; // Error: wait for WDT
		#endif
		break;
		default:  // FLASH
		/*
		* Default to writing to Flash program memory.  By making this
		* the default rather than checking for the correct code, we save
		* space on chips that don't support any other memory types.
		*/
		{
			// Copy buffer into programming buffer
			uint8_t *bufPtr = mybuff;
			uint16_t addrPtr = (uint16_t)(void*)address;

			/*
			* Start the page erase and wait for it to finish.  There
			* used to be code to do this while receiving the data over
			* the serial link, but the performance improvement was slight,
			* and we needed the space back.
			*/
			__boot_page_erase_short((uint16_t)(void*)address);
			boot_spm_busy_wait();

			/*
			* Copy data from the buffer into the flash write buffer.
			*/
			do {
				uint16_t a;
				a = *bufPtr++;
				a |= (*bufPtr++) << 8;
				__boot_page_fill_short((uint16_t)(void*)addrPtr,a);
				addrPtr += 2;
			} while (len -= 2);

			/*
			* Actually Write the buffer to flash (and wait for it to finish.)
			*/
			__boot_page_write_short((uint16_t)(void*)address);
			boot_spm_busy_wait();
			#if defined(RWWSRE)
			// Reenable read access to flash
			boot_rww_enable();
			#endif
		} // default block
		break;
	} // switch
}

static inline void read_mem(uint8_t memtype, uint16_t address, pagelen_t length)
{
	uint8_t ch;

	switch (memtype) {

		#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
		case 'E': // EEPROM
		do {
			putch(eeprom_read_byte((uint8_t *)(address++)));
		} while (--length);
		break;
		#endif
		default:
		do {
			#ifdef VIRTUAL_BOOT_PARTITION
			// Undo vector patch in bottom page so verify passes
			if (address == 0)       ch=rstVect & 0xff;
			else if (address == 1)  ch=rstVect >> 8;
			else if (address == 8)  ch=wdtVect & 0xff;
			else if (address == 9) ch=wdtVect >> 8;
			else ch = pgm_read_byte_near(address);
			address++;
			#elif defined(RAMPZ)
			// Since RAMPZ should already be set, we need to use EPLM directly.
			// Also, we can use the autoincrement version of lpm to update "address"
			//      do putch(pgm_read_byte_near(address++));
			//      while (--length);
			// read a Flash and increment the address (may increment RAMPZ)
			__asm__ ("elpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
			#else
			// read a Flash byte and increment the address
			__asm__ ("lpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
			#endif
			putch(ch);
		} while (--length);
		break;
	} // switch
}
