/*
LUFA Library
Copyright (C) Dean Camera, 2014.

dean [at] fourwalledcubicle [dot] com
www.lufa-lib.org
*/

/*
Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

Permission to use, copy, modify, distribute, and sell this
software and its documentation for any purpose is hereby granted
without fee, provided that the above copyright notice appear in
all copies and that both that the copyright notice and this
permission notice and warranty disclaimer appear in supporting
documentation, and that the name of the author not be used in
advertising or publicity pertaining to distribution of the
software without specific, written prior permission.

The author disclaims all warranties with regard to this
software, including all implied warranties of merchantability
and fitness.  In no event shall the author be liable for any
special, indirect or consequential damages or any damages
whatsoever resulting from loss of use, data or profits, whether
in an action of contract, negligence or other tortious action,
arising out of or in connection with the use or performance of
this software.
*/

/*
Copyright(c) 2014-2015 NicoHood
See the readme for credit to other people.

This file is part of Hoodloader2.

Hoodloader2 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Hoodloader2 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Hoodloader2.  If not, see <http://www.gnu.org/licenses/>.
*/

/** \file
*
*  Main source file for the CDC class bootloader. This file contains the complete bootloader logic.
*/

#include "hoodserial.h"

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_RX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};


/* NOTE: Using Linker Magic,
* - Reserved 256 bytes from start of RAM at 0x100 for UART RX Buffer
* so we can use 256-byte aligned addresssing.
* - Also 128 bytes from 0x200 for UART TX buffer, same addressing.
* normal RAM data starts at 0x280, see offset in makefile*/

#define USART2USB_BUFLEN 256 // 0xFF - 8bit
#define USB2USART_BUFLEN 128 // 0x7F - 7bit

// USB-Serial buffer pointers are saved in GPIORn
// for better access (e.g. cbi) in ISRs
// This has nothing to do with r0 and r1!
// GPIORn – General Purpose I/O Register are located in RAM.
// Make sure to set DEVICE_STATE_AS_GPIOR to 2 in the Lufa config.
// They are initialied in the CDC LineEncoding Event
#define USBtoUSART_ReadPtr GPIOR0 // to use cbi()
#define USARTtoUSB_WritePtr GPIOR1

/* USBtoUSART_WritePtr needs to be visible to ISR. */
/* USARTtoUSB_ReadPtr needs to be visible to CDC LineEncoding Event. */
static volatile uint8_t USBtoUSART_WritePtr = 0;
static volatile uint8_t USARTtoUSB_ReadPtr = 0;


/** Main program entry point. This routine configures the hardware required by the bootloader, then continuously
*  runs the bootloader processing routine until instructed to soft-exit, or hard-reset via the watchdog to start
*  the loaded application code.
*/
int main(void)
{
	/* Setup hardware required for the bootloader */
	SetupHardware();

	/* Enable global interrupts so that the USB stack can function */
	GlobalInterruptEnable();

	while(true) {
		// Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type
		#define TX_RX_LED_PULSE_MS 3
		uint8_t TxLEDPulse = 0;
		uint8_t RxLEDPulse = 0;

		// USB-Serial main loop
		// Process USB_Task to get device configured
		USB_USBTask();
		while(USB_DeviceState == DEVICE_STATE_Configured) {
			// USB Transer Main Loop
			/* Check if endpoint has a command in it sent from the host */
			Endpoint_SelectEndpoint(CDC_RX_EPADDR);
			uint8_t countRX = 0;

			if (Endpoint_IsOUTReceived()){
				// Check if we received any new bytes and if we still have space in the buffer
				countRX = Endpoint_BytesInEndpoint();

				// Acknowledge zero length packet and dont call any read functions
				if (!countRX)
					Endpoint_ClearOUT();
			}


			//================================================================================
			// USBtoUSART
			//================================================================================

			// Check how much free space the USBtoUSART buffer has
			uint8_t USBtoUSART_free = (USB2USART_BUFLEN-1) - ( (USBtoUSART_WritePtr - USBtoUSART_ReadPtr) & (USB2USART_BUFLEN-1) );

			// Read new data from the USB host if we still have space in the buffer
			if(countRX && countRX <= USBtoUSART_free )
			{
				// Prepare temporary pointer
				uint16_t tmp; // = 0x200 | USBtoUSART_WritePtr;
				asm (
					"ldi %B[tmp], 0x02\n\t"			// (1) Force high byte to 0x200
					"lds %A[tmp], %[writePtr]\n\t"	// (1) Load USBtoUSART_WritePtr into low byte
					// Outputs
					: [tmp] "=&e" (tmp)	// Pointer register, output only
					// Inputs
					: [writePtr] "m" (USBtoUSART_WritePtr) // Memory location
				);

				// Save USB bank into our USBtoUSART ringbuffer
				do {
					register uint8_t data;
					data = Endpoint_Read_8();
					asm (
						"st %a[tmp]+, %[data]\n\t" 	// (2) Save byte in buffer and increment
						"andi %A[tmp], 0x7F\n\t" 	// (1) Wrap around pointer, 128 bytes
						// Outputs
						: [tmp] "=e" (tmp) // Input and output
						// Inputs
						: "0" (tmp), [data] "r" (data)
					);
				} while (--countRX);

				// Acknowledge data
				Endpoint_ClearOUT();

				// Save back new pointer position
				// Just save the lower byte of the pointer
				USBtoUSART_WritePtr = tmp & 0xFF;

				// Enable USART again to flush the buffer
				UCSR1B = (_BV(RXCIE1) | _BV(TXEN1) | _BV(RXEN1) | _BV(UDRIE1));

				// Force Leds to turn on
				USBtoUSART_free = 0;
			}

			// Light RX led if we still have data in the USBtoUSART buffer
			if (USBtoUSART_free != (USB2USART_BUFLEN-1)) {
				LEDs_TurnOnRXLED;
				RxLEDPulse = TX_RX_LED_PULSE_MS;
			}

			//================================================================================
			// USARTtoUSB
			//================================================================================

			// This requires the USART RX buffer to be 256 bytes.
			uint8_t count = USARTtoUSB_WritePtr - USARTtoUSB_ReadPtr;

			// Check if we have something worth to send
			if (count) {

				// Check if the UART receive buffer flush timer has expired or the buffer is nearly full
				if ((TIFR0 & (1 << TOV0)) || (count >= (CDC_TX_EPSIZE - 1)) )
				{
					// Send data to the USB host
					Endpoint_SelectEndpoint(CDC_TX_EPADDR);

					// CDC device is ready for receiving bytes
					if (Endpoint_IsINReady())
					{
						// Send a maximum of up to one bank minus one.
						// If we fill the whole bank we'd have to send an empty Zero Length Packet (ZLP)
						// afterwards to determine the end of the transfer.
						// Since this is more complicated we only send single packets
						// with one byte less than the maximum.
						uint8_t txcount = CDC_TX_EPSIZE - 1;
						if (txcount > count)
							txcount = count;

						// Prepare temporary pointer
						uint16_t tmp; // = 0x100 | USARTtoUSBReadPtr
						asm (
							// Do not initialize high byte, it will be done in first loop below.
							"lds %A[tmp], %[readPtr]\n\t"	// (1) Copy read pointer into lower byte
							// Outputs
							: [tmp] "=&e" (tmp)	// Pointer register, output only
							// Inputs
							: [readPtr] "m" (USARTtoUSB_ReadPtr) // Memory location
						);

						// Write all bytes from USART to the USB endpoint
						do {
							register uint8_t data;
							asm (
								"ldi %B[tmp] , 0x01\n\t" 		// (1) Force high byte to 0x01
								"ld %[data] , %a[tmp] +\n\t" 	// (2) Load next data byte, wraps around 255
								// Outputs
								: [data] "=&r" (data),	// Output only
								[tmp] "=e" (tmp) 		// Input and output
								// Inputs
								: "1" (tmp)
							);
							Endpoint_Write_8(data);
						} while (--txcount);

						// Send data to USB Host now
						Endpoint_ClearIN();

						// Save new pointer position
						USARTtoUSB_ReadPtr = tmp & 0xFF;
					}
				}

				// Light TX led if there is data to be send
				LEDs_TurnOnTXLED;
				TxLEDPulse = TX_RX_LED_PULSE_MS;
			}

			// LED timer overflow.
			// Check Leds (this methode takes less flash than an ISR)
			if (TIFR0 & (1 << TOV0)){
				// Reset the timer
				// http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_intbits
				TIFR0 = (1 << TOV0);

				// Turn off TX LED once the TX pulse period has elapsed
				if (TxLEDPulse && !(--TxLEDPulse))
				LEDs_TurnOffTXLED;

				// Turn off RX LED once the RX pulse period has elapsed
				if (RxLEDPulse && !(--RxLEDPulse))
				LEDs_TurnOffRXLED;
			}

			USB_USBTask();
		};

		// Dont forget LEDs on if suddenly unconfigured.
		// TODO:  Need to set baud rate to 0, how do I do that internally?
		LEDs_TurnOffTXLED;
		LEDs_TurnOffRXLED;
	}
}

/** Configures all hardware required for the bootloader. */
static void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
    /* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	// Disable clock division
	clock_prescale_set(clock_div_1);
#endif

	/* Initialize the USB and other board hardware drivers */
	USB_Init();

	/* Start the flush timer for Leds */
	TCCR0B = (1 << CS02); // clk I/O / 256 (From prescaler)

	// Inits Serial pins, leds, reset and erase pins
	Board_Init();

	LEDs_TurnOffTXLED;
	LEDs_TurnOffRXLED;
}



/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 * Connects the Arduino reset line to the DTR signal.
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
	Board_Reset(CurrentDTRState);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
*  for later transmission to the host.
*/
ISR(USART1_RX_vect, ISR_NAKED)
{
	// This ISR doesnt change SREG. Whoa.
	asm volatile (
		"lds r3, %[UDR1_Reg]\n\t" 		// (1) Load new Serial byte (UDR1) into r3
		"movw r4, r30\n\t" 				// (1) Backup Z pointer (r30 -> r4, r31 -> r5)
		"in r30, %[writePointer]\n\t" 	// (1) Load USARTtoUSB write buffer 8 bit pointer to lower Z pointer
		"ldi r31, 0x01\n\t" 			// (1) Set higher Z pointer to 0x01
		"st Z+, r3\n\t" 				// (2) Save UDR1 in Z pointer (USARTtoUSB write buffer) and increment
		"out %[writePointer], r30\n\t" 	// (1) Save back new USARTtoUSB buffer pointer location
		"movw r30, r4\n\t" 				// (1) Restore backuped Z pointer
		"reti\n\t"						// (4) Exit ISR

		// Inputs:
		:: [UDR1_Reg] "m" (UDR1), 		// Memory location of UDR1
		[writePointer] "I" (_SFR_IO_ADDR(USARTtoUSB_WritePtr)) // 8 bit pointer to USARTtoUSB write buffer
	);
}

ISR(USART1_UDRE_vect, ISR_NAKED)
{
	// Another SREG-less ISR.
	asm volatile (
		"movw r4, r30\n\t" 					// (1) Backup Z pointer (r30 -> r4, r31 -> r5)
		"in r30, %[readPointer]\n\t"		// (1) Load USBtoUSART read buffer 8 bit pointer to lower Z pointer
		"ldi r31, 0x02\n\t" 				// (1) Set higher Z pointer to 0x02
		"ld r3, Z+\n\t" 					// (2) Load next byte from USBtoUSART buffer into r3
		"sts %[UDR1_Reg], r3\n\t"			// (2) Save r3 (next byte) in UDR1
		"out %[readPointer], r30\n\t" 		// (1) Save back new USBtoUSART read buffer pointer location
		"cbi %[readPointer], 7\n\t" 		// (2) Wrap around for 128 bytes
		//     smart after-the-fact andi 0x7F without using SREG
		"movw r30, r4\n\t"					// (1) Restore backuped Z pointer
		"in r2, %[readPointer]\n\t"			// (1) Load USBtoUSART read buffer 8 bit pointer to r2
		"lds r3, %[writePointer]\n\t" 		// (1) Load USBtoUSART write buffer to r3
		"cpse r2, r3\n\t"					// (1/2) Check if USBtoUSART read buffer == USBtoUSART write buffer
		"reti\n\t"							// (4) They are not equal, more bytes coming soon!
		"ldi r30, 0x98\n\t"					// (1) Set r30 temporary to new UCSR1B setting ((1<<RXCIE1) | (1 << RXEN1) | (1 << TXEN1))
		//     ldi needs an upper register, restore Z once more afterwards
		"sts %[UCSR1B_Reg], r30\n\t"		// (2) Turn off this interrupt (UDRIE1), all bytes sent
		"movw r30, r4\n\t"					// (1) Restore backuped Z pointer again (was overwritten again above)
		"reti\n\t"							// (4) Exit ISR

		// Inputs:
		:: [UDR1_Reg] "m" (UDR1),
		[readPointer] "I" (_SFR_IO_ADDR(USBtoUSART_ReadPtr)), 	// 7 bit pointer to USBtoUSART read buffer
		[writePointer] "m" (USBtoUSART_WritePtr), 				// 7 bit pointer to USBtoUSART write buffer
		[UCSR1B_Reg] "m" (UCSR1B)			// Memory location of UDR1
	);
}


/** Event handler for the CDC Class driver Line Encoding Changed event.
*
*  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
*/
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	/* Keep the TX line held high (idle) while the USART is reconfigured */
	PORTD |= (1 << 3);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Flush data that was about to be sent. */
	USBtoUSART_ReadPtr = 0;
	USBtoUSART_WritePtr = 0;
	USARTtoUSB_ReadPtr = 0;
	USARTtoUSB_WritePtr = 0;

	// Only reconfigure USART if the CDC Serial is not disabled
	uint32_t BaudRateBPS = CDCInterfaceInfo->State.LineEncoding.BaudRateBPS;
	uint8_t ConfigMask = 0;
	if (BaudRateBPS) {

		switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
		{
			case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
			case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
		}

		if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
		ConfigMask |= (1 << USBS1);

		switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
		{
			case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
			case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
			case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
		}

		// Set the new baud rate before configuring the USART
		uint8_t clockSpeed = (1 << U2X1);
		uint16_t brr = SERIAL_2X_UBBRVAL(BaudRateBPS);

		// Or special case 57600 baud for compatibility with the ATmega328 bootloader.
		// Also handle situations where U2X is not needed or not possible.
		if ((brr & 1) || (brr > 4095)) {
			brr >>= 1;
			clockSpeed = 0;
		}
		else if(BaudRateBPS == 57600) {
			brr = SERIAL_UBBRVAL(BaudRateBPS);
			clockSpeed = 0;
		}

		UBRR1 = brr;

		// Reconfigure the USART
		UCSR1C = ConfigMask;
		UCSR1A = clockSpeed;
		UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
	}

	/* Release the TX line after the USART has been reconfigured */
	PORTD &= ~(1 << 3);
}
