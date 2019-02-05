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
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions.
 */

#include "Descriptors.h"

// Only use RAM Descriptors if we have enough ram
#ifdef USE_RAM_DESCRIPTORS
#define DESCRIPTOR_PROGMEM
#define STRING_PROGMEM(x) (x)
#else // PROGMEM descriptors
#define DESCRIPTOR_PROGMEM PROGMEM
#define STRING_PROGMEM(x) pgm_read_byte(&x)
#endif

/** Device descriptor structure. This descriptor, located in SRAM memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
const USB_Descriptor_Device_t DESCRIPTOR_PROGMEM DeviceDescriptor =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},

	.USBSpecification       = VERSION_BCD(1,1,0),
	.Class                  = CDC_CSCP_CDCClass,
	.SubClass               = CDC_CSCP_NoSpecificSubclass,
	.Protocol               = CDC_CSCP_NoSpecificProtocol,

	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,

	// passed through makefile
	.VendorID 				= VENDORID,
	.ProductID 				= PRODUCTID,
	.ReleaseNumber          = VERSION_BCD(1,0,0),

	.ManufacturerStrIndex   = STRING_ID_Manufacturer,
	.ProductStrIndex        = STRING_ID_Product,
#if defined(SERIALID) || defined(PRUSA_MK3_SERIAL)
	.SerialNumStrIndex      = STRING_ID_Serial,
#else
	.SerialNumStrIndex      = USE_INTERNAL_SERIAL,
#endif
	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in SRAM memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
const USB_Descriptor_Configuration_t DESCRIPTOR_PROGMEM ConfigurationDescriptor =
{
	.Config =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize = sizeof(USB_Descriptor_Configuration_t),
			.TotalInterfaces        = 2,

			.ConfigurationNumber    = 1,
			.ConfigurationStrIndex  = NO_DESCRIPTOR,

			.ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),

			.MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
		},

	.CDC_CCI_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = INTERFACE_ID_CDC_CCI,
			.AlternateSetting       = 0,

			.TotalEndpoints         = 1,

			.Class                  = CDC_CSCP_CDCClass,
			.SubClass               = CDC_CSCP_ACMSubclass,
			.Protocol               = CDC_CSCP_ATCommandProtocol,

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC_Functional_Header =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t), .Type = CDC_DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_Header,

			.CDCSpecification       = VERSION_BCD(1,1,0),
		},

	.CDC_Functional_ACM =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t), .Type = CDC_DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_ACM,

			.Capabilities           = 0x06,
		},

	.CDC_Functional_Union =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t), .Type = CDC_DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_Union,

			.MasterInterfaceNumber  = INTERFACE_ID_CDC_CCI,
			.SlaveInterfaceNumber   = INTERFACE_ID_CDC_DCI,
		},

	.CDC_NotificationEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = CDC_NOTIFICATION_EPADDR,
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_NOTIFICATION_EPSIZE,
			.PollingIntervalMS      = 0xFF
		},

	.CDC_DCI_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = INTERFACE_ID_CDC_DCI,
			.AlternateSetting       = 0,

			.TotalEndpoints         = 2,

			.Class                  = CDC_CSCP_CDCDataClass,
			.SubClass               = CDC_CSCP_NoDataSubclass,
			.Protocol               = CDC_CSCP_NoDataProtocol,

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC_DataOutEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = CDC_RX_EPADDR,
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_RX_EPSIZE,
			.PollingIntervalMS      = 0x01
		},

	.CDC_DataInEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = CDC_TX_EPADDR,
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_TX_EPSIZE,
			.PollingIntervalMS      = 0x01
		}
};

/** Language descriptor structure. This descriptor, located in SRAM memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
const USB_Descriptor_String_t DESCRIPTOR_PROGMEM LanguageString = USB_STRING_DESCRIPTOR_ARRAY(LANGUAGE_ID_ENG);

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
const USB_Descriptor_String_t DESCRIPTOR_PROGMEM ManufacturerString = USB_STRING_DESCRIPTOR(MANUFACTURER_STRING);

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
const USB_Descriptor_String_t DESCRIPTOR_PROGMEM ProductString = USB_STRING_DESCRIPTOR(PRODUCT_STRING);

#ifdef SERIALID
const USB_Descriptor_String_t DESCRIPTOR_PROGMEM SerialString = USB_STRING_DESCRIPTOR(SERIALID);
#endif

#ifdef PRUSA_MK3_SERIAL
static void read_prusa_serial(void)
{
	uint8_t* addr = (uint8_t*)256;
	uint8_t count = 0;
	uint8_t serial_bytes[32];
	while(count < 32)
	{
		serial_bytes[count] = eeprom_read_byte(addr);
		if (serial_bytes[count] == 0xFF)
			break;
		serial_bytes[count] &= 0x7F;
		count++;
		addr++;
	}

	if (!count)
		return;

	struct
	{
		USB_Descriptor_Header_t Header;
		uint16_t                UnicodeString[count];
	} SerialDescriptor;

	SerialDescriptor.Header.Type = DTYPE_String;
	SerialDescriptor.Header.Size = USB_STRING_LEN(count);

	for (uint8_t i = 0; i < count; i++) {
		SerialDescriptor.UnicodeString[i] = cpu_to_le16(serial_bytes[i]);
	}

	Endpoint_ClearSETUP();

	Endpoint_Write_Control_Stream_LE(&SerialDescriptor, sizeof(SerialDescriptor));
	Endpoint_ClearOUT();
}
#endif

/** This function is called by the library when in device mode, and must be overridden (see LUFA library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void* Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

	switch (DescriptorType)
	{
		case DTYPE_Device:
			Address = &DeviceDescriptor;
			Size    = sizeof(USB_Descriptor_Device_t);
			break;
		case DTYPE_Configuration:
			Address = &ConfigurationDescriptor;
			Size    = sizeof(USB_Descriptor_Configuration_t);
			break;
		case DTYPE_String:
			if (DescriptorNumber == STRING_ID_Language)
			{
				Address = &LanguageString;
				Size    = STRING_PROGMEM(LanguageString.Header.Size);
			}
			else if (DescriptorNumber == STRING_ID_Manufacturer)
			{
				Address = &ManufacturerString;
				Size    = STRING_PROGMEM(ManufacturerString.Header.Size);
			}
			else if (DescriptorNumber == STRING_ID_Product)
			{
				Address = &ProductString;
				Size    = STRING_PROGMEM(ProductString.Header.Size);
			}
#if defined(SERIALID)
			else if (DescriptorNumber == STRING_ID_Serial)
			{
				Address = &SerialString;
				Size    = STRING_PROGMEM(SerialString.Header.Size);
			}
#elif defined(PRUSA_MK3_SERIAL)
			else if (DescriptorNumber == STRING_ID_Serial)
			{
				read_prusa_serial();
			}
#endif
			break;
	}

	*DescriptorAddress = Address;
	return Size;
}

