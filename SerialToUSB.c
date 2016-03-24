/* 
 * Serial to USB HID
 */

#include "SerialToUSB.h"

/** Buffer to hold the previously generated Mouse HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevMouseHIDReportBuffer[sizeof(USB_MouseReport_Data_t)];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[128];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Mouse_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_Mouse,
				.ReportINEndpoint             =
					{
						.Address              = MOUSE_EPADDR,
						.Size                 = MOUSE_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevMouseHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevMouseHIDReportBuffer),
			},
	};



#define LED 6 // PE6

#define USART_BAUDRATE 1200
#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2) ) / ( USART_BAUDRATE )) - 1)

volatile char receivedByte = 0;

int main(void)
{
  SetupHardware();
  RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));

  LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
  GlobalInterruptEnable();
  
  for (;;) {
    HID_Device_USBTask(&Mouse_HID_Interface);
    USB_USBTask();
  }

  return 0;
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
        uint8_t ReceivedByte = UDR1;

        if ((USB_DeviceState == DEVICE_STATE_Configured) && !(RingBuffer_IsFull(&USARTtoUSB_Buffer)))
          RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

  UCSR1B = (1 << RXEN1) | (1 << TXEN1); // Turn on the transmission and reception circuitry
  UCSR1C = (1 << UCSZ11) | (0 << UCSZ10); // Use 7-bit character sizes
  UBRR1H = (BAUD_PRESCALE >> 8);   // upper 8 bits of baud
  UBRR1L = (BAUD_PRESCALE & 0xff); // lower 8 bits

  UCSR1B |= (1 << RXCIE1 ); // enable receive complete interrupt

	/* Hardware Initialization */
	LEDs_Init();
	Buttons_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Mouse_HID_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Mouse_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Mouse_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
      USB_MouseReport_Data_t* MouseReport = (USB_MouseReport_Data_t*)ReportData;

      char button = 0;
      int x = 0;
      int y = 0; 

      uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
      if (BufferCount > 2)
      {
        unsigned char s[3];
        while (BufferCount--)
        {
          s[2 - BufferCount] = RingBuffer_Remove(&USARTtoUSB_Buffer);
        }

        DecodeMouse(s, &button, &x, &y); 
    }

    if (x > 0)
     MouseReport->X = 1;
    if (x < 0)
     MouseReport->X = -1;
    if (y > 0)
     MouseReport->Y = 1;
    if (y < 0)
     MouseReport->Y = -1;

    if (button == 'l')
     MouseReport->Button |= (1 << 0);

    if (button == 'r')
     MouseReport->Button |= (1 << 1);

    *ReportSize = sizeof(USB_MouseReport_Data_t);
    return true;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}

/*
   s should consist of 3 bytes from the mouse
*/
void DecodeMouse(unsigned char *s, char *button, int *x, int *y)
{
   *button = 'n'; /* No button - should only happen on an error */
   if ((s[0] & 0x20) != 0)
      *button = 'l';
   else if ((s[0] & 0x10) != 0)
      *button = 'r';
   *x = (s[0] & 0x03) * 64 + (s[1] & 0x3F);
   if (*x > 127)
      *x = *x - 256;
   *y = (s[0] & 0x0C) * 16 + (s[2] & 0x3F);
   if (*y > 127)
      *y = *y - 256;
}
