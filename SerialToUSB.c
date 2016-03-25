/* 
 * Serial mouse to USB HID using LUFA.
 * Cribbed heavily from LUFA USBtoSerial and ClassDriver/Mouse demos.
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
        .InterfaceNumber          = INTERFACE_ID_Mouse,
        .ReportINEndpoint         =
        {
            .Address              = MOUSE_EPADDR,
            .Size                 = MOUSE_EPSIZE,
            .Banks                = 1,
        },
        .PrevReportINBuffer       = PrevMouseHIDReportBuffer,
        .PrevReportINBufferSize   = sizeof(PrevMouseHIDReportBuffer),
    },
};


#define USART_BAUDRATE 1200
#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2) ) / ( USART_BAUDRATE )) - 1)

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
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

    MySerial_Init(USART_BAUDRATE);

    /* Hardware Initialization */
    LEDs_Init();
    Buttons_Init();
    USB_Init();
}

/** Initializes the USART, ready for serial data transmission and reception. This initializes the interface to
 *  7-bit, no parity, 1 stop bit settings suitable for Microsoft mouse protocol use.
 *
 *  \param[in] BaudRate     Serial baud rate, in bits per second.
 */
void MySerial_Init(const uint32_t BaudRate) {
    UBRR1H = (BAUD_PRESCALE >> 8);   // upper 8 bits of baud
    UBRR1L = (BAUD_PRESCALE & 0xff); // lower 8 bits

    UCSR1C = (1 << UCSZ11) | (0 << UCSZ10); // Use 7-bit character sizes
    UCSR1A = 0; // Not double speed
    UCSR1B = (1 << RXEN1) | (1 << TXEN1); // Turn on the transmission and reception circuitry

    UCSR1B |= (1 << RXCIE1 ); // enable receive complete interrupt

	DDRD  |= (1 << 3);
	PORTD |= (1 << 2);

    /* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
    // TODO rgm: Read the datasheet and learn about this
    //TCCR0B = (1 << CS02);
    //UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));


    /* LUFA wants to do this in its Serial_Init, investigate differences.
	UBRR1  = SERIAL_UBBRVAL(BaudRate);
	UCSR1C = ((1 << UCSZ11) | (1 << UCSZ10));
	UCSR1A = (DoubleSpeed ? (1 << U2X1) : 0);
	UCSR1B = ((1 << TXEN1)  | (1 << RXEN1));
	DDRD  |= (1 << 3);
	PORTD |= (1 << 2);
    */
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

    // Read the 3-byte mouse data packet from the serial buffer
    uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
    
    if (BufferCount >= 3)
    {
        unsigned char data[3];
        for (int i = 0; i <= 2; i++)
        {
            data[i] = RingBuffer_Remove(&USARTtoUSB_Buffer);
            if (data[i] == 'M' || data[i] == '3') // Ignore mouse init messages
            {
                *ReportSize = 0;
                return false;
            }
        }

        Decode_MS(MouseReport, data);

        MouseReport->X = CONSTRAIN(MouseReport->X, -10, 10);
        MouseReport->Y = CONSTRAIN(MouseReport->Y, -10, 10);

        *ReportSize = sizeof(USB_MouseReport_Data_t);
        return true;
    }

    *ReportSize = 0;
    return false;
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

/** Microsoft Serial Mouse decoder.
 *
 * Thanks to Paul Bourke.
 * http://paulbourke.net/dataformats/serialmouse/
 *
 * Every time the mouse changes state (moved or button pressed) a three byte "packet" is sent to the serial interface.
 *
 * \param[in]   s Pointer to 3 byte character array of serial data received from the mouse
 * \param[out]  button  Pointer to a character representing button state - 'l' for left, 'r' for right
 * \param[out]  x       Pointer to the X displacement integer, from -127 to +127
 * \param[out]  y       Pointer to the Y displacement integer, from -127 to +127
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

void Decode_MS(USB_MouseReport_Data_t *MouseReport, unsigned char *data)
{
   // some devices report a change of middle-button state by
   // repeating the current button state  (patch by Mark Lord)
   static unsigned char prev=0;
   MouseReport->Button = 0;

   if (data[0] == 0x40 && !(prev|data[1]|data[2])) {
      MouseReport->Button |= (1 << 2);  //GPM_B_MIDDLE); // third button on MS compatible mouse 
   }
   else {
      /* GPM uses bits in the opposite order:
        gpmleft   = 4  100
        gpmmiddle = 2  010
        gpmright =  1  001
        none     =     000
      */
      MouseReport->Button = ((data[0] & 0x20) >> 5) | ((data[0] & 0x10) >> 3);
   }
   prev = MouseReport->Button;
   MouseReport->X=      (signed char)(((data[0] & 0x03) << 6) | (data[1] & 0x3F));
   MouseReport->Y=      (signed char)(((data[0] & 0x0C) << 4) | (data[2] & 0x3F));
}

// from gpm mice.c
/*
static int M_ms(Gpm_Event *state,  unsigned char *data)
{
    // some devices report a change of middle-button state by
    // repeating the current button state  (patch by Mark Lord)
   static unsigned char prev=0;

   if (data[0] == 0x40 && !(prev|data[1]|data[2]))
      state->buttons = GPM_B_MIDDLE; // third button on MS compatible mouse 
   else
      state->buttons= ((data[0] & 0x20) >> 3) | ((data[0] & 0x10) >> 4);
   prev = state->buttons;
   state->dx=      (signed char)(((data[0] & 0x03) << 6) | (data[1] & 0x3F));
   state->dy=      (signed char)(((data[0] & 0x0C) << 4) | (data[2] & 0x3F));

   return 0;
}
*/




// vim: noai:ts=4:sw=4:expandtab
