

#ifndef HARDWARE_PROFILE_PICDEM_FSUSB_H //para funcionar com 4550 se usa esse define
#define HARDWARE_PROFILE_PICDEM_FSUSB_H

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //The PICDEM FS USB Demo Board platform supports the USE_SELF_POWER_SENSE_IO
    //and USE_USB_BUS_SENSE_IO features.  Uncomment the below line(s) if
    //it is desireable to use one or both of the features.
    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTAbits.RA2
    #else
    #define self_power          1
    #endif

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISAbits.TRISA1    // Input
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTAbits.RA1
    #else
    #define USB_BUS_SENSE       1
    #endif


    //Uncomment the following line to make the output HEX of this
    //  project work with the MCHPUSB Bootloader
    //#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER

    //Uncomment the following line to make the output HEX of this
    //  project work with the HID Bootloader
    #define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    #define DEMO_BOARD PICDEM_FS_USB
    #define PICDEM_FS_USB
    #define CLOCK_FREQ 48000000
    #define GetSystemClock() (CLOCK_FREQ)

    /** RS - 232  ****************************************************/
    #define UART_TRISTx   TRISCbits.TRISC6
    #define UART_TRISRx   TRISCbits.TRISC7
    #define UART_Tx       PORTCbits.RC6
    #define UART_Rx       PORTCbits.RC7
    #define UART_ENABLE	  RCSTAbits.SPEN

    /** USB tranceiver externo (opcional) ******************/
    #define tris_usb_vpo        TRISBbits.TRISB3    // Output
    #define tris_usb_vmo        TRISBbits.TRISB2    // Output
    #define tris_usb_rcv        TRISAbits.TRISA4    // Input
    #define tris_usb_vp         TRISCbits.TRISC5    // Input
    #define tris_usb_vm         TRISCbits.TRISC4    // Input
    #define tris_usb_oe         TRISCbits.TRISC1    // Output
    #define tris_usb_suspnd     TRISAbits.TRISA3    // Output

    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0

    //pinos da uart (se precisar)

    #define UART_DTS PORTBbits.RB4
    #define UART_DTR LATDbits.LATD3
    #define UART_RTS LATAbits.LATA2
    #define UART_CTS PORTAbits.RA3

    #define mInitRTSPin() {TRISAbits.TRISA2 = 0;}   //Configura RTS como saida digital.
    #define mInitCTSPin() {TRISAbits.TRISA3 = 1;}   //Configura CTS como entrada.  (Make sure pin is digital if ANxx functions is present on the pin)
    #define mInitDTSPin() {TRISBbits.TRISB4 = 1;}   //Configura DTS como entrada.  (Make sure pin is digital if ANxx functions is present on the pin)
    #define mInitDTRPin() {TRISDbits.TRISD3 = 0;}   //Configura DTR como saida.
#endif  
