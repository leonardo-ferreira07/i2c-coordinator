/*

Nome:
 * 
- Leonardo Kaminski Ferreira
 *
 *                      MASTER I2C
 * 
Engenharia da Computação - 3º Ano - Manhã - Arquitetura e Organização de Computadores


 */
#include <p18cxxx.h>
#include "./USB/usb.h"//chama todas as lib necessarias
#include "/MPLAB.X/HardwareProfile.h" //aqui está configurado o clock e os pinos do microcontrolador
#include "GenericTypeDefs.h"//lib que define os tipos utilizados
#include "./USB/usb_function_cdc.h"//funçoes da serial
#include "Compiler.h"//aqui se define o compilador que está sendo utilizado
#include "usb_config.h"//configura a usb
#include "USB/usb_device.h"
#include "delays.h"
#include "stdlib.h"
#include "stdio.h"
#include "i2c.h"



//////////////config. dos fuses do microcontrolador///////////////////////////////
#pragma config PLLDIV   = 5         // (usei um cristal de 20 MHz )
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // o clock do pll será 96MHz PLL/2 = 48Mhz
#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = ON // alterado
#pragma config IESO     = ON // alterado
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON      //ativa o regulador de voltagem da USB
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
//      #pragma config CCP2MX   = ON
#pragma config STVREN   = ON
#pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
#pragma config XINST    = OFF       // Extended Instruction Set
#pragma config CP0      = OFF
#pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
#pragma config CPB      = OFF
//      #pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
#pragma config WRTB     = OFF       // proteção de escrita da inicialização (Boot)
#pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF

/////////////////////variáveis requerida pela USB /////////////////////////////////////////////////////

#pragma udata
char USB_Out_Buffer[CDC_DATA_OUT_EP_SIZE];
char RS232_Out_Data[CDC_DATA_IN_EP_SIZE];

unsigned char NextUSBOut;
unsigned char NextUSBOut;
//char RS232_In_Data;
unsigned char LastRS232Out; // Number of characters in the buffer
unsigned char RS232cp; // current position within the buffer
unsigned char RS232_Out_Data_Rdy = 0;
USB_HANDLE lastTransmission;
BOOL check = 0;
BOOL lixo = 0;
int contador = 0;

unsigned char string_pwm[] = "xx", string_intensidade[] = "xxx";
unsigned char comando[6];



//BOOL stringPrinted;


/**********protótipo das funções ***************************************/
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
//void BlinkUSBStatus(void);
void UserInit(void);
void InitializeUSART(void);
unsigned char getcUSART();
void enviarUSB(char *msg);
unsigned int filtro();
int recebe_usb_i2c();


/********** remapeando vetores ****************************/
#define REMAPPED_RESET_VECTOR_ADDRESS		0x1000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#define APP_VERSION_ADDRESS                     0x1016 //Fixed location, so the App FW image version can be read by the bootloader.
#define APP_SIGNATURE_ADDRESS                   0x1006 //Signature location that must be kept at blaknk value (0xFFFF) in this project (has special purpose for bootloader).

#define APP_FIRMWARE_VERSION_MAJOR  1   //valid values 0-255
#define APP_FIRMWARE_VERSION_MINOR  0   //valid values 0-99

/////////////////defines da porta serial /////////////////////////////

#define mDataRdyUSART() PIR1bits.RCIF
#define mTxRdyUSART()   TXSTAbits.TRMT
//--------------------------------------------------------------------------






//--------------------------------------------------------------------------


#pragma romdata AppVersionAndSignatureSection = APP_VERSION_ADDRESS
ROM unsigned char AppVersion[2] = {APP_FIRMWARE_VERSION_MINOR, APP_FIRMWARE_VERSION_MAJOR};
#pragma romdata AppSignatureSection = APP_SIGNATURE_ADDRESS
ROM unsigned short int SignaturePlaceholder = 0xFFFF;

#pragma code HIGH_INTERRUPT_VECTOR = 0x08

void High_ISR(void) {
    _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18

void Low_ISR(void) {
    _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
extern void _startup(void); // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS

void _reset(void) {
    _asm goto _startup _endasm
}
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS

void Remapped_High_ISR(void) {
    _asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS

void Remapped_Low_ISR(void) {
    _asm goto YourLowPriorityISRCode _endasm
}

#pragma code

#pragma interrupt YourHighPriorityISRCode
void YourHighPriorityISRCode() {

    

    #if defined(USB_INTERRUPT)
        USBDeviceTasks();
    #endif




    


} //This return will be a "retfie fast", since this is in a #pragma interrupt section


#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode() {



} //This return will be a "retfie", since this is in a #pragma interruptlow section

#pragma code
static void inicializa_pic(void)
{
    ADCON1 |= 0x0F; // coloca todos os pinos como saida
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // veja HardwareProfile.h
    #endif
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // veja HardwareProfile.h
    #endif

    UserInit();
    USBDeviceInit(); //usb_device.c.  inicializa o modulo USB e os SFRs
    //variaveis para saber o status.
}//fim de inicializa_cpu

void UserInit(void) {
    unsigned char i;
    InitializeUSART();
    // 	inicializa vetor
    for (i = 0; i<sizeof (USB_Out_Buffer); i++) {
        USB_Out_Buffer[i] = 0;
    }
    NextUSBOut = 0;
    LastRS232Out = 0;
    lastTransmission = 0;
}//end UserInit

void InitializeUSART(void) {
    unsigned char c;
    UART_TRISRx = 1; // RX
    UART_TRISTx = 0; // TX
    TXSTA = 0x24; // TX enable BRGH=1
    RCSTA = 0x90; // Single Character RX
    SPBRG = 0x71;
    SPBRGH = 0x02; // 0x0271 for 48MHz -> 19200 baud
    BAUDCON = 0x08; // BRG16 = 1
    c = RCREG; // read

}//end InitializeUSART

void putcUSART(char c) {//escreve na serial
    TXREG = c;
}

#if defined(USB_CDC_SET_LINE_CODING_HANDLER)
void mySetLineCodingHandler(void)
{    
    if (cdc_notice.GetLineCoding.dwDTERate.Val <= 115200)//velocidade maxima
     {
        //atualiza o baudrate no drive CDC
        CDCSetBaudRate(cdc_notice.GetLineCoding.dwDTERate.Val);
        //atualiza o baudrate da porta serial
#if defined(__18CXX) || defined(__XC8) // sem este "if" o programa não funciona(bug do compilador)
        {
            DWORD_VAL dwBaud;
            dwBaud.Val = (DWORD) (GetSystemClock() / 4) / line_coding.dwDTERate.Val - 1;
            SPBRG = dwBaud.v[0];
            SPBRGH = dwBaud.v[1];
        }
    }
}
#endif

unsigned char getcUSART() { //pega caracteres vindo da serial
    char c;
    if (RCSTAbits.OERR) // no caso de ocorrer um overrun
    {
        RCSTAbits.CREN = 0; // reseta porta
        c = RCREG;
        RCSTAbits.CREN = 1; // volta a funciona.
    } else {
        c = RCREG;
    }
    return c;
}



///funÃ§oes prontas para os eventos da usb, caso queira usar alguma Ã© sÃ³ inserir os codigo-fonte nas chaves
//////////////////////////
void USBCBSuspend(void) {}
void USBCBWakeFromSuspend(void) {}
void USBCB_SOF_Handler(void) {}
void USBCBErrorHandler(void) {}
void USBCBCheckOtherReq(void)
{ 
    USBCheckCDCRequest();
}
void USBCBStdSetDscHandler(void) {}
void USBCBInitEP(void)
{    //habilita os enpoits do modo CDC
    CDCInitEP();
}

void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == TRUE) {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE;
            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}
////////////////////////////////////////////////////////////////////
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)

void USBCBEP0DataReceived(void) {
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //funções de retorno da porta usb
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}

void recebe_usb(void)
{

   
   if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
         {
             //se a usb estiver em modo suspenso não faz nada
             return;
         }else{
            if(mUSBUSARTIsTxTrfReady()) // não tem dados pendentes?
            {

                unsigned char recebido[] = "\n";
                  
                if((getsUSBUSART((char *)recebido, sizeof(recebido))) != 0) // se tem dados na USB para que o PIC leia
                  {

                    if(recebido[0] == 'K'){
                        check = 1;
                    }

                  }
                    
            }
         CDCTxService();
         }

}//fim da função de recebimento, verificação de hand-shake com o computador



int recebe_usb_i2c(void)
{


   if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
         {
             //se a usb estiver em modo suspenso não faz nada
             return 22;
         }else{
            if(mUSBUSARTIsTxTrfReady()) // não tem dados pendentes?
            {
              
                
                if((getsUSBUSART((char *)comando, sizeof(comando))) != 0) // enquanto não têm dados na USB para que o PIC leia, fica preso no while
                  {

                    if(comando[2] == ';'){
                        return 0;
                    }

                  }
            }
             
         CDCTxService();
         }

}//fim da função de recebimento, verificação e tratamento da string recebida





void atraso(void) {

    Delay10KTCYx(1); // atraso de 1ms
}

void delaay(void) {

    Delay10KTCYx(80); // atraso de 80ms


}

void atraso60(void) {

    Delay10KTCYx(60); // atraso de 60ms


}

void atraso70(void) {

    Delay10KTCYx(70); // atraso de 70ms


}






void enviarUSB(char *msg){
    
    // escrita na USB
    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
    {
         //se a usb estiver em modo suspenso não faz nada
         return;
    }else{
        if(mUSBUSARTIsTxTrfReady()) // não tem dados pendentes?
        {

            //putsUSBUSART(msg); // mando os estados das vias via USB para o controlador local
            putUSBUSART((char *) msg, strlen(msg));

        }
     CDCTxService();
    }
     
}





void main(void)
{


    
    unsigned char toSend[] = "x\n";
    
    unsigned char sync_mode=0, slew=0, add1,w,data,status,length;

    unsigned char I2C_Send[6] = "xxxxxx";
    unsigned char I2C_Recv[20];

    

    inicializa_pic();
  //  #if defined(USB_INTERRUPT) // se definido a usb vai usar as interrupções
    USBDeviceAttach(); // para verifica o status e os eventos gerados
 //   #endif
    
    TRISBbits.RB0 = 1;
    TRISBbits.RB1 = 1;

    Delay10TCYx(5);
    


    
    enviarUSB((char *) toSend);


    while (1)
    {
       #if defined(USB_POLLING) // se definido para verificar de tempo em tempo
        USBDeviceTasks();//executa as tarefas da usb(verifica o status e os eventos gerados)
        #endif
            
            if (!check) {

                while(1){

                    toSend[0] = 'K';

                    delaay();

                    enviarUSB((char *) toSend);
                    recebe_usb();
                    USBDeviceTasks();
                    
                    if(check){
                        break;
                    }
                }
                
            }
        
        


        if(check){

                //sprintf(teste, "indice = %i\n", indice);
                //enviarUSB((char *) teste);

            
            
                while(1){ // ****** começou o WHILE *******

                    USBDeviceTasks();

                    

                    if((USBGetDeviceState() < CONFIGURED_STATE) || (USBIsDeviceSuspended() == 1)) {
                        continue;
                    } else{


                            while(recebe_usb_i2c() != 0); // fica aguardando nesta linha até que receba uma msg via serial destinada à ativação do PWM.
                            

                            add1 = 0xA2;		//address of the device (slave) under communication

                            CloseI2C();	//close i2c if was operating earlier

                    //------------------------INITIALISE THE I2C MODULE FOR MASTER MODE WITH 100KHz ---------------------------
                            sync_mode = MASTER;
                            slew = SLEW_OFF;

                            OpenI2C(sync_mode,slew);

                            SSPADD = 49;			 // 100kHz Baud rate of transmission


                            //check for bus idle condition in multi master communication
                            IdleI2C();
                    //--------------------START I2C---------------



                            StartI2C();

                        for(w=0;w<20;w++){

                            I2C_Recv[w]=0;

                        }

                //************** write the address of the device for communication ************

                    data = SSPBUF;		//read any previous stored content in buffer to clear buffer full status

                        do
                        {
                        status = WriteI2C( add1 | 0x00 );	//write the address of slave
                        // retorna 0 se o envio ocorreu com sucesso e -1 se ocorreu uma colisão

                                if(status == -1)		//check if bus collision happened
                                {
                                        data = SSPBUF;		//upon bus collision detection clear the buffer,
                                        SSPCON1bits.WCOL = 0;	// clear the bus collision status bit
                                }
                        }
                        while(status!=0);		//write untill successful communication




                //R/W BIT IS '0' FOR FURTHER WRITE TO SLAVE

                //*********** WRITE THE THE DATA TO BE SENT FOR SLAVE ****************



                    I2C_Send[0] = comando[0];
                    I2C_Send[1] = comando[1];
                    I2C_Send[2] = comando[2];
                    I2C_Send[3] = comando[3];
                    I2C_Send[4] = comando[4];
                    I2C_Send[5] = comando[5];

                    while(putsI2C(I2C_Send) != 0);	//write string of data to be transmitted to slave
                    // problema para enviar mais de uma vez está aqui *****%%%%%%%&&&&&&&&&



                    // ******  Condições para o MASTER *****

                    // retorna 0 se encontrou um caracter nulo na string, ou seja, terminou o envio com sucesso
                    // retorna -2 se o escravo respondeu com um NOTAck
                    // retorna -3 se ocorreu uma colisão de escrita


                //-------------TERMINATE COMMUNICATION FROM MASTER SIDE---------------
                        IdleI2C();
                    // faz um loop (aguarda) até que o barramento esteja livre

                //-----------------RESTART I2C COMMUNICATION---------------------------------------


                        RestartI2C();

                        IdleI2C();
                //************** write the address of the device for communication ************
                    data = SSPBUF;		//read any previous stored content in buffer to clear buffer full status

                //R/W BIT IS '1' FOR READ FROM SLAVE


                    add1 = 0xA2;

                // 0 indica que o MASTER irá fazer ESCRITA e 1 é para o MASTER fazer LEITURA



                    do
                        {
                        status = WriteI2C( add1 | 0x01 );  //write the address of slave
                                if(status == -1)		//check if bus collision happened
                                {
                                        data = SSPBUF;		//upon bus collision detection clear the buffer,
                                        SSPCON1bits.WCOL = 0;	// clear the bus collision status bit
                                }
                        }
                        while(status!=0);			//write untill successful communication

                //******************* Recieve data from slave ******************************

                        while( getsI2C(I2C_Recv, 20) ); //recieve data string of lenght 20 from slave

                        I2C_Recv[18] = '\n' ;
                        I2C_Recv[19] = '\0' ;


                    IdleI2C();

                    NotAckI2C(); //send the end of transmission signal through nack
                    StopI2C();


                    while( SSPCON2bits.ACKEN != 0);		//wait till ack sequence is complete



                    // ACKEN é 1 enquanto houver dados nas linhas SDA e SCL
                    // e 0 quando a sequência é finalizada.
                    // ACKEN é limpa por hardware automaticamente.

                    USBDeviceTasks();
                    enviarUSB((char *) I2C_Recv); // envio o que foi recebido para a USB
                    USBDeviceTasks();

                //********************* close I2C *****************************************

                        RestartI2C();
                        IdleI2C();

                        CloseI2C();

                        
                
                    } 
                        

                } // ******* fim do WHILE

        //CloseI2C();								//close I2C module

        //while(1); // fim do programa

        } // fim do check que verifica se o Java já está pronto para receber informações do PIC

        
    }//fim do while
    
}//fim da main
