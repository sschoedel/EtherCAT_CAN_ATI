#include "UART_TIVA.h"

/**
 * Initializes UART0 for communicating between Tiva and Master
 */
void UART0Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //also for uart0 and SSI
    // Set PA0 and PA1 as UART pins.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 1,152,000, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 1152000,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

/**
 * Initializes UART0 for communicating between Tiva and motor controller
 */
void UART1Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Set PB0 and PB1 as UART pins.
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

/**
 * Sends instruction to users to the console
 */
void sendInstruction()
{
    UARTSendString(0, "TIVA initialized, begin logging: \r\n");
}

/*
 * Send a string to the UART.
 * uartMod is used to indicate which UART module you want to use
 * UART0 is used between MCU and the Computer console
 * UART1 is used between MCU and the motor controller
 */
void UARTSendString(uint8_t uartMod, char pui8Buffer[])
{
    //
    // Loop while there are more characters to send.
    //
    uint8_t i;
    for (i = 0; i < strlen(pui8Buffer); i++)
    {
        //
        // Write the next character to the UART.
        //
        if (uartMod == 0)
        {
            //non blocking function were causing problem
//            UARTCharPutNonBlocking(UART0_BASE, pui8Buffer[i]);//uart0 module
            UARTCharPut(UART0_BASE, pui8Buffer[i]);        //uart0 module
        }
        else if (uartMod == 1)
        { //Unnecessary for Thor Actuator applications
            //UARTPutNonBlockingFlag will only become 0 if there is no space available in transmit FIFO
            //UARTPutNonBlockingFlag = UARTCharPutNonBlocking(UART1_BASE, pui8Buffer[i]);//uart1 module
            UARTCharPut(UART1_BASE, pui8Buffer[i]);
        }
    }
}

void checkKeyboard()
{
    // Read the next character from the UART and write it back to the UART.
    uint8_t readdata = UARTCharGetNonBlocking(UART0_BASE);
    UARTprintf("val received: %d\n", readdata);



    // Read the next character from the UART and write it back to the UART.
    readdata = UARTCharGetNonBlocking(UART0_BASE);
    // op_state = testing; //reinitialize op_state
//
//    //change joint limit
//    if (readdata == 'j')
//    {
//        //reset lower and upper limit
//        memset(lower_joint_limit_str, 0, 15);
//        memset(upper_joint_limit_str, 0, 15);
//        op_state = changing_lower_joint_limit;
//        UARTSendString(0, "lo_jt_limit= ");
//    }
}

