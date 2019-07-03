/*
    (C)2017 Gemtek

Description: This example describes how to use Timer.

*/
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board-config.h"
#include "timer.h"

#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512

uint8_t Uart1TxBuffer[FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[FIFO_RX_SIZE];
char str[20];
static TimerEvent_t Task1Timer;
volatile bool  Task1TimerEvent = false;

void  OnTask1TimerEvent( void )
{
     Task1TimerEvent = true;
}

static TimerEvent_t  Task2Timer;
volatile bool Task2TimerEvent = false;

void OnTask2TimerEvent( void )
{
    Task2TimerEvent = true;
}

/**
 * Main function
 */
int main( void )
{
    BoardInitMcu( );
	
    /* Initial FIFOs for UART1 transmit/receive respectively. */
    FifoInit( &Uart1.FifoTx, Uart1TxBuffer, FIFO_TX_SIZE );
    FifoInit( &Uart1.FifoRx, Uart1RxBuffer, FIFO_RX_SIZE );

    /* Initializes the UART object and MCU peripheral */
    UartInit( &Uart1, UART_1, UART_TX, UART_RX );

    /* Configures the UART object and MCU peripheral */
    UartConfig( &Uart1, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
	
    /* Initial 2 Timers for Print Task1 and Task2 */
    TimerInit( &Task1Timer, OnTask1TimerEvent );
    TimerInit( &Task2Timer, OnTask2TimerEvent );

    /* Print Task1 every 5000 ms */
    TimerSetValue( &Task1Timer, 5000 );

    /* Print Task2 500 ms after Task1 */
    TimerSetValue( &Task2Timer, 500 );

    /* Start Timer(On) */
    TimerStart( &Task1Timer );

    /* The UART Screen will print Task1 every 5000 ms and print Task2 after 500 ms */
    while( 1 )
    {
        if( Task1TimerEvent )
        {
            Task1TimerEvent = false;
            sprintf(str, "Task1 Timer Event!\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
            TimerStart( &Task2Timer );
        }

        if( Task2TimerEvent )
        {
            Task2TimerEvent = false;
            sprintf(str, "Task2 Timer Event!\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
            TimerStart( &Task1Timer );
        }
    }
}
