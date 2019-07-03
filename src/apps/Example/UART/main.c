/*
    (C)2017 Gemtek

Description: This example describes how to configure and use UART.

*/
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board-config.h"
#include "delay.h"

#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512
char str[50];

uint8_t UartTxBuffer[FIFO_TX_SIZE];
uint8_t UartRxBuffer[FIFO_RX_SIZE];

void UartIrqNotify( UartNotifyId_t id );

/**
 * Main function
 */
int main( void )
{
    BoardInitMcu( );

    /* Initial FIFOs for UART1 transmit/receive respectively */
    FifoInit( &Uart1.FifoTx, UartTxBuffer, FIFO_TX_SIZE );
    FifoInit( &Uart1.FifoRx, UartRxBuffer, FIFO_RX_SIZE );

    /* Set UART1 notification callback */
    Uart1.IrqNotify = UartIrqNotify;

    /* Initializes the UART object and MCU peripheral */
    UartInit( &Uart1, UART_1, UART_TX, UART_RX );

    /* Configures the UART object and MCU peripheral */
    UartConfig( &Uart1, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    
    /* Infinite loop */
    sprintf(str, "Starting UART Test\r\n");
    UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));	
    DelayMs(1000);
    while( 1 );
}

void UartIrqNotify( UartNotifyId_t id )
{
    uint8_t data;

    /* Transmit the received data when RX interrupt occurs */
    if( id == UART_NOTIFY_RX )
    {        
        if( UartGetChar( &Uart1, &data ) == 0 )
        {
            UartPutChar( &Uart1, data );
        }
    }
    if( id == UART_NOTIFY_TX )
    {}
}
