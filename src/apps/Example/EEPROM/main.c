/*
    (C)2017 Gemtek

Description: This example describes how to configure and use EEPROM Read/Write.
This sample code shows how to Write 2 bytes to EEPROM start address 0x08080C00, and read 2 bytes form EEPROM.

*/
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board-config.h"
#include "eeprom.h"
#include "fifo.h"
#include "uart.h"

#define EEPROM_CONFIG_START_ADDR                0x08080C00
#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512

uint8_t Uart1TxBuffer[FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[FIFO_RX_SIZE]; 

/**
 * Main function
 */
int main( void )
{   
    uint8_t Readdata[2];
    uint8_t WriteData[2];
    char str[20];
    BoardInitMcu( );
    WriteData[0]=0x94;
    WriteData[1]=0x53;

    /* Initial FIFOs for UART1 transmit/receive respectively. */
    FifoInit( &Uart1.FifoTx, Uart1TxBuffer, FIFO_TX_SIZE );
    FifoInit( &Uart1.FifoRx, Uart1RxBuffer, FIFO_RX_SIZE ); 

    /* Initializes the UART object and MCU peripheral */
    UartInit( &Uart1, UART_1, UART_TX, UART_RX );

    /* Configures the UART object and MCU peripheral */
    UartConfig( &Uart1, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    //EEPROM Write 2 bytes
    EepromWriteBuffer( EEPROM_CONFIG_START_ADDR, WriteData, 2 );
    //EEPROM Read 2 bytes
    EepromReadBuffer(EEPROM_CONFIG_START_ADDR, Readdata, 2);
    //Output read data to screen
    sprintf(str, "EepromRead: %x,%x\r\n",Readdata[0],Readdata[1]);
    UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
    while( 1 );
}
