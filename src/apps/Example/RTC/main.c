/*
    (C)2017 Gemtek

Description: This example describes how to use RTC alarm.

*/
#include <string.h>

#include "stm32l0xx.h"
#include "board.h"
#include "board-config.h"
#include "eeprom.h"
#include "fifo.h"
#include "uart.h"
#include "rtc-board.h"


#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512

uint8_t Uart1TxBuffer[FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[FIFO_RX_SIZE];

bool isSetAlarm = true;

char str[20];
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
	
    RtcSetTimeout( 5000 );

    /* Toggle LED(PC1) in an infinite loop */
    while( 1 )
    {
        if( !isSetAlarm )
        {
            isSetAlarm = true;

            /* Set RTC alarm every 5000 ms */
            RtcSetTimeout( 5000 );
        }
    }
}

/* Declare RTC alarm callback function for toggle LED(PC1) */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    //Output read data to screen
    sprintf(str, "RTC_Alarm Event\r\n");
    UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
    isSetAlarm = false;
}
