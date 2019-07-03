/*
    (C)2017 Gemtek

Description: This example describes how to use Tx Continuous mode.

*/
#include <stdio.h> 
#include <string.h>

#include "board.h"
#include "board-config.h"
#include "radio.h"

#define REGION_AS923 1

#if defined( REGION_AS923 )

#define RF_FREQUENCY                                923000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_AU915 )

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_CN470 )

#define RF_FREQUENCY                                470000000 // Hz
#define TX_OUTPUT_POWER                             20        // 20 dBm

#elif defined( REGION_CN779 )

#define RF_FREQUENCY                                779000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_EU433 )

#define RF_FREQUENCY                                433000000 // Hz
#define TX_OUTPUT_POWER                             20        // 20 dBm

#elif defined( REGION_EU868 )

#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_KR920 )

#define RF_FREQUENCY                                920000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_IN865 )

#define RF_FREQUENCY                                865000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_US915 )

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_US915_HYBRID )

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#else

    #error "Please define a frequency band in the compiler options."

#endif
#define TX_TIMEOUT                                  65535     // seconds (MAX value)

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnRadioTxTimeout( void )
{
    // Restarts continuous wave transmission when timeout expires
    Radio.SetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
}

#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512
char str[128];
uint8_t UartTxBuffer[FIFO_TX_SIZE];
uint8_t UartRxBuffer[FIFO_RX_SIZE];

/**
 * Main application entry point.
 */
int main( void )
{
    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );
    
    /* Initial FIFOs for UART1 transmit/receive respectively */
    FifoInit( &Uart1.FifoTx, UartTxBuffer, FIFO_TX_SIZE );
    FifoInit( &Uart1.FifoRx, UartRxBuffer, FIFO_RX_SIZE );

    /* Initializes the UART object and MCU peripheral */
    UartInit( &Uart1, UART_1, UART_TX, UART_RX );

    /* Configures the UART object and MCU peripheral */
    UartConfig( &Uart1, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );    

    // Radio initialization
    RadioEvents.TxTimeout = OnRadioTxTimeout;
    Radio.Init( &RadioEvents );

    Radio.SetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );

    sprintf(str, "Starting TX Continue Mode Freq:%d TxPower:%d\r\n",RF_FREQUENCY,TX_OUTPUT_POWER);
    UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));	
    // Blink LEDs just to show some activity
    while( 1 )
    {
        
    }
}
