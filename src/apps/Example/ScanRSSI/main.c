/*
    (C)2017 Gemtek

Description: This example describes how to scan RSSI.

*/
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board-config.h"
#include "sx1276-board.h"
#include "LoRaMac.h"
#include "sx1276-board.h"

static TimerEvent_t ReadRssiTimer;
volatile bool ReadRssiFlag = false;

#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512

uint8_t UartTxBuffer[FIFO_TX_SIZE];
uint8_t UartRxBuffer[FIFO_RX_SIZE];

void UartIrqNotify( UartNotifyId_t id );

static void McpsConfirm( McpsConfirm_t *mcpsConfirm ){}
static void McpsIndication( McpsIndication_t *mcpsIndication ){}
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm ){}

uint8_t RssiMsg[FIFO_TX_SIZE];

void OnReadRssiTimerEvent( void )
{
    ReadRssiFlag = true;
}


/**
 * Main function
 */
int main( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;

    uint16_t i;
    int16_t rssi;
    uint32_t frequency;
    uint8_t tmpStr[16] = "";

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

    /*Initial Radio Configurations, Read RSSI value must initial radio configuration first*/
    LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
    LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
    LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
    LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923);

    /* Initial Timers for reading rssi value every 5 seconds*/
    TimerInit( &ReadRssiTimer, OnReadRssiTimerEvent );
    TimerSetValue( &ReadRssiTimer, 5000 );
    TimerStart( &ReadRssiTimer );

    /* Infinite loop */
    while( 1 )
    {
        if(ReadRssiFlag == true)
        {
            ReadRssiFlag = false;

            memset(RssiMsg,0,FIFO_TX_SIZE);
            strcat((char*)RssiMsg,"\r\nFrequency: Rssi\r\n");

            for( i = 0; i < 8; i++ )
            {
                frequency = 922.625e6 + i * 250e3;
                rssi = SX1276ReadRssiFsk(frequency);
                sprintf((char*)tmpStr, "%d: %d\r\n", frequency,rssi);
                strcat((char*)RssiMsg, (char*)tmpStr);
            }

            UartPutBuffer(&Uart1, RssiMsg, strlen((const char *)RssiMsg));

            TimerStart( &ReadRssiTimer );
        }
    }

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
    {
    }
}

