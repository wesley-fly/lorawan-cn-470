/*
    (C)2017 Gemtek

Description: This example describes how to configure and use Ping Pong Test.
1.Config Master or Slave mode by isMaster = true or false;
2.Config RF_FREQUENCY,TX_OUTPUT_POWER,LORA_BANDWIDTH,LORA_SPREADING_FACTOR ... etc.
3.Config TX_PACKET_NUMBER for how many packets for test.
4.Config TOTAL_RX_TIME for how much time for test.
5.The Slave Side will leaving test mode when receive PONG packet or reach the TOTAL_RX_TIME.


*/
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board-config.h"
#include "radio.h"
#include "timer.h"

#define USE_MODEM_LORA

#define RF_FREQUENCY                                470000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here
#define TX_PACKET_NUMBER                            10
#define TOTAL_RX_TIME                               10000 //10s			

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

#define FIFO_TX_SIZE                            128
#define FIFO_RX_SIZE                            128

uint8_t Uart1TxBuffer[FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[FIFO_RX_SIZE];

uint16_t RadioBufferSize = BUFFER_SIZE;
uint8_t RadioBuffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;
char str[20];
int TxCounter=1;
int RxCounter=0;

static TimerEvent_t ReceiveTimer;
static TimerTime_t RxDoneTime;
void OnReceiveTimerEvent( void );
uint8_t StopFlag=0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/**
 * Main application entry point.
 */
int main( void )
{
	/* true = Master, false = Slave */
    bool isMaster = false;
    uint8_t i;

    // Target board initialization
    BoardInitMcu( );
    
    /* Initial FIFOs for UART1 transmit/receive respectively. */
    FifoInit( &Uart1.FifoTx, Uart1TxBuffer, FIFO_TX_SIZE );
    FifoInit( &Uart1.FifoRx, Uart1RxBuffer, FIFO_RX_SIZE );

    /* Initializes the UART object and MCU peripheral */
    UartInit( &Uart1, UART_1, UART_TX, UART_RX );

    /* Configures the UART object and MCU peripheral */
    UartConfig( &Uart1, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    
    /* Initial Timer for Receive */
    TimerInit( &ReceiveTimer, OnReceiveTimerEvent );
    
    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif
                                  
    if( isMaster == true )
    {
        if(TxCounter==1)
        {
            sprintf(str, "Starting Master Ping-Pong Test\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));	
        }
        State = TX;
    }
    else
    {
        if(RxCounter==0)
        {
            sprintf(str, "Starting Slave Ping-Pong Test\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));	
            /* ReceiveTimer in ms */
            TimerSetValue( &ReceiveTimer, TOTAL_RX_TIME );
            /* Start Timer */
            TimerStart( &ReceiveTimer );
            /* Get Current Time */
            TimerTime_t curTime = TimerGetCurrentTime( );
            RxDoneTime = curTime;
        }	
        Radio.Rx( RX_TIMEOUT_VALUE );
    }
    
    while( 1 )
    {
		/* Leaving Ping Pong Test */
		if(StopFlag)
		{
            sprintf(str, "leaving Ping Pong mode\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));	
            while(1);
        }
        
        switch( State )
        {
        case RX:
            if( isMaster == false )
            {		
                if( RadioBufferSize > 0 )
                {
                    if( strncmp( ( const char* )RadioBuffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        //Indicates on that the received frame is a PONG, means the end of transmission
                        RxCounter++;
                        /* Get Rx done Time */
                        sprintf(str, "%d packets received in %d seconds\r\n",RxCounter,TimerGetElapsedTime( RxDoneTime )/1000);
                        UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));	
                        TimerStop( &ReceiveTimer );
                        StopFlag=1;
                    }
                    if( strncmp( ( const char* )RadioBuffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        // Indicates on that the received frame is a PING										
                        RxCounter++;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            // Indicates on that we have sent a PING
            // Indicates on that we have sent a PONG, means the end of transmission
            //There will have TX_PACKET_NUMBER-1 PING Packet and 1 PONG Packet,PONG packet means the end of transmission
            if( (isMaster == true) && (TxCounter <= TX_PACKET_NUMBER) )
            {
                // Send the End PONG frame
                if(TxCounter == TX_PACKET_NUMBER)
                {
                    RadioBuffer[0] = 'P';
                    RadioBuffer[1] = 'O';
                    RadioBuffer[2] = 'N';
                    RadioBuffer[3] = 'G';
                }
                else
                {
                    RadioBuffer[0] = 'P';
                    RadioBuffer[1] = 'I';
                    RadioBuffer[2] = 'N';
                    RadioBuffer[3] = 'G';
                }

                for( i = 4; i < RadioBufferSize; i++ )
                {
                    RadioBuffer[i] = i - 4;
                }
                Radio.Send( RadioBuffer, RadioBufferSize );
                TxCounter++;
            }
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            if( isMaster == false )
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }
        //TimerLowPowerHandler( );
    }	
}

void OnTxDone( void )
{
    Radio.Sleep( );
    if(TxCounter <= TX_PACKET_NUMBER)
    {
        State = TX;
    }
   
    if(TxCounter == TX_PACKET_NUMBER)
    {
        sprintf(str, "%d Packets already transmitted\r\n",TX_PACKET_NUMBER);
        UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));	
    }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    RadioBufferSize = size;
    memcpy( RadioBuffer, payload, RadioBufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
}

void OnReceiveTimerEvent( )
{
    sprintf(str, "%d packets received in %d seconds\r\n",RxCounter,TOTAL_RX_TIME/1000);
    UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
    StopFlag=1;
}
