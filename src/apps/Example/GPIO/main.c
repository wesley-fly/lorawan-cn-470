/*
    (C)2017 Gemtek

Description: This example describes how to configure and use GPIOs.
This sample code shows how to 
1. using ADC to get pin voltage.
2. using GPIO set interrupt and callback
*/
#include <string.h>

#include "board.h"
#include "adc-board.h"
#include "board-config.h"
#include "fifo.h"
#include "uart.h"
#include "stm32l0xx_hal.h"
#include "delay.h"

#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512

uint8_t Uart1TxBuffer[FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[FIFO_RX_SIZE];

char sendBuff[80];

float GetPinVoltage( uint32_t channel );
void GPIOCallback(void);

extern Adc_t Adc;
/**
 * Main function
 */
int main( void )
{
    
    Gpio_t irq;
    
    BoardInitMcu( );
    /* Initial FIFOs for UART1 transmit/receive respectively. */
    FifoInit( &Uart1.FifoTx, Uart1TxBuffer, FIFO_TX_SIZE );
    FifoInit( &Uart1.FifoRx, Uart1RxBuffer, FIFO_RX_SIZE );

    /* Initializes the UART object and MCU peripheral */
    UartInit( &Uart1, UART_1, UART_TX, UART_RX );

    /* Configures the UART object and MCU peripheral */
    UartConfig( &Uart1, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

    //GPIOCallback will trigger by I2C_SCL(PB6)   
    GpioInit(&irq, I2C_SCL, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
    GpioSetInterrupt(&irq, IRQ_RISING_EDGE, IRQ_MEDIUM_PRIORITY, GPIOCallback);
    
    while( 1 )
    {
        sprintf(sendBuff, "ADC TEST PinVoltage of PB0 = %.2f\r\n",GetPinVoltage(ADC_CHANNEL_8));
		UartPutBuffer(&Uart1, (uint8_t *)sendBuff, strlen(sendBuff));
        
        /* Delay 5s */
        DelayMs(5000);        
    }
}

float GetPinVoltage(uint32_t channel)
{   
    float pmSensor_V_bat;
    uint16_t vdiv = 0, vref = 0;
    
    AdcInit(&Adc,BAT_LEVEL_PIN);
    vdiv = AdcReadChannel(&Adc,channel);
    vref = AdcReadChannel(&Adc,ADC_CHANNEL_17);
    pmSensor_V_bat = 2.1065*vdiv/vref;
    return pmSensor_V_bat*10;
}

void GPIOCallback(void)
{
    sprintf(sendBuff, "GPIOCallback TEST!\r\n");
    UartPutBuffer(&Uart1, (uint8_t *)sendBuff, strlen(sendBuff));
}
