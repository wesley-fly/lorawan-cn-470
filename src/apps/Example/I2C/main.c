/*
    (C)2017 Gemtek

Description: This example describes how to configure and use I2C.
This sample code shows how to read/write I2C to SI7020_A20 sensor.
The SI7020_A20 is i2c interface humidity and temperature sensor.

*/
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board-config.h"
#include "utilities.h"
#include "i2c.h"
#include "delay.h"

#define I2C_ADDRESS                             0x40 <<1  //SI7020_A20 I2C address is 0x40(64) and 7-bit base slave address

#define HUMIDITY_ADDRESS                        0xE5      //humidity measurement command 0xF5
#define TEMPERATURE_ADDRESS                     0xE3      //temperature measurement command 0xF3

#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512

uint8_t Uart1TxBuffer[FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[FIFO_RX_SIZE];

char str[20];

uint8_t data[2] = {0};
uint8_t txData = 0;

static I2c_t I2c;

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
    UartConfig( &Uart1, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    
    I2cInit( &I2c, I2C_1, I2C_SCL, I2C_SDA );

    while(1)
    {			
        if(I2cWrite(&I2c, I2C_ADDRESS, HUMIDITY_ADDRESS,txData)!= SUCCESS)
        {
            sprintf(str, "I2cWrite Fail!\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
        }
            
        DelayMs(1000);
            
        // Read 2 bytes of humidity data
        // humidity msb, humidity lsb
        if(I2cRead(&I2c, I2C_ADDRESS, HUMIDITY_ADDRESS,(uint8_t *)data)!= SUCCESS)
        {
            sprintf(str, "I2cRead Fail!\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
        }
            
        // Convert the data
        float humidity = (((data[0] * 256 + data[1]) * 125.0) / 65536.0) - 6;
        
        // Output data to screen
        sprintf(str, "Relative Humidity : %.2f RH \r\n",humidity);
        UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
            
        // Send temperature measurement command, NO HOLD MASTER(0xF3)
        if(I2cWrite(&I2c, I2C_ADDRESS, TEMPERATURE_ADDRESS, txData)!= SUCCESS)
        {
            sprintf(str, "I2cWrite Fail!\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
        }
            
        DelayMs(1000);

        // Read 2 bytes of temperature data
        // temp msb, temp lsb
        if(I2cRead(&I2c, I2C_ADDRESS, TEMPERATURE_ADDRESS,(uint8_t *)data)!= SUCCESS)
        {
            sprintf(str, "I2cRead Fail!\r\n");
            UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
        }
            
        // Convert the data
        float cTemp = (((data[0] * 256 + data[1]) * 175.72) / 65536.0) - 46.85;
        float fTemp = cTemp * 1.8 + 32;
        
        // Output data to screen
        sprintf(str, "Temperature in Celsius : %.2f C \r\n", cTemp);
        UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
        sprintf(str, "Temperature in Fahrenheit : %.2f F \r\n\r\n", fTemp);
        UartPutBuffer(&Uart1, (uint8_t *)str, strlen(str));
        
        DelayMs(1000);
    }
}

