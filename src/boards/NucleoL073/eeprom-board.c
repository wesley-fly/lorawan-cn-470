/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "stm32l0xx.h"
#include "utilities.h"
#include "eeprom-board.h"

uint8_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t status = FAIL;

    assert_param( ( DATA_EEPROM_BASE + addr ) >= DATA_EEPROM_BASE );
    assert_param( buffer != NULL );
    assert_param( size < ( DATA_EEPROM_END - DATA_EEPROM_BASE ) );

    if( HAL_FLASHEx_DATAEEPROM_Unlock( ) == HAL_OK )
    {
        for( uint16_t i = 0; i < size; i++ )
        {
            if( HAL_FLASHEx_DATAEEPROM_Program( FLASH_TYPEPROGRAMDATA_BYTE,
                                                ( DATA_EEPROM_BASE + addr + i ),
                                                  buffer[i] ) != HAL_OK )
            {
                // Failed to write EEPROM
                break;
            }
        }
        status = SUCCESS;
    }

    HAL_FLASHEx_DATAEEPROM_Lock( );
    return status;
}

uint8_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    assert_param( ( DATA_EEPROM_BASE + addr ) >= DATA_EEPROM_BASE );
    assert_param( buffer != NULL );
    assert_param( size < ( DATA_EEPROM_END - DATA_EEPROM_BASE ) );

    memcpy1( buffer, ( uint8_t* )( DATA_EEPROM_BASE + addr ), size );
    return SUCCESS;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
    assert_param( FAIL );
}

uint8_t EepromMcuGetDeviceAddr( void )
{
    assert_param( FAIL );
    return 0;
}

void InternalEepromReadBytes(uint32_t EepromAddr, uint8_t *Buffer, uint16_t Length)  
{  
    uint8_t *wAddr;  
    wAddr=(uint8_t *)(EepromAddr);  
    while(Length--){  
        *Buffer++=*wAddr++;  
    }     
}  

void InternalEepromReadWords(uint32_t EepromAddr, uint16_t *Buffer, uint16_t Length)  
{  
    uint32_t *wAddr;  
    wAddr=(uint32_t *)(EepromAddr);  
    while(Length--){  
        *Buffer++=*wAddr++;  
    }     
}

uint8_t InternalEepromWriteBytes( uint32_t EepromAddrStart, uint8_t *Buffer, uint16_t BufferSize )
{
		uint16_t AddrIdx = 0;
		HAL_StatusTypeDef Write_Result;
				
		__HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR |FLASH_FLAG_SIZERR );
		
    HAL_FLASHEx_DATAEEPROM_Unlock( );
    HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram( );
	
		for( AddrIdx = 0 ; AddrIdx < BufferSize ; AddrIdx++ ){
      
//        Write_Result = HAL_FLASHEx_DATAEEPROM_Program( FLASH_TYPEPROGRAMDATA_FASTBYTE, EepromAddrStart+AddrIdx , Buffer[AddrIdx] );

        Write_Result = HAL_FLASHEx_DATAEEPROM_Program( FLASH_TYPEPROGRAMDATA_BYTE, EepromAddrStart+AddrIdx , Buffer[AddrIdx] );
      
				if( Write_Result != HAL_OK )
						break;
		}
		
    HAL_FLASHEx_DATAEEPROM_Lock( );
    HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram( );
		
        if( Write_Result == HAL_OK )
            return SUCCESS;
        return FAIL;
}

uint8_t InternalEepromWriteWords( uint32_t EepromAddrStart, uint8_t *Buffer, uint16_t BufferSize )
{   
    HAL_StatusTypeDef Write_Result;
    uint16_t ByteToWordSize = ( BufferSize / sizeof( uint32_t ) );
    
		__HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR |FLASH_FLAG_SIZERR );
		
    HAL_FLASHEx_DATAEEPROM_Unlock( );
    HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram( );
    
    for( uint16_t AddrIdx = 0 ; AddrIdx < ByteToWordSize ; AddrIdx++ )
    {
        uint16_t AddrOffset = AddrIdx * sizeof( uint32_t );
        uint32_t WriteData = 0;
      
        for( int8_t idx = sizeof( uint32_t ) - 1; idx >= 0 ; idx-- )
        {
            WriteData <<= 8;
            WriteData |= Buffer[ AddrOffset + idx ];
        }
        
        Write_Result = HAL_FLASHEx_DATAEEPROM_Program( FLASH_TYPEPROGRAMDATA_WORD, 
                                                       EepromAddrStart + AddrOffset,
                                                       WriteData );
      
				if( Write_Result != HAL_OK )
						break;
		}
		
    HAL_FLASHEx_DATAEEPROM_Lock( );
    HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram( );
		
        if( Write_Result == HAL_OK )
            return SUCCESS;
        return FAIL;
}

