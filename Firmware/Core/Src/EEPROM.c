/*
 * EEPROM.c
 *
 *  Created on: Jul 4, 2023
 *      Author: ubuntu
 */

#include "EEPROM.h"
#include "m95p32.h"


uint16_t get_eeprom_data_len()
{
	uint8_t		status;
	uint16_t	data_len = 0;
	uint16_t 	ff_ctr = 0;
	uint16_t	empty_ctr = 0;

	for(uint16_t ii=0;ii<eeprom_size;ii++)
	{
		Single_Read(&status, ii*M95P32_PAGESIZE ,1);
		//kappa("\r\n [%d]: %x", ii, status);
		if(status!=0xFF)
		{
			data_len++;
			ff_ctr=0;
		}
		else if(data_len>0)	//If some data is already found, start counting!
		{
			ff_ctr++;
			if(ff_ctr>3)
				break;
		}else
		{
			empty_ctr++;
			if(empty_ctr>100)	//TODO: Improve this condition (Rd index could be stored on non-volatile memory)
				break;
		}
	}
	//kappa("\r\nData_len:%d", data_len);
	return data_len;
}

uint16_t get_eeprom_read_idx()
{
	uint8_t		status = 0;
	for(uint16_t ii=0; ii<eeprom_size; ii++)
	{
		Single_Read(&status, ii*M95P32_PAGESIZE ,1);
		if(status!=0xFF)
			return ii;
	}
	return 0; //Empty
}
