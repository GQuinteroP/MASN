/*
 * SIMCOM7022.c
 *
 *  Created on: Jun 2, 2023
 *      Author: ubuntu
 */

#include "SIMCOM7022.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "arm_math.h"

struct LWPA_def LPWA = {.S7022_COAPOPEN=0,.S7022_COAPSTART=0,.operator=0, .status=0, .coap_id=0,.errors={.S7022_AT=0,
						.S7022_CEREG=0, .S7022_COAPSTART=0, .S7022_COAPOPEN=0, .S7022_COAPHEAD=0,
						.S7022_COAPOPTION=0, .S7022_COAPSENDTX=0}};

const uint8_t	method[5] 	=	"put";
const uint8_t	coap_path[9] = "TEST";	//The token of CoAP message
const uint8_t	operator_list[3][6] = {"21403", "21401", "21407"};

//const uint8_t	coap_msgid = 0;	//The CoAP message ID,the range is 0 to 65535
const uint8_t	coap_tkl = 1;	//The length of token,the range is 0 to 8
//const uint8_t	coap_tk[9] = "1";	//The token of CoAP message

const uint8_t	cmd_AT[3] =	"AT\r";
const uint8_t	cmd_CSQ[7] =	"AT+CSQ\r";
const uint8_t	cmd_PIN[9] =	"AT+CPIN?\r";
const uint8_t	cmd_CGATT[10] =	"AT+CGATT?\r";
const uint8_t	cmd_CEREG[10] =	"AT+CEREG?\r";
const uint8_t	cmd_CGSN[10] =	"AT+CGSN=1\r";
uint8_t	cmd_QCPMUCFG[16] =	"AT+QCPMUCFG=1,4\r";

const uint8_t	cmd_QCDNS[18] = "AT+QCDNS=\"coap.me\"";	//TODO: URL must be changed to server if this command is used (dynamic DNS)

const uint8_t	cmd_COAPSTART[13]	=	"AT+COAPSTART\r";
const uint8_t	cmd_COAPSTOP[13]	=	"AT+COAPSTOP\r";

const char *resp_error = "ERROR";
const char *resp_ok = "OK";
const char *resp_dummy = "dummy";

uint8_t buffer_lpwa_resp[3][100];	//TODO: Remove this

extern struct BUFF_G
{
	uint16_t bufferLength;	//Actual buffer length
	uint16_t writeIndex;	 //	Increase writeIndex position to prepare for next write
	uint16_t readIndex;
	uint8_t	ncols;
	uint8_t nrows;
	uint8_t  **bufferMain;
} BUFF_LPWA;

struct BUFF_RESP
{
	uint16_t bufferLength;	//Actual buffer length
	uint16_t writeIndex;	 //	Increase writeIndex position to prepare for next write
	uint16_t readIndex;
	float32_t code[max_blocks_LPWA]; //The received code of the message
	uint8_t id[max_blocks_LPWA];	//The received id of the message
};
struct BUFF_RESP LPWA_confirm_msgs;

volatile uint64_t received_bitmask = 0;
volatile uint8_t bitmask_ready = 0;

uint8_t S7022_rx_put(struct BUFF_RESP *queue, float32_t code_in, uint8_t id_in)
{
	if (queue->bufferLength == max_blocks_LPWA)
	{
		#ifdef DEBUG
			debug("\r\n*[S7022_rx_put] Buffer is full!");
		#endif
		return osError;
	}
	queue->code[queue->writeIndex]=code_in;
	queue->id[queue->writeIndex]=id_in;

	queue->bufferLength++;	 //	Increase buffer size after writing

	#ifdef DEBUG
		//debug("\r\n*[S7022_rx_put] bufferLength[%d] code:%d - id:%d", (uint8_t) queue->code[queue->writeIndex], queue->id[queue->writeIndex]);
	#endif

	queue->writeIndex++;	 //	Increase writeIndex position to prepare for next write

	// If at last index in buffer, set writeIndex back to 0
	if (queue->writeIndex == max_blocks_LPWA)
		queue->writeIndex = 0;

	return osOK;
}

uint8_t S7022_rx_get(struct BUFF_RESP *queue, float32_t *code_out, uint8_t *id_out)
{
	if (queue->bufferLength == 0)
	{
		#ifdef DEBUG
			debug("\r\n[S7022_rx_get]Buffer is empty!");
		#endif
		return osError;
	}
	*code_out = queue->code[queue->readIndex];
	*id_out = queue->id[queue->readIndex];

	#ifdef DEBUG
		debug("\r\n*[S7022_rx_get] bufferLength[%d] code:%d - id:%d", queue->bufferLength,  (uint8_t) queue->code[queue->readIndex], queue->id[queue->readIndex]);
	#endif

	queue->code[queue->readIndex] = 0.0;
	queue->id[queue->readIndex] = 0;

	queue->bufferLength--;	 //	Decrease buffer size after reading
	queue->readIndex++;	 //	Increase readIndex position to prepare for next read

	// If at last index in buffer, set readIndex back to 0
	if (queue->readIndex == max_blocks_LPWA)
		queue->readIndex = 0;

	return osOK;
}

//TODO: Better LPWA_rcv_msgs as argument or not?
uint8_t S7022_PROCESS_Rx(struct BUFF_G *LPWA_rcv_msgs, uint8_t r_type, const char *expected_resp)
{
	uint8_t ret = 0xFF;
	uint8_t buff_len = LPWA_rcv_msgs->bufferLength;
	uint8_t array[LPWA_rx_len];
	char *tmp = (char *) array;

	if(buff_len>0)
	{
		switch(r_type)
		{
			case 0:	//Simple OK (NO CoAP message sent)
				while(MAIN_Queue_get(LPWA_rcv_msgs, (uint8_t *) &tmp[0]) == osOK)
				{
					char *resp_i= strstr((char*) tmp, resp_ok);

					if(resp_i)
					{
						#ifdef debug_LPWA
							debug("\r\n[S7022_PROCESS_Rx][%d] %s", r_type, resp_ok);
						#endif
						return 0;
					}
					else
					{
						char *resp_i= strstr((char*) tmp, resp_error);
						if(resp_i)
						{
							#ifdef debug_LPWA
								debug("\r\n[S7022_PROCESS_Rx][%d] ERROR! %s", r_type, tmp);
							#endif
							return 1;
						}else
						{
							#ifdef debug_LPWA
								debug("\r\n[S7022_PROCESS_Rx][%d] NOT IN", r_type, tmp);
							#endif
							return 2;
						}
					}
				}
			break;

			case 1:	//Custom response (NO CoAP message sent)
				while(MAIN_Queue_get(LPWA_rcv_msgs, (uint8_t *) &tmp[0]) == osOK)
				{
					char *resp_i= strstr((char*) tmp, expected_resp);

					if(resp_i)
					{
						#ifdef debug_LPWA
							debug("\r\n[S7022_PROCESS_Rx][%d] %s", r_type, expected_resp);
						#endif
						return 0;
					}
					else
					{
						char *resp_i= strstr((char*) tmp, resp_error);
						if(resp_i)
						{
							#ifdef debug_LPWA
								debug("\r\n[S7022_PROCESS_Rx][%d] ERROR! %s", r_type, tmp);
							#endif
							ret = 1;
						}else
						{
							#ifdef debug_LPWA
								debug("\r\n[S7022_PROCESS_Rx][%d] NOT IN", r_type);
							#endif
							ret = 2;
						}
					}
				}
			break;

			case 2:	//Custom response (AWAITING for CoAP message response)
				uint8_t counter = 0;	//To avoid infinite loop, maximum one full round

				while((MAIN_Queue_get(LPWA_rcv_msgs, (uint8_t *) &tmp[0]) == osOK) && (counter<=buff_len))
				{
					char *resp_i= strstr((const char*) &tmp[2], "\r\n\r\n"); //Start searching after 2 chars
					if(resp_i)
					{
						uint16_t local_ctr = 0;	//
						uint8_t delta = 0;
						while(resp_i)
						{
							delta = (uint8_t) (resp_i - &tmp[local_ctr]);
							MAIN_Queue_put(LPWA_rcv_msgs, (uint8_t *) &tmp[local_ctr], delta);
							local_ctr += (uint8_t) (delta);

							#ifdef debug_LPWA
								debug("\r\n[S7022_PROCESS_Rx] Double:%d", local_ctr);
							#endif
							resp_i= strstr(&tmp[local_ctr+4], "\r\n\r\n");
						}
						delta = (uint8_t) strlen(tmp) - local_ctr - 4;
						MAIN_Queue_put(LPWA_rcv_msgs, (uint8_t *) &tmp[local_ctr+4], delta);

						memset(tmp, 0, LPWA_rx_len);
						buff_len = LPWA_rcv_msgs->bufferLength;
						ret = 0;
						counter = 0;
					}else
					{
						resp_i= strstr((const char*) tmp, expected_resp);
						if(resp_i)
						{
							int8_t len1=0, len2=0;
							len1 = strlen(expected_resp) + 2;
							len2 = strlen(tmp);

							//TODO: After > expected response, sometimes it receives +coaprcv msg
							if((len1 - len2) > 4)	//Extra data is received (without double \r\n)
							{
								memset(resp_i, 0, len1);
								//MAIN_Queue_put(LPWA_rcv_msgs, (uint8_t *) &tmp, len2);
							}

							#ifdef debug_LPWA
								debug("\r\n[S7022_PROCESS_Rx][%d] %s", r_type, expected_resp);
							#endif
							ret = 0;
							break;
						}

						resp_i= strstr((const char*) tmp, resp_error);
						if(resp_i)
						{
							#ifdef debug_LPWA
								debug("\r\n[S7022_PROCESS_Rx][%d] ERROR! %s", r_type, tmp);
							#endif
							return 1;
						}

					}
					counter++;
				}
				return ret;
			break;

			case 3: //Confirmation reception
				counter = 0;	//To avoid infinite loop, maximum one full round
				while((MAIN_Queue_get(LPWA_rcv_msgs, (uint8_t *) &tmp[0]) == osOK) && (counter<=buff_len))
				{
					char *resp_i= strstr((const char*) &tmp[2], "\r\n\r\n"); //Start searching after 2 chars

					resp_i= strstr((const char*) tmp, "+COAPRECV:");
					if(resp_i)
					{
						#ifdef debug_LPWA
							debug("\r\n[S7022_PROCESS_Rx][%d] +COAPRECV", r_type);
						#endif
						int code_major = 0;
						int code_minor = 0;
						unsigned int id = 0;
						int data_len = 0;

						// Parse CoAP code as two integers (%d.%d) to bypass missing float support
						if(sscanf((const char *) resp_i, "+COAPRECV: response,from session %*d,%d.%d,%u,%d", &code_major, &code_minor, &id, &data_len) >= 4)
						{
							#ifdef debug_LPWA
								debug("\r\n[S7022_PROCESS_Rx] code:%d.%02d - id:%u - ", code_major, code_minor, id);
							#endif
							if(data_len == 16)
							{
								// Find the start of the payload data
								char *data_start = strrchr((const char *)resp_i, ',');

								if(data_start != NULL)
								{
									char *hex_ptr = data_start + 1;
									received_bitmask = strtoull(hex_ptr, NULL, 16);
									bitmask_ready = 1;
									debug("Bitmask:%s", hex_ptr);
								}
							}
						}
						memset(tmp, 0, LPWA_rx_len);
						ret = 0;
						counter = 0;
						buff_len = LPWA_rcv_msgs->bufferLength;
					} else
					{
						#ifdef debug_LPWA
							debug("\r\n[S7022_PROCESS_Rx][%d] NOT IN (%s)", r_type, tmp);
						#endif
						ret = 2;
					}
					counter++;
				}
			break;

			case 4: //NO RESPONSE
				return 0;
			break;
		}
	}
	/*#ifdef debug_LPWA
		else
			debug("\r\n[S7022_PROCESS_Rx] Nothing to process");
	#endif*/

	return 3;
}

int8_t S7022_SCAN_Rx(struct BUFF_G *LPWA_rcv_msgs, char *expected_resp, char *scan_str, uint8_t *result)
{
	int8_t ret = -1;
	uint8_t tmp[LPWA_rx_len]={'\0'};
	while(MAIN_Queue_get(LPWA_rcv_msgs, tmp) == osOK)
	{
		char *resp_i= strstr((char*) tmp, expected_resp);

		if(resp_i)
		{
			if(sscanf((const char *)resp_i, scan_str, result))
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_SCAN_Rx] result: %s", result);
				#endif
				return 0;
			}
		}
		else
		{
			char *resp_i= strstr((char*) tmp, resp_error);
			if(resp_i)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_SCAN_Rx] ERROR! %s", tmp);
				#endif
				return 1;
			}else
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_SCAN_Rx] NOT IN %s", tmp);
				#endif
				ret = 2;
			}
		}
	}
	return ret;
}

/*uint8_t S7022_Confirm_Rx(uint8_t buffer[max_blocks_LPWA][block_len], uint8_t conf_vector[max_blocks_LPWA])
{
	float32_t code = 1.1;
	uint8_t id = 99;
	uint8_t n_resp = 0;

	#ifdef debug_LPWA
		debug("\r\n[S7022_Confirm_Rx] Started!\r\n");
	#endif

	S7022_PROCESS_Rx(&BUFF_LPWA, 2, resp_dummy); //To check if any confirmation stills in buffer
	while(S7022_rx_get(&LPWA_confirm_msgs, &code, &id) == osOK)
	{
		if(code == 2.01f || code == 2.03f)	//2.01-Created, 2.03-Valid (Already exists), 4.09-When error
		{
			conf_vector[id] = 1;
			#ifdef debug_LPWA
				debug("\r\n[S7022_Confirm_Rx] code:%1.2f - id: %d", code, id);
			#endif
			S7022_rst_ind_err(&LPWA.errors.S7022_SERVER_RESP);
		}else
			#ifdef debug_LPWA
				debug("\r\n[S7022_Confirm_Rx] code (Not managed):%1.2f - id: %d", code, id);
			#endif
	}
	for(uint8_t jj=0; jj<max_blocks_LPWA; jj++)
		n_resp +=conf_vector[jj];
	return n_resp;
}*/

uint8_t S7022_Confirm_Rx(uint8_t buffer[max_blocks_LPWA][block_len])
{
	float32_t code = 1.1;
	uint8_t id = 99;

	#ifdef debug_LPWA
		debug("\r\n[S7022_Confirm_Rx] Started!\r\n");
	#endif

	S7022_PROCESS_Rx(&BUFF_LPWA, 3, resp_dummy); //To check if any confirmation stills in buffer
}

osStatus_t S7022_AT(uint16_t timeout)		//[1] Test communication is correct
{
	uint8_t res = S7022_SEND_CMD_NEW(cmd_AT, 0);
	//debug("\r\n[S7022_AT] S7022_SEND_CMD_NEW\r\n");
	if(res>=0)
	{
		for(uint8_t ii=0; ii<(timeout/LPWA_rx_def_tout); ii++)
		{
			vTaskDelay(LPWA_rx_def_tout);
			if(S7022_PROCESS_Rx(&BUFF_LPWA, 0, NULL)==0)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_AT] OK\r\n");
				#endif
				S7022_rst_ind_err(&LPWA.errors.S7022_AT);
				return osOK;
			}
		}
	}
	#ifdef debug_LPWA
		debug("\r\n[S7022_AT] ERROR[%d]\r\n", res);
	#endif
	S7022_error(&LPWA.errors.S7022_AT);
	return osError;
}

osStatus_t S7022_CGSN(uint8_t *IMEI, uint16_t timeout)
{
	uint8_t res = S7022_SEND_CMD_NEW(cmd_CGSN, 0);
	uint8_t IMEI_tmp[15] =  "123456789112345";

	if(res>=0)
	{
		char *exp_resp = "+CGSN:";
		char *scan_str = "+CGSN: \"%s\"";
		for(uint8_t ii=0; ii<(timeout/20); ii++)
		{
			vTaskDelay(20);
			if(S7022_SCAN_Rx(&BUFF_LPWA, exp_resp, scan_str, IMEI_tmp)==0)
			{
				memcpy(IMEI, &IMEI_tmp[8], 6);
				#ifdef debug_LPWA
					debug("\r\n[S7022_CGSN] IMEI: %s\r\n", IMEI);
				#endif
				return osOK;
			}
		}
	}
	#ifdef debug_LPWA
		debug("\r\n[S7022_CGSN] ERROR\r\n");
	#endif
	return osError;
}

osStatus_t S7022_QCPMUCFG(uint8_t activated, uint16_t timeout)		//
{
	if(activated == 0)
		cmd_QCPMUCFG[14] = '0';	//0-Running, 1-IDLE, 2-Sleep1, 3-Sleep2, 4-Hybernate
	else
		cmd_QCPMUCFG[14] = '1';
	uint8_t res = S7022_SEND_CMD_NEW(cmd_QCPMUCFG, 0);

	if(res>=0)
	{
		for(uint8_t ii=0; ii<(timeout/20); ii++)
		{
			vTaskDelay(20);
			if(S7022_PROCESS_Rx(&BUFF_LPWA, 0, NULL)==0)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_QCPMUCFG] PSM [%d] OK!\r\n", activated);
				#endif
				S7022_rst_ind_err(&LPWA.errors.S7022_QCPMUCFG);
				return osOK;
			}
		}
	}
	#ifdef debug_LPWA
		debug("\r\n[S7022_QCPMUCFG] ERROR\r\n!");
	#endif
	S7022_error(&LPWA.errors.S7022_QCPMUCFG);
	return osError;
}

osStatus_t S7022_CEREG(uint16_t timeout, uint8_t retry) //3.2.5 AT+CREG Network Registration (<n>,<stat>[,[<tac>],[<ci>],[<AcT>][,<cause_type>,<reject_cause>]]) - [1]Page 20
{
	for(uint8_t ii=0; ii<retry; ii++)
	{
		int8_t res = S7022_SEND_CMD_NEW(cmd_CEREG, 0);
		if(res>=0)
		{
			char *exp_resp = "+CEREG:";
			char *scan_str = "+CEREG: %*hu,%hu,";
			for(uint8_t jj = 0; jj<(timeout/(5*LPWA_rx_def_tout)); jj++)
			{
				vTaskDelay(5*LPWA_rx_def_tout);
				if(S7022_SCAN_Rx(&BUFF_LPWA, exp_resp, scan_str, (uint8_t *) &LPWA.S7022_CEREG)==0)
				{
					#ifdef debug_LPWA
						debug("\r\n[S7022_CEREG] Stat[%d/%d]: %d\r\n", ii+1, retry, LPWA.S7022_CEREG);
					#endif
					if(LPWA.S7022_CEREG == 5 || LPWA.S7022_CEREG == 1) //1,5 =>roaming and registered, roaming
					{
						S7022_rst_ind_err(&LPWA.errors.S7022_CEREG);
						return osOK;
					}else if(LPWA.S7022_CEREG == 2) //Trying to attach
					{
						debug("\r\n[S7022_CEREG] Trying to attach!");
						jj = 0;
						vTaskDelay(5*LPWA_rx_def_tout);
					}
				}
			}
		}
		vTaskDelay(100);
	}
	#ifdef debug_LPWA
		debug("\r\n[S7022_CEREG] ERROR!\r\n");
	#endif
	S7022_error(&LPWA.errors.S7022_CEREG);
	return osError;
}

osStatus_t S7022_COAPSTART(uint16_t timeout)	//
{
	if(LPWA.S7022_COAPSTART==1)
		return osOK;
	int8_t res = S7022_SEND_CMD_NEW(cmd_COAPSTART, 0);
	if(res>=0)
	{
		for(uint8_t ii=0; ii<(timeout/LPWA_rx_def_tout); ii++)
		{
			vTaskDelay(LPWA_rx_def_tout);
			if(S7022_PROCESS_Rx(&BUFF_LPWA, 0, NULL)==0)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_COAPSTART] Ok\r\n");
				#endif
				LPWA.S7022_COAPSTART=1;
				S7022_rst_ind_err(&LPWA.errors.S7022_COAPSTART);
				return osOK;
			}
		}
	}

	#ifdef debug_LPWA
		debug("\r\n[S7022_COAPSTART] ERROR\r\n");
	#endif
	S7022_error(&LPWA.errors.S7022_COAPSTART);
	return osError;
}

osStatus_t S7022_COAPSTOP(uint16_t timeout)		//
{
	if(LPWA.S7022_COAPSTART==0)
		return osOK;
	int8_t res = S7022_SEND_CMD_NEW(cmd_COAPSTOP, 0);
	if(res>=0)
	{
		for(uint8_t ii=0; ii<(timeout/200); ii++)
		{
			vTaskDelay(200);
			int8_t res2 = S7022_PROCESS_Rx(&BUFF_LPWA, 0, resp_ok);
			switch(res2)
			{
				case 0:
					#ifdef debug_LPWA
						debug("\r\n[S7022_COAPSTOP] Ok");
					#endif
					LPWA.S7022_COAPSTART=0;
					S7022_rst_ind_err(&LPWA.errors.S7022_COAPSTOP);
					return osOK;
				break;

				case 1: //ERROR
					res = S7022_SEND_CMD_NEW(cmd_COAPSTOP, 0);
					if(res!=0)
					{
						#ifdef debug_LPWA
							debug("\r\n[S7022_COAPSTOP] ERROR 1");
						#endif
						S7022_error(&LPWA.errors.S7022_COAPSTOP);
						return osError;
					}
				#ifdef debug_LPWA
					default:
						debug("\r\n[S7022_COAPSTOP] ERROR DEFAULT");
				#endif
			}
		}
	}
	#ifdef debug_LPWA
		debug("\r\n[S7022_COAPSTOP] ERROR");
	#endif
	S7022_error(&LPWA.errors.S7022_COAPSTOP);
	return osError;
}

osStatus_t S7022_COAPOPEN(uint16_t timeout)	//
{
	if(LPWA.S7022_COAPOPEN==1)
		return osOK;
	LPWA.coap_id = 0;
	uint8_t cmd_COAPOPEN[40];
	sprintf((char *)cmd_COAPOPEN, "AT+COAPOPEN=\"%s\",%d\r", server_IP, server_port);
	uint8_t res = S7022_SEND_CMD_NEW(cmd_COAPOPEN, 0);
	if(res>=0)
	{
		char *exp_resp = "+COAPOPEN: ";
		char *scan_str = "+COAPOPEN: %hu";
		for(uint8_t ii=0; ii<(timeout/LPWA_rx_def_tout); ii++)
		{
			vTaskDelay(LPWA_rx_def_tout);
			if(S7022_SCAN_Rx(&BUFF_LPWA, exp_resp, scan_str, (uint8_t *) &LPWA.coap_id)==0)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_COAPOPEN] Coap id: %d\r\n", LPWA.coap_id);
				#endif
				S7022_rst_ind_err(&LPWA.errors.S7022_COAPOPEN);
				LPWA.S7022_COAPOPEN=1;
				return osOK;
			}
		}
	}
	debug("\r\n[S7022_COAPOPEN] ERROR!\r\n");
	S7022_error(&LPWA.errors.S7022_COAPOPEN);
	return osError;
}

osStatus_t S7022_COAPCLOSE(uint16_t timeout)	//
{
	if(LPWA.S7022_COAPOPEN==0)
		return osOK;

	uint8_t cmd_COAPCLOSE[40];
	sprintf((char *)cmd_COAPCLOSE, "AT+COAPCLOSE=%d\r", LPWA.coap_id);
	uint8_t res = S7022_SEND_CMD_NEW(cmd_COAPCLOSE, 0);
	if(res>=0)
	{
		for(uint8_t ii=0; ii<(timeout/20); ii++)
		{
			vTaskDelay(20);
			if(S7022_PROCESS_Rx(&BUFF_LPWA, 0, resp_ok)==0)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_COAPCLOSE] Ok");
				#endif
				LPWA.S7022_COAPOPEN=0;
				S7022_rst_ind_err(&LPWA.errors.S7022_COAPCLOSE);
				return osOK;
			}
		}
	}
	#ifdef debug_LPWA
		debug("\r\n[S7022_COAPCLOSE] ERROR");
	#endif
	S7022_error(&LPWA.errors.S7022_COAPCLOSE);
	return osError;
}

osStatus_t S7022_COAPHEAD(uint16_t coap_msgid, const uint8_t *coap_tk, uint16_t timeout)	//<coap_id>,<msgId>,<tkl>,<token>
{
	uint8_t cmd_COAPHEAD[40];
	sprintf((char *)cmd_COAPHEAD, "AT+COAPHEAD=%d,%d,%d,\"%s\"\r",LPWA.coap_id, coap_msgid, coap_tkl, coap_tk);
	uint8_t res = S7022_SEND_CMD_NEW(cmd_COAPHEAD, 0);
	if(res>=0)
	{
		for(uint8_t ii=0; ii<(timeout/LPWA_rx_def_tout); ii++)
		{
			vTaskDelay(LPWA_rx_def_tout);
			//if(osMutexAcquire(mutex_lpwaHandle, 10)==osOK)	//TODO: Evaluate if necessary
			//{
				if(S7022_PROCESS_Rx(&BUFF_LPWA, 0, resp_ok)==0)
				{
					#ifdef debug_LPWA
						debug("\r\n[S7022_COAPHEAD] OK\r\n", cmd_COAPHEAD);
					#endif
					S7022_rst_ind_err(&LPWA.errors.S7022_COAPHEAD);
					return osOK;
				}
			/*	osMutexRelease(mutex_lpwaHandle);
			}
			#ifdef debug_LPWA
				else
					debug("\r\n[S7022_COAPHEAD] MUTEX ERROR\r\n");
			#endif*/
		}
		#ifdef debug_LPWA
			debug("\r\n[S7022_COAPHEAD] Timeout\r\n");
		#endif
	}
	#ifdef debug_LPWA
		debug("\r\n[S7022_COAPHEAD] ERROR\r\n");
	#endif
	S7022_error(&LPWA.errors.S7022_COAPHEAD);
	return osError;
}

osStatus_t S7022_COAPOPTION(uint16_t timeout)	//<coap_id>,<opt_count>,<optNum>,<optValue>... up to 10 options
{
	uint8_t cmd_COAPOPTION[40];
	sprintf((char *)cmd_COAPOPTION, "AT+COAPOPTION=%d,%d,%d,\"%s\"\r", LPWA.coap_id, 1 , COAP_OPT_URI_PATH, coap_path);
	uint8_t res = S7022_SEND_CMD_NEW(cmd_COAPOPTION, 0);
	//S7022_SEND_CMD_NEW('\r', 1);
	if(res>=0)
	{
		for(uint8_t ii=0; ii<(timeout/LPWA_rx_def_tout); ii++)
		{
			vTaskDelay(LPWA_rx_def_tout);
			if(S7022_PROCESS_Rx(&BUFF_LPWA, 0, resp_ok)==0)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_COAPOPTION] OK\r\n");
				#endif
				S7022_rst_ind_err(&LPWA.errors.S7022_COAPOPTION);
				return osOK;
			}
		}
	}
	#ifdef debug_LPWA
		debug("\r\n[S7022_COAPOPTION] Timeout\r\n");
	#endif
	S7022_error(&LPWA.errors.S7022_COAPOPTION);
	return osError;
}

osStatus_t S7022_COAPSEND(const uint8_t *data, uint16_t data_len, uint16_t timeout)	//<coap_id>,<type>(“con”,”non”,”ack”,”rst”),
//<method>(“get”,”post”,”put”,”delete”,”fetch”,”patch”,”ipatch”)[,<data_len>,<data>]
{
	uint8_t cmd_COAPSEND[30 + block_len];
	//sprintf((char *)cmd_COAPSEND, "AT+COAPSEND=%d,\"%s\",\"%s\",%u,\"%s\"\r", LPWA.coap_id,"con",
		//	"post", data_len, data);
	sprintf((char *)cmd_COAPSEND, "AT+COAPSEND=%d,\"%s\",\"%s\",%u,\"", LPWA.coap_id,"con","put", data_len);
	uint8_t cmd_len = strlen((const char*) cmd_COAPSEND);
	memcpy(&cmd_COAPSEND[cmd_len], data, data_len);
	memcpy(&cmd_COAPSEND[cmd_len + data_len], "\"\r\0", 3);
	//debug("STRING: %s", cmd_COAPSEND);

	#ifdef debug_LPWA
		debug("\r\n[S7022_COAPSEND] %s",cmd_COAPSEND);
	#endif

	uint8_t res = S7022_SEND_CMD_NEW(cmd_COAPSEND, 0);
	if(res>=0)
	{
		for(uint8_t ii=0; ii<(timeout/20); ii++)
		{
			vTaskDelay(20);
			if(S7022_PROCESS_Rx(&BUFF_LPWA, 0, resp_ok)==0)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_COAPSEND] Ok\r\n");
				#endif
				S7022_rst_ind_err(&LPWA.errors.S7022_COAPSENDTX);
				return osOK;
			}
		}
	}
	debug("\r\n[S7022_COAPSEND] ERROR\r\n");
	S7022_error(&LPWA.errors.S7022_COAPSENDTX);
	return osError;
}

osStatus_t S7022_COAPSENDTX(const uint8_t *data, uint16_t data_len, uint16_t timeout)	//<coap_id>,<type>(“con”,”non”,”ack”,”rst”),
//<method>(“get”,”post”,”put”,”delete”,”fetch”,”patch”,”ipatch”)[,<data_len>,<data>]
{
	uint8_t cmd_COAPSEND[100];
	//sprintf((char *)cmd_COAPSEND, "AT+COAPSENDTX=%d,\"%s\",\"%s\",%u\r", LPWA.coap_id, "con", method, data_len);

	sprintf((char *)cmd_COAPSEND, "AT+COAPSENDTX=%d,\"%s\",\"%s\",%u\r", LPWA.coap_id,"non", "put", data_len);

	//**** TEST BED ****//
//	sprintf((char *)cmd_COAPSEND, "AT+COAPSENDTX=%d,\"non\",\"post\",%u\r", LPWA.coap_id, data_len);

	uint8_t res = S7022_SEND_CMD_NEW(cmd_COAPSEND, 0);

	if(res>=0)
	{
		char *exp_resp = ">";
		//TODO:Validate these timeouts (seems the summation is not timeout)
		for(uint8_t ii=0; ii<(timeout/LPWA_rx_def_tout); ii++)
		{
			vTaskDelay(LPWA_rx_def_tout);
			if(S7022_PROCESS_Rx(&BUFF_LPWA, 2, exp_resp)==0)
			{
				#ifdef debug_LPWA
					debug("\r\n[S7022_COAPSENDTX] >");
				#endif
				vTaskDelay(LPWA_rx_def_tout);
				res = S7022_SEND_CMD_NEW(data, data_len);
				for(uint8_t jj=ii; jj<(timeout/LPWA_rx_def_tout); jj++)
				{

					if(S7022_PROCESS_Rx(&BUFF_LPWA, 0, resp_ok)==0)
					{
						#ifdef debug_LPWA
							debug("\r\n[S7022_COAPSENDTX] Ok\r\n");
						#endif
						S7022_rst_ind_err(&LPWA.errors.S7022_COAPSENDTX);
						return osOK;
					}
					/*#ifdef debug_LPWA
						else
							debug("\r\n[S7022_COAPSENDTX] Error receiving >");
						#endif*/

					vTaskDelay(LPWA_rx_def_tout);
				}
				break;
			}
		}
		#ifdef debug_LPWA
			debug("\r\n[S7022_COAPSENDTX] Timeout\r\n");
		#endif
	}
	#ifdef debug_LPWA
		else
			debug("\r\n[S7022_COAPSENDTX] ERROR\r\n");
	#endif
	S7022_error(&LPWA.errors.S7022_COAPSENDTX);
	return osError;
}

//TODO: Return value should be okOK or a new enumarete for LPWA
int8_t S7022_SEND_CMD_NEW(const uint8_t *cmd, uint8_t data_len)
{
	HAL_StatusTypeDef res_uart=HAL_OK;
	uint8_t cmd_len;

	//TODO: Improve this
	if(data_len == 0)
		cmd_len = strlen((const char *)cmd);
	else
		cmd_len = data_len;

	if((res_uart=HAL_UART_Transmit_DMA(&LPWA_UART, cmd, cmd_len))!=HAL_OK)
	{
		debug("\r\nTx[%d]", res_uart);
		return -1;
	}
	return 0;
}

uint8_t S7022_SEND_CMD(const uint8_t *cmd, uint8_t data_len,  uint8_t resp_buff[3][100], uint16_t timeout)
{
	HAL_StatusTypeDef res_uart=HAL_OK;
	uint8_t	lpwa_ctr_local=0;		//Counter for full response number of characters
	uint8_t	lpwa_n_ctr_local = 0;	//Counter of number of characters for individual response
	uint8_t	lpwa_n_resp = 0;		//Counter of processed responses
	osStatus_t res_cmd = osOK;
	uint8_t cmd_len;

	if(data_len == 0)
		cmd_len = strlen((const char *)cmd);
	else
		cmd_len = data_len;

	if((res_uart=HAL_UART_Receive_DMA(&LPWA_UART, buffer_lpwa, 1))!=HAL_OK)
	{
		debug("\r\nRx[%d]", res_uart);
		return 0;
	}else if((res_uart=HAL_UART_Transmit_DMA(&LPWA_UART, cmd, cmd_len))!=HAL_OK)
	{
		debug("\r\nTx[%d]", res_uart);
		return 0;
	}

	if((res_cmd = osSemaphoreAcquire(sem_lwpaHandle, timeout)) == osOK)
	{
		debug("\r\nbuff(%s):%s", cmd, buffer_lpwa);
		while(buffer_lpwa[lpwa_ctr_local]!=0)
		{
			if(buffer_lpwa[lpwa_ctr_local]=='\r')
			{
				if(lpwa_n_ctr_local>=1)	//Message received apart from \r\n
				{
					resp_buff[lpwa_n_resp][lpwa_n_ctr_local] = 0;
					lpwa_n_resp++;
				}
				lpwa_n_ctr_local=0;
				lpwa_ctr_local+=1;	//Jump \n
				buffer_lpwa[lpwa_ctr_local] = 0;
			}else
			{
				resp_buff[lpwa_n_resp][lpwa_n_ctr_local] = buffer_lpwa[lpwa_ctr_local];
				lpwa_n_ctr_local++;
			}
			buffer_lpwa[lpwa_ctr_local] = 0;
			lpwa_ctr_local++;
		}
		return lpwa_n_resp;
	}
	else
		debug("\r\nErr Code: %d", res_cmd);
	return 0;
}

void S7022_error(uint8_t *source)
{
	//TODO: Handle individual error sources, for example: S7022_COAPOPEN could mean server down, S7022_CEREG could mean no network or another error
	*source = *source+1;
	#ifdef debug_LPWA
		debug("\r\n[S7022_error]: %d \r\n", *source);
	#endif
	//Error due to M2M communication
	if((*source>=lpwa_err_lim_rst) & ((source == &LPWA.errors.S7022_AT) | (source == &LPWA.errors.S7022_COAPHEAD)
			| (source == &LPWA.errors.S7022_COAPOPTION) | (source == &LPWA.errors.S7022_COAPSTART)
			| (source == &LPWA.errors.S7022_COAPOPEN) | (source == &LPWA.errors.S7022_COAPCLOSE)
			| (source == &LPWA.errors.S7022_COAPSTOP) | (source == &LPWA.errors.S7022_QCPMUCFG)))
	{
		debug("\r\n\r\n Reset sent!");
		S7022_rst_all();
		S7022_RST();
		restart_UART_DMA();
	}

	//Error due to server communication
	//TODO: Correct this
	if((*source>=lpwa_err_lim_tout)&((source == &LPWA.errors.S7022_COAPSENDTX) | (source == &LPWA.errors.S7022_SERVER_RESP)))
	{
		if((*source)<(lpwa_err_lim_tout + 2))
		{
			#ifdef debug_LPWA
				debug("\r\n[S7022_error] SHORT timeout started (%d)!", *source);
			#endif
			osTimerStart(timeout_LPWAHandle, lwpa_short_timeout);
		}else
		{
			#ifdef debug_LPWA
				debug("\r\n[S7022_error] LONG Timeout started (%d)!", *source);
			#endif
			osTimerStart(timeout_LPWAHandle, lwpa_long_timeout);
		}
		HAL_GPIO_WritePin(RST_port, RST_pin, GPIO_PIN_RESET);
		LPWA.status = 1;
		osSemaphoreRelease(sem_process_LPWAHandle);
		return;
	}

	//Error due to module communication to network
	if((*source>=lpwa_err_lim_tout)&(source == &LPWA.errors.S7022_CEREG))
	{
		if((LPWA.S7022_CEREG == 0) | (LPWA.S7022_CEREG == 3) | (LPWA.S7022_CEREG == 4) |
				(LPWA.S7022_CEREG == 2) | (LPWA.S7022_CEREG > 5))//Not registered, denied, unknown or emergency only (CHANGE OPERATOR)
		{
			S7022_COPS(operator_list[LPWA.operator]);
			LPWA.operator++;
			if(LPWA.operator>2)
				LPWA.operator=0;
			debug("\r\n\r\n Timeout started(registering for other operator)!");
			LPWA.status = 1;
			vTaskDelay(10);
			osTimerStart(timeout_LPWAHandle, lwpa_short_timeout);
		}		//TODO: IMPLEMENT CEREG = -1, which means neither "OK" nor "ERROR" has been received
		else if(LPWA.S7022_CEREG == -1)//Error in M2M communication
		{
			debug("\r\n\r\n Reset sent!");
			S7022_RST();
		}
		S7022_rst_all();
	}
	//restart_UART_DMA();
}

void S7022_rst_ind_err(uint8_t *source)
{
	*source = 0;
}

void S7022_rst_all()
{
	LPWA.errors.S7022_AT=0;
	LPWA.errors.S7022_CEREG=0;
	LPWA.errors.S7022_COAPHEAD=0;
	LPWA.errors.S7022_COAPOPEN=0;
	LPWA.errors.S7022_COAPOPTION=0;
	LPWA.errors.S7022_COAPSENDTX=0;
	LPWA.errors.S7022_COAPSTART=0;
	#ifdef debug_LPWA
		debug("\r\n[S7022_rst_all]");
	#endif
}

void S7022_RST()
{
	LPWA.S7022_CEREG=0;
	LPWA.S7022_COAPOPEN = 0;
	LPWA.S7022_COAPSTART = 0;
	LPWA.coap_id = 99;
	LPWA.operator = 0;
	LPWA.status = 3;

	HAL_GPIO_WritePin(RST_port, RST_pin, GPIO_PIN_RESET);
	vTaskDelay(100);
	HAL_GPIO_WritePin(RST_port, RST_pin, GPIO_PIN_SET);
	#ifdef debug_LPWA
		debug("\r\n[S7022_RST]");
	#endif
	vTaskDelay(100);
}

osStatus_t S7022_PIN()		//[1] Test sim card is ok
{
	uint8_t n_res = S7022_SEND_CMD(cmd_PIN, 0, buffer_lpwa_resp, 1000);
	if(n_res>0)
	{
		if(strstr((const char *)buffer_lpwa_resp[0], "+CPIN: READY")!= NULL)
			return osOK;
	}
	return osError;
}

osStatus_t S7022_CSQ(unsigned short *rssi, unsigned short *ber)		//[1] Check signal quality <rssi>,<ber>
{
	uint8_t n_res = S7022_SEND_CMD(cmd_CSQ, 0, buffer_lpwa_resp, 1000);

	if(n_res>0)
	{
		if(sscanf((const char *)buffer_lpwa_resp[0], "+CSQ:%hu,%hu", rssi, ber) >= 1)
			return osOK;
	}
	return osError;
}

osStatus_t S7022_CGATT()	//3.2.12 AT+CGATT Attach or Detach from GPRS Service - [1]Page 31
{
	uint8_t n_res = S7022_SEND_CMD(cmd_CGATT, 0, buffer_lpwa_resp, 1000);
	uint8_t attached = 0;
	if(n_res>0)
	{
		if(sscanf((const char *)buffer_lpwa_resp[0], "+CGATT:%hhu", &attached) >= 1)
		{
			if(attached == 1)
				return osOK;
		}
	}
	return osError;
}

osStatus_t S7022_COPS(uint8_t *operator)	//3.2.11 AT+COPS Operator Selection (<mode>[,<format>,<oper>][,<AcT>]) - [1]Page 29
{
	uint8_t cmd_COPS[100];

	sprintf((char *)cmd_COPS, "AT+COPS=4,2,\"%s\"\r", operator);
	debug("\r\n%s",cmd_COPS);

	uint8_t n_res = S7022_SEND_CMD(cmd_COPS, 0, buffer_lpwa_resp, 1000);
	if(n_res>0)
	{
		for(int rr=0; rr< n_res; rr++)
			debug("\r\n[%d]:%s", rr, &buffer_lpwa_resp[rr]);
		return osOK;
	}
	return osError;
}

osStatus_t S7022_QCDNS()	//3.2.11 AT+COPS Operator Selection (<mode>[,<format>,<oper>][,<AcT>]) - [1]Page 29
{
	uint8_t n_res = S7022_SEND_CMD(cmd_QCDNS, 0, buffer_lpwa_resp, 10000);
	if(n_res>0)
	{
		/*for(int rr=0; rr< n_res; rr++)
			debug("\r\n[%d]:%s", rr, &buffer_lpwa_resp[rr]);*/
		return osOK;
	}
	return osError;
}


//[1] - SIM7022 Series_AT Command Manual_V1.01
//[2] - SIM7022 Series_CoAP_Application Note_V1.00
//[3] - rfc7252.pdf
//[4] - SIM7022 Series_AT Command Manual_V1.05
