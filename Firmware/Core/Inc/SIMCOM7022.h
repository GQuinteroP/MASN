/*
 * SIMCOM7022.h
 *
 *  Created on: Jun 2, 2023
 *      Author: ubuntu
 */

#ifndef INC_SIMCOM7022_H_
#define INC_SIMCOM7022_H_

#include "cmsis_os.h"
#include "stm32l4xx_hal.h"
#include "main.h"

//Debug
extern void kappa(const char *fmt, ...);

//Must be declared previously in main
extern UART_HandleTypeDef huart3;
extern osSemaphoreId_t sem_lwpaHandle;
extern uint8_t buffer_lpwa_ext[250];
extern uint8_t buffer_lpwa[250];
extern uint8_t buffer_state;
extern osMutexId_t iot_mutexHandle;
extern uint8_t coap_custom_res_flag;
extern uint8_t coap_custom_res[50];
extern void restart_UART_DMA();
extern osTimerId_t timeout_LPWAHandle;
extern const uint8_t	server_IP[15];
extern const uint16_t	server_port;
extern osMutexId_t mutex_lpwaHandle;

extern osSemaphoreId_t sem_process_LPWAHandle;
extern struct BUFF_G BUFF_LPWA;	//TODO: If necessary, LPWA_msgs buffer size could be reduced
osStatus_t MAIN_Queue_put(struct BUFF_G *queue, uint8_t *msg_in, uint8_t msg_len);
osStatus_t MAIN_Queue_get(struct BUFF_G *queue, uint8_t *msg_out);

extern volatile uint64_t received_bitmask;
extern volatile uint8_t bitmask_ready;

//RST pins
#define		RST_pin		LPWA_NRST_Pin
#define 	RST_port	LPWA_NRST_GPIO_Port
#define 	LPWA_UART	huart3
#define		lpwa_err_lim_rst	5	//Times an error (not related to server or network) can occur before reset
#define		lpwa_err_lim_tout	2	//Times an error related to server or network can occur before timeout

#define		lwpa_short_timeout	30000
#define		lwpa_long_timeout	2*60000//3600000

struct LWPA_error {
  uint8_t S7022_AT;
  uint8_t S7022_CEREG;
  uint8_t S7022_COAPSTART;
  uint8_t S7022_COAPSTOP;
  uint8_t S7022_COAPOPEN;
  uint8_t S7022_COAPCLOSE;
  uint8_t S7022_COAPHEAD;
  uint8_t S7022_COAPOPTION;
  uint8_t S7022_COAPSENDTX;
  uint8_t S7022_QCPMUCFG;
  uint8_t S7022_SERVER_RESP;
};

struct LWPA_def
{
  uint8_t S7022_COAPSTART;	//Flag - started client
  uint8_t S7022_COAPOPEN;	//Flag - open connection
  int8_t S7022_CEREG;		//Flag - network attachment connection ([4] - P: 26/257)
  int8_t coap_id;			//Retrieved after executing COAPOPEN
  uint8_t operator;			//Selected operator
  uint8_t status;			//LWPA status: 0 - active / 1 - timed out (15s) / 2 - Sending data / 3 - Disabled / 4 - Starting
  struct LWPA_error errors;
};

typedef enum
 {
    COAP_OPT_IF_MATCH       = 1,   //RFC 7252
    COAP_OPT_URI_HOST       = 3,   //RFC 7252
    COAP_OPT_ETAG           = 4,   //RFC 7252
    COAP_OPT_IF_NONE_MATCH  = 5,   //RFC 7252
    COAP_OPT_OBSERVE        = 6,   //RFC 7641
    COAP_OPT_URI_PORT       = 7,   //RFC 7252
    COAP_OPT_LOCATION_PATH  = 8,   //RFC 7252
    COAP_OPT_URI_PATH       = 11,  //RFC 7252
    COAP_OPT_CONTENT_FORMAT = 12,  //RFC 7252
    COAP_OPT_MAX_AGE        = 14,  //RFC 7252
    COAP_OPT_URI_QUERY      = 15,  //RFC 7252
    COAP_OPT_ACCEPT         = 17,  //RFC 7252
    COAP_OPT_LOCATION_QUERY = 20,  //RFC 7252
    COAP_OPT_BLOCK2         = 23,  //RFC 7959
    COAP_OPT_BLOCK1         = 27,  //RFC 7959
    COAP_OPT_SIZE2          = 28,  //RFC 7959
    COAP_OPT_PROXY_URI      = 35,  //RFC 7252
    COAP_OPT_PROXY_SCHEME   = 39,  //RFC 7252
    COAP_OPT_SIZE1          = 60,  //RFC 7252
    COAP_OPT_ECHO           = 252, //RFC 9175
    COAP_OPT_NO_RESPONSE    = 258, //RFC 7967
    COAP_OPT_REQUEST_TAG    = 292  //RFC 9175
 } CoapOptionNumber;


uint8_t S7022_PROCESS_Rx(struct BUFF_G *LPWA_rcv_msgs, uint8_t r_type, const char *expected_resp);
int8_t S7022_SCAN_Rx(struct BUFF_G *LPWA_rcv_msgs, char *expected_resp, char *scan_str, uint8_t *result);
//uint8_t S7022_Confirm_Rx(uint8_t buffer[max_blocks_LPWA][block_len], uint8_t conf_vector[max_blocks_LPWA]);
uint8_t S7022_Confirm_Rx(uint8_t buffer[max_blocks_LPWA][block_len]);

uint8_t S7022_SEND_CMD(const uint8_t *cmd, uint8_t data_len,  uint8_t resp_buff[3][100], uint16_t timeout);
int8_t S7022_SEND_CMD_NEW(const uint8_t *cmd, uint8_t data_len);

void S7022_RST();
osStatus_t S7022_AT(uint16_t timeout);
osStatus_t S7022_CGSN(uint8_t *IMEI, uint16_t timeout);
osStatus_t S7022_QCPMUCFG(uint8_t activated, uint16_t timeout);
osStatus_t S7022_CEREG(uint16_t timeout, uint8_t retry);

osStatus_t S7022_PIN();
osStatus_t S7022_CSQ(unsigned short *rssi, unsigned short *ber);
osStatus_t S7022_CGATT();
osStatus_t S7022_COPS();


//CoAP
osStatus_t S7022_COAPSTART(uint16_t timeout);
osStatus_t S7022_COAPSTOP(uint16_t timeout);
osStatus_t S7022_COAPOPEN(uint16_t timeout);	//
osStatus_t S7022_COAPCLOSE(uint16_t timeout);	//
osStatus_t S7022_COAPHEAD(uint16_t coap_msgid, const uint8_t *coap_tk, uint16_t timeout);	//
osStatus_t S7022_COAPOPTION(uint16_t timeout);	//
osStatus_t S7022_COAPSENDTX(const uint8_t *data, uint16_t data_len, uint16_t timeout);	//
osStatus_t S7022_COAPSEND(const uint8_t *data, uint16_t data_len,uint16_t timeout);	//

osStatus_t S7022_QCDNS();	//Get IP from URL


void S7022_error(uint8_t *source);
void S7022_rst_ind_err(uint8_t *source);
void S7022_rst_all();

#endif /* INC_SIMCOM7022_H_ */
