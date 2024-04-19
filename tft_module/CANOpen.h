/*
 * CANOpen.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Oleksii
 */

#ifndef INC_CANOPEN_H_
#define INC_CANOPEN_H_



/* --------------- ERROR ----------------*/

#define OK_SAVE         0x60
#define RESPONSE_ERROR  0x80

const uint32_t error_msg[]={

#define NO_ERROR         0
0x00000000,
#define ERROR_NO_ACCESS  1
0x06010000,
#define ERROR_NO_READ    2
0x06010001,
#define ERROR_NO_SAVE    3
0x06010002,
#define ERROR_NO_OBJECT  4
0x06020000,
#define ERROR_OBJECT_PDO 5
0x06040041,
#define ERROR_SIZE_PDO   6
0x06040042,
#define ERROR_LEN_OBJECT 7
0x06070010,
#define ERROR_bLEN_OBJECT 8
0x06070012,
#define ERROR_sLEN_OBJECT 9
0x06070013,
#define ERROR_SUB_INDEX	  10
0x06090011,
#define ERROR_SYSTEM      11
0x06040047,
#define ERROR_EQUIPMENT   12
0x06060000,
#define ERROR_TOGGLE_BIT  13
0x05030000,
#define ERROR_SDO_SERVER  14
0x05040001,
#define ERROR_DATA        15
0x06090030,
#define ERROR_BIG_DATA_OBJ 16
0x06090031,
#define ERROR_SMALL_DATA_OBJ  17
0x06090032,
#define ERROR_ALL_OD_TABLE 18
0x08000000,
#define ERROR_OBJ_DICT    19
0x80000023,
};

/* deftype */

#define boolean         0x01
#define INT8            0x02
#define INT16           0x03
#define INT32           0x04
#define UINT8           0x05
#define UINT16          0x06
#define UINT32          0x07
#define REAL32          0x08
#define VISIBLE_STRING	0x09
#define OCTET_STRING	0x0A
#define UNICODE_STRING	0x0B
#define TIME_OF_DAY     0x0C
#define TIME_DIFFERENCE	0x0D

#define DOMAIN		0x0F
#define INT24		0x10
#define REAL64		0x11
#define INT40		0x12
#define INT48		0x13
#define INT56		0x14
#define INT64		0x15
#define UINT24		0x16

#define UINT40		0x18
#define UINT48		0x19
#define UINT56		0x1A
#define UINT64		0x1B

#define PDO_COMM        0x20
#define PDO_MAPPING     0x21
#define SDO_PARAMETER	0x22
#define IDENTITY        0x23

/*Object Dictionary TYPE Definitions cia301*/
#define OD_NULL 0
/* Large variable amount of data e.g. executable program code */
#define OD_DOMAIN 2
/*A multiple data field object where the data fields may be any combination of
simple variables. Sub-index 0 is of UNSIGNED8 and sub-index 255 is of
UNSIGNED32 and therefore not part of the RECORD data */
#define OD_RECORD 9
/* Defines a new record type e.g. the PDO mapping structure at 21h*/
#define OD_DEFSTRUCT 6
/* Denotes a type definition such as a BOOLEAN, UNSIGNED16, FLOAT and so on*/
#define OD_DEFTYPE 5
/* A single value such as anUNSIGNED8, BOOLEAN, FLOAT,INTEGER16, VISIBLE STRING etc.*/
#define OD_VAR  7
/* A multiple data field object where each data field is a simple variable of
the SAME basic data type e.g. array of UNSIGNED16 etc. Sub-index 0 is
of UNSIGNED8 and therefore not part of the ARRAY data*/
#define OD_ARRAY 8


const uint32_t subindex_FF[]={

#define _OD_NULL 0
0x00000000,
#define UINT8_OD_VAR 1
0x00050007,
#define UINT16_OD_VAR 2
0x00060007,
#define UINT24_OD_VAR 3
0x00160007,
#define UINT32_OD_VAR 4
0x00070007,
#define UINT8_OD_ARRAY 5
0x00050008,
#define UINT16_OD_ARRAY 6
0x00060008,
#define UINT24_OD_ARRAY 7
0x00160008,
#define UINT32_OD_ARRAY 8
0x00070008,
#define PDO_COMM_OD_DEFSTRUCT 9
0x00200006,
#define PDO_MAPPING_OD_DEFSTRUCT 10
0x00210006,
#define SDO_PARAMETER_OD_DEFSTRUCT 11
0x00220006,
#define IDENTITY_DEFSTRUCT 12
0x00230006,
};
#define  txSDO  0x580
#define  rxSDO  0x600

#define  txPDO1 0x180
#define  txPDO2 0x280
#define  txPDO3 0x380
#define  txPDO4 0x480

#define  rxPDO1 0x200
#define  rxPDO2 0x300
#define  rxPDO3 0x400
#define  rxPDO4 0x500

// command NMT
#define id_NMT_control 0
#define NMT_stop 2
#define NMT_start 1
#define NMT_pre_operational 128
#define NMT_reset 129

// status Block Rele

#define NMT_status_Stopped 0x04
#define NMT_status_Operational 0x05
#define NMT_status_Pre_Operational 0x7F

/*Attribute*/

#define _CONST	0
#define RO      0b0001
#define WO      0b0010
#define RW      0b0011
#define NO_MAP  0b0100


/* structure */

#define SDO_request  0
#define MAP_info     1

/*Initiate SDO Protocol

------ expedited transfer (быстрая загрузка/выгрузка)---

Download  to client --> server
request
--------
[byte 0] (command;
bit 7..5
0x01 - Initiate_download_request  (запрос на загрузку на сервер)
bit4   0
bit3.2 n- no_used_byte ( 4 - n);
bit1   e - 0 normal transfer / 1 - expedited (быстрая)
bit0   s - 0 - off n_byte / 1 - on n_byte   (n не указан или указан)
multiplexor
[byte 1-2] index
[byte 3]   sub-index
[byte 4-7] data

response
---------
bit 7..5  0x03 - Initiate_download_response (ответ от сервера клиенту, что все ок)
bit 4..0  0
multiplexor
[byte 1-2] index
[byte 3]   sub-index

Error
---------
0x04 - Error (ошибка)

Upload to client <-- server

request
--------

bit 7..5  0x02 - initiate upload request (запрос инициации выгрузки c сервера)
bit 4..0  0
multiplexor
[byte 1-2] index
[byte 3]   sub-index

response
--------
[byte 0] (command;
bit 7..5 0x02 - Initiate_upload_request  (запрос на загрузку на сервер)
bit4   0
bit3.2 n- no_used_byte ( 4 - n);
bit1   e - 0 normal transfer / 1 - expedited (быстрая)
bit0   s - 0 - off n_byte 	 / 1 - on n_byte   (n не указан или указан)
multiplexor
[byte 1-2] index
[byte 3]   sub-index
[byte 4-7] data



*/

#define Initiate_download_request  0x20
#define Initiate_download_response 0x60
#define Initiate_upload_response   0x40
#define Initiate_upload_request    0x40
#define Error_answer			   0x80


#define n_no_byte_used  		   0x0C
#define flag_Expedited_SDO 			0x02
#define flag_Size_byte    				0x01



/*----------------------------- CAN buffer -------------------------*/

#define MAX_BUFFER_CAN 128

struct CAN_frame{

	uint32_t id;
	uint32_t  dlc;
	uint32_t msg[2];

};

struct  CAN_buffer{

	struct CAN_frame* rdata;
	struct CAN_frame* wdata;
	struct CAN_frame* begin_frame;
	struct CAN_frame* end_frame;

};

void init_can_buffer(struct CAN_buffer* buf,struct CAN_frame *frame,uint16_t size){

	buf->rdata = frame;
	buf->wdata = frame;
	buf->begin_frame = frame;
	buf->end_frame = frame + size;

};
uint8_t read_can_buffer(struct CAN_buffer* buf,struct CAN_frame *frame){

	if(buf == NULL) return 1;
	if(buf->rdata == buf->wdata) return 1;
	frame->id = buf->rdata->id;
	frame->msg[0] = buf->rdata->msg[0];
	frame->msg[1] = buf->rdata->msg[1];
	frame->dlc = buf->rdata->dlc;
	buf->rdata ++;
	if(buf->rdata >= buf->end_frame) buf->rdata = buf->begin_frame;
	return 0;
};

struct CAN_frame* read_can_buffer2(struct CAN_buffer* buf){

	struct CAN_frame *frame ;
	if(buf == NULL) return NULL;
	if(buf->rdata == buf->wdata) return NULL;
	frame = buf->rdata ++;
	if(buf->rdata >= buf->end_frame) buf->rdata = buf->begin_frame;
	return frame;
};

uint8_t write_can_buffer (struct CAN_buffer* buf, struct CAN_frame *frame){

	if(buf == NULL) return 1;
	struct CAN_frame *fr =  buf->wdata;
	buf->wdata->id = frame->id;
	buf->wdata->msg[0] = frame->msg[0];
	buf->wdata->msg[1] = frame->msg[1];
	buf->wdata->dlc = frame->dlc;
	buf->wdata++;
	if(buf->wdata >= buf->end_frame) buf->wdata = buf->begin_frame;

	if(buf->rdata == buf->wdata) {buf->wdata = fr;return 1;}
	return 0;
};

/*---------------------------------------------------------------------*/


struct Data_Object{

    uint8_t     request_type;
    uint8_t     attribute;
    uint8_t     nbit;
    uint8_t     sub_index;
    uint8_t     sub_index_ff;
    void*       data_object;
    void*       rw_object;

};

struct OD_Object{

    uint16_t index;
    void*     data;
    void (*data_func)(struct Data_Object*);

};

struct one_type_array{

    uint8_t sub_index;
    void*   array;

};

struct record_arr_object{

    uint8_t  sub_index;
    uint8_t* nbit;    //arr[subindex] = {0x08,0x20....0x10};
    void**   array; //arr[subindex];*uint8,*uint?_t .....

};

struct string_object{

    uint8_t*    text;
    uint8_t*    text_buffer;
    uint8_t     n_byte;;
    uint8_t     cond_sdo;

};



struct map_info{

	uint8_t  nbit;
	uint8_t  sub_index;
	uint16_t index;

};

union  map_data{

    uint32_t        data32;
    struct map_info info;

};

struct PDO_Object{

    uint32_t*	cob_id ;

    uint8_t*	sub_index ;
    uint8_t*	Transmission_type;
    uint8_t*	Sync_start_value;
    uint8_t *   counter_sync;

    uint16_t*	Inhibit_time; 	// n x 100ms
    uint16_t*	counter_Inhibit_time; // 0 <-- (Inhibit_time --)

    uint16_t*	Event_timer;	// n x 100ms
    uint16_t*	counter_Event_timer;  // 0 <-- (Event_timer--)

    // mapping

    uint8_t*	sub_index_map;
    uint32_t*   map;	      // massiv[MAX_MAP_DATA]
    uint8_t**	map_addr_obj; // massiv[MAX_MAP_DATA]
    uint8_t*    n_byte_pdo_map;

    struct
    OD_Object*	OD_Object_list; //

    // Buffer

    uint8_t*    buffer;

    // function

    void   (*init_pdo)(struct PDO_Object* pdo);
    void   (*build_map)(struct PDO_Object* pdo);
    void   (*process_rxpdo)(struct PDO_Object *pdo);
    void   (*process_txpdo)(struct PDO_Object *pdo);

};

// ROM memory
struct SDO_Object{

    uint8_t*	cond;

    uint8_t*	sub_index;
    uint8_t*	node_id;
    uint32_t*	cob_id_client;
    uint32_t*	cob_id_server;

};

struct data_in_can_message{

	uint32_t  id_object;
	uint32_t* data_in_message;
	uint32_t  data_frame[2];
	uint8_t   dlc_frame;
	uint8_t   n_bits;
	uint8_t   bit_offset;
	uint8_t   status;

};





struct CANopen{

	uint32_t  	can_id;
	struct
	PDO_Object**  pdo;
	uint8_t		  n_pdo;
	struct
	SDO_Object**  sdo;
	uint8_t		  n_sdo;





};












#endif /* INC_CANOPEN_H_ */
