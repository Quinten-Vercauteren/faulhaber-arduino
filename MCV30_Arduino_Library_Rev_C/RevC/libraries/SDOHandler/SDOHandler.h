#ifndef SDOHANDLER_H
#define SDOHANDLER_H

/*--------------------------------------------------------------
 * class SDOHandler
 * handles R/W of parameters via SDO
 * uses an already existing instance of the MsgHandler
 *
 * 2020-05-24 AW Frame
 * 2021-04-21 removed reference to any timer service
 *
 *-------------------------------------------------------------*/
 
//--- inlcudes ----
 
#include <MsgHandler.h>
#include <stdint.h>

//--- SDO service defines ---

//--- define the max size of the msg
typedef struct __attribute__((packed)) SDOMaxMsg {
   uint8_t u8Prefix  : 8;
   uint8_t u8Len     : 8;
   uint8_t u8NodeNr  : 8;
   MCMsgCommands u8Cmd : 8;
   uint16_t  Idx;
   uint8_t SubIdx;
   uint8_t u8UserData[4];
   uint8_t CRC : 8;
   uint8_t u8Suffix	: 8;
} SDOMaxMsg;


//define a Msg of type SDORxRequest - payload only

typedef struct __attribute__((packed)) SDORxRq_Data {
   uint16_t  Idx;
   uint8_t SubIdx;
} SDORxRq_Data;


typedef struct __attribute__((packed)) SDORxResp_Data {
   uint16_t  Idx;
   uint8_t SubIdx;
   uint8_t Data;
} SDORxResp_Data;

//define a Msg of type SDOTxRequest
 
typedef struct __attribute__((packed)) SDOTxRq_Data {
   uint16_t  Idx;
   uint8_t SubIdx;
   uint8_t Data;
} SDOTxRq_Data;


typedef struct __attribute__((packed)) SDOTxResp_Data {
   uint16_t  Idx;
   uint8_t SubIdx;
} SDOTxResp_Data;

//define a Msg of type SDOErrorResponse

typedef SDOTxResp_Data SDOErrResp_Data;

//define the enum with the Comm states

typedef enum SDOCommStates {
	eIdle,
	eWaiting,
	eBusy,
	eDone,
	eError,
	eRetry,
	eTimeout
}
 SDOCommStates;

//define the class itself

class SDOHandler {
	public:
		SDOHandler();
		void init(MsgHandler *,uint8_t);
		void SetActTime(uint32_t);
		
		SDOCommStates ReadSDO(uint16_t, uint8_t);
		SDOCommStates WriteSDO(uint16_t, uint8_t,uint32_t *,uint8_t);
		unsigned long GetObjValue();
		SDOCommStates CheckComState();
		void ResetComState(); 
		void SetTORetryMax(uint8_t);
		void SetBusyRetryMax(uint8_t);
		
		//handler to be registered at the Msghandler instance
		static void OnSDOMsgRxCb(void *op,void *p) {
			((SDOHandler *)op)->OnRxHandler((MCMsg *)p);
		};
		
		//hander to be registered at the OsTimer
		static void OnTimeOutCb(void *p) {
			((SDOHandler *)p)->OnTimeOut();
		};
	
	private:
		void OnRxHandler(MCMsg *);
		void OnTimeOut();
		char Channel = InvalidSlot;

		SDOMaxMsg TxRqMsg;
		SDOMaxMsg RxRqMsg;
		SDOCommStates RxTxState = eIdle;

		unsigned long RxData;
		char RxLen;

		MsgHandler *Handler;
		
		uint32_t RequestSentAt;
		bool isTimerActive = false;
		uint32_t actTime;

		bool hasMsgHandlerLocked = false;
				
		uint8_t TORetryCounter = 0;
		uint8_t TORetryMax = 1;
		uint8_t BusyRetryCounter = 0;
		uint8_t BusyRetryMax = 3;
};
 

#endif
