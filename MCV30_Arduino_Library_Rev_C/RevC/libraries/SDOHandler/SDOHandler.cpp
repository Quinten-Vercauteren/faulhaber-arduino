/*---------------------------------------------------
 * SDOHandler.cpp
 * handels R/W access to deive parameters via SDO services
 * does itself no interpreation
 *
 * 2020-05-24 AW Frame
 * 2021-04-21 removed reference to any timer service
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <SDOHandler.h>

#define DEBUG_RXMSG		0x0001
#define DEBUG_WREQ		0x0002
#define DEBUG_RREQ		0x0004
#define DEBUG_ERROR		0x0008
#define DEBUG_TO		0x0010

#define DEBUG_SDO (DEBUG_TO | DEBUG_ERROR)

//--- implementation ---

const unsigned int SDORespTimeOut = 20;

//--- public calls ---

/*---------------------------------------------------
 * SDOHandler()
 * init the instance by at least initializing the RxLen
 * 
 * 2020-11-18 AW Done
 *---------------------------------------------------*/
 
SDOHandler::SDOHandler()
{
	RxLen = 0;
}

/*-------------------------------------------------------
 * void init(MsgHandler *,uint8_t)
 * create a functor to register this instance of a SDOHandler
 * at the Msghander which is referred to.
 * The SDOHandler will store the pointer to the Msghandler for further
 * use. Needs to be given the handle under which the node is registered
 * at the MsgHandler and uses this to finally regsiter the Call-back for
 * SDO messages
 * 
 * 2020-11-18 AW Done
 * ---------------------------------------------------------------*/

void SDOHandler::init(MsgHandler *ThisHandler, uint8_t Handle)
{
	pfunction_holder Cb;
	//register Cb
	Cb.callback = (pfunction_pointer_t)SDOHandler::OnSDOMsgRxCb;
	Cb.op = (void *)this;
	
	Handler = ThisHandler;
	Channel = Handle;
	Handler->Register_OnRxSDOCb(Channel,&Cb);
	RxTxState = eIdle;	
}

/*---------------------------------------------------------------
 * SDOCommStates CheckComState()
 * return the state of either the Rx or Tx of an SDO
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------------*/
 
SDOCommStates SDOHandler::CheckComState()
{
	return RxTxState;
}

/*--------------------------------------------------------------
 * void SetTORetryMax(uint8_t)
 * An attempted transfer can run into a timeout either while trying to send
 * or by waiting for a response. The Time-out counter is incremented if 
 * a consecutive occurs. the call here cen be used to modify the
 * default max value for the number of time-outs in a row
 * 
 * 2020-11-18 AW Done
 * --------------------------------------------------------------*/ 

void SDOHandler::SetTORetryMax(uint8_t value)
{
	TORetryMax = value;
}

/*--------------------------------------------------------------
 * void SetBusyRetryMax(uint8_t)
 * An attempted transfer can fail because the Msghandler and Uart are blocked
 * either while trying to send a request. The Time-out counter is incremented if 
 * a consecutive occurs. the call here cen be used to modify the
 * default max value for the number of time-outs in a row
 * 
 * 2020-11-18 AW Done
 * --------------------------------------------------------------*/
 
void SDOHandler::SetBusyRetryMax(uint8_t value)
{
	BusyRetryMax = value;
}

/*----------------------------------------------
 * void SDOHandler::ResetComState()
 * to be called after each interaction to 
 * move the RxTxState from eDone to eIdle
 * 
 * 2020-10-16 AW inital
 * ---------------------------------------------*/

void SDOHandler::ResetComState()
{
	RxTxState = eIdle;
	TORetryCounter = 0;
	BusyRetryCounter = 0;
	//Handler should not be reset, as it could be used by different
	//instances of the Drive
	//Handler->ResetMsgHandler();	
	//Handler can be unlocked if it is still denoted to be locked
	
	if(hasMsgHandlerLocked)
	{
		Handler->UnLockHandler();
		hasMsgHandlerLocked = false;
	}
}

/*-------------------------------------------------------------
 * SDOCommStates ReadSDO(uint16_t Idx, uint8_t SubIdx)
 * Try to read a drive parameter identified by its Idx and SubIdx.
 * Is using a step sequence of sendig a request, waiting for an answer
 * and giving it a retry if not sucessfull.
 * Reception fo the responses is via the OnRxHandler().
 * RxTxState is used to handle the steps and as the central feedback
 * ReadSDO will end up in eDone state to indicate the requested value
 * has been received and can be read.
 * So actuall reading the value will reset the communication state to eIdle.
 * 
 * As any access to the MsgHandler ReadSDO will lock the Msghandler and
 * will only unlock it the actual service failed.
 * Successful servie will unlock in OnRxHandler().

 * 
 * 2020-11-18 AW Rev_A
 * 2021-04-22 AW Removed reference to timer service
 * -------------------------------------------------------------*/


SDOCommStates SDOHandler::ReadSDO(uint16_t Idx, uint8_t SubIdx)
{
	switch(RxTxState)
	{
		case eIdle:
		case eRetry:
			//fill header
			RxRqMsg.u8Len = 7;
			RxRqMsg.u8Cmd = eSdoReadReq;
			RxRqMsg.Idx = Idx;
			RxRqMsg.SubIdx = SubIdx;

			if(hasMsgHandlerLocked = Handler->LockHandler())
			{
				//try to send the data
				if(Handler->SendMsg(Channel,(MCMsg *)&RxRqMsg))
				{
					RxTxState = eWaiting;

					BusyRetryCounter = 0;
					
					#if(DEBUG_SDO & DEBUG_RREQ)
					Serial.print("SDO RxReq ok ");
					Serial.print(Idx, HEX);
					Serial.println(" --> cWaiting");
					#endif

					//time our is handled by polling
					RequestSentAt = actTime;
					isTimerActive = true;
				}
				else
				{
					Handler->UnLockHandler();
					hasMsgHandlerLocked = false;

					//didn't work
					BusyRetryCounter++;
					if(BusyRetryCounter > BusyRetryMax)
					{
						RxTxState = eError;
						#if(DEBUG_SDO & DEBUG_ERROR)
						Serial.print("N ");
						Serial.print(Handler->GetNodeId(Channel),DEC);
						Serial.println(" SDO: RxReq failed --> eError");
						#endif
					}
					else
					{
						RxTxState = eRetry;
						#if(DEBUG_SDO & DEBUG_RREQ)
						Serial.print("N ");
						Serial.print(Handler->GetNodeId(Channel),DEC);
						Serial.println(" SDO: RxReq busy --> eRetry");
						#endif
					}
				}
			}
			break;
	}
	return RxTxState;
}

/*-------------------------------------------------------------
 * SDOCommStates WriteSDO(uint16_t Idx, uint8_t SubIdx,uint32_t *Data,uint8_t len)
 * Try to write a drive parameter identified by its Idx and SubIdx.
 * Additional parameters are the value itself and the size of the parameter in bytes.
 * Is using a step sequence of sendig a request, waiting for an answer
 * and giving it a retry if not sucessfull.
 * Reception fo the responses is via the OnRxHandler().
 * RxTxState is used to handle the steps and as the central feedback
 * WriteSDO will end up in eDone state to indicate the requested value
 * has been sent. Nedes to be reset explictily by calling ResetComState().
 * 
 * As any access to the MsgHandler WriteSDO will lock the Msghandler and
 * will only unlock it the actual service failed.
 * Successful servie will unlock in OnRxHandler().
 * 
 * 2020-11-18 AW Rev_A
 * 2021-04-22 AW Removed reference to timer service
 * -------------------------------------------------------------*/

SDOCommStates SDOHandler::WriteSDO(uint16_t Idx, uint8_t SubIdx,uint32_t *Data,uint8_t len)
{
	switch(RxTxState)
	{
		case eIdle:
		case eRetry:
		//fill header
			TxRqMsg.u8Len = 7 + len;
			TxRqMsg.u8Cmd = eSdoWriteReq;
			TxRqMsg.Idx = Idx;
			TxRqMsg.SubIdx = SubIdx;
			
			//copy the data into the message
			if(len == 1)
				*((uint8_t *)TxRqMsg.u8UserData) = *(uint8_t *)Data;
			else if(len == 2)
				*((uint16_t *)TxRqMsg.u8UserData) = *(uint16_t *)Data;
			else if(len == 4)
				*((uint32_t *)TxRqMsg.u8UserData) = *(uint32_t *)Data;
				
			if(hasMsgHandlerLocked = Handler->LockHandler())
			{				 
				//send the data
				if(Handler->SendMsg(Channel,(MCMsg *)&TxRqMsg))
				{
					RxTxState = eWaiting;
					BusyRetryCounter = 0;
					
					#if(DEBUG_SDO & DEBUG_WREQ)
					Serial.print("N ");
					Serial.print(Handler->GetNodeId(Channel),DEC);
					Serial.print(" SDO: TxReq ok ");
					Serial.println(Idx, HEX);
					#endif

					//handle time-out
					RequestSentAt = actTime;
					isTimerActive = true;
				}
				else
				{
					Handler->UnLockHandler();
					hasMsgHandlerLocked = false;

					BusyRetryCounter++;
					if(BusyRetryCounter > BusyRetryMax)
					{
						RxTxState = eError;
						#if(DEBUG_SDO & DEBUG_ERROR)
						Serial.print("N ");
						Serial.print(Handler->GetNodeId(Channel),DEC);
						Serial.println(" SDO: TxReq failed");
						#endif
					}
					else
					{
						RxTxState = eRetry;
						#if(DEBUG_SDO & DEBUG_WREQ)
						Serial.print("N ");
						Serial.print(Handler->GetNodeId(Channel),DEC);
						Serial.println(" SDO: TxReq busy");
						#endif
					}
				}
			}
			break;
	} //end of switch (RxTxState)
	return RxTxState;
}

/*-----------------------------------------------------
 * uint32_t GetObjValue()
 * Acutally read the last received object value.
 * It is returned as an int32_t and might have to be casted by a caller
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------*/

uint32_t SDOHandler::GetObjValue()
{
	uint32_t retValue = RxData;
	
	#if(DEBUG_SDO & DEBUG_RREQ)
	Serial.println("SDO Data fetched ");
	#endif

	if(RxTxState == eDone)
		RxTxState = eIdle;
		
	return retValue;	
}
//-------------------------------------------------------------------
//--- private calls ---

/*-------------------------------------------------------------------
 * void OnRxHandler(MCMsg *Msg)
 * The actual handler for any SDO services received by the MsgHandler
 * Checks wheter the received response belongs to any open
 * requenst and will switch these to eDone.
 * Other will transit to eError.
 * 
 * 2020-11-18 AW Rev_A
 * 2021-04-22 AW removed reference to timer
 * -----------------------------------------------------------------*/

void SDOHandler::OnRxHandler(MCMsg *Msg)
{
	MCMsgCommands Cmd = Msg->Hdr.u8Cmd;
	SDOMaxMsg *SDO = (SDOMaxMsg *)Msg;
	
	
	switch(Cmd)
	{
		case eSdoReadReq:
			//should contain the requested data
			if((RxRqMsg.Idx == SDO->Idx) && (RxRqMsg.SubIdx == SDO->SubIdx) && 
				((RxTxState == eWaiting) || (RxTxState == eRetry)) )
			{
				//correct answer
				//calc the length of the payload
				RxLen = (SDO->u8Len) - 7;
				
				//reset any active timer
				isTimerActive = false;
				
				#if(DEBUG_SDO  & DEBUG_RXMSG)
				Serial.print("SDO: Rx Idx ");
				Serial.print(SDO->Idx, HEX);
				Serial.print(" len: ");
				Serial.println(RxLen, DEC);
				#endif

				//cast the response to an unit32_t depending on the
				//lenght of the payload	
				if(RxLen == 1)
					//this is char
					RxData = (uint32_t)(*((uint8_t *)SDO->u8UserData));
				else if(RxLen == 2)
					//this is int
					RxData = (uint32_t)SDO->u8UserData[0] + (uint32_t)((SDO->u8UserData[1])<<8);					
				else if(RxLen == 4)
					//this is long data
					RxData =  ( ((uint32_t)(SDO->u8UserData[3]) << 24) + 
								((uint32_t)(SDO->u8UserData[2]) << 16) +
								((uint32_t)(SDO->u8UserData[1]) <<  8) +
								 (uint32_t)SDO->u8UserData[0]             );
				
				//switch transfer to eDone state and unlock the 
				//used MsgHandler	
				RxTxState = eDone;
				Handler->UnLockHandler();
				hasMsgHandlerLocked = false;

			}
			else
			{
				//wrong answer
				RxTxState = eError;
				
				#if(DEBUG_SDO & DEBUG_ERROR)
				Serial.print("SDO: Rx Error!");
				Serial.print(" Idx: ");
				Serial.print(SDO->Idx, HEX);
				Serial.print(".");				
				Serial.print(SDO->SubIdx, HEX);
				Serial.print(" >> ");
				Serial.println(RxTxState, DEC);
				#endif				
			}
			break;
		case eSdoWriteReq:
			//should be the response only
			if((TxRqMsg.Idx == SDO->Idx) && (TxRqMsg.SubIdx == SDO->SubIdx) && 
				((RxTxState == eWaiting) || (RxTxState == eRetry)))
			{
				//correct answer
				//swtich the state to the eDone and unlock the underlying 
				//MsgHandler
				RxTxState = eDone;
				Handler->UnLockHandler();
				hasMsgHandlerLocked = false;
				
				//reset any active timer
				isTimerActive = false;

				#if(DEBUG_SDO  & DEBUG_RXMSG)
				Serial.print("SDO: Tx Idx ");
				Serial.print(SDO->Idx, HEX);
				Serial.println(" ok");
				#endif
			}
			else
			{
				//wrong answer
				RxTxState = eError;	
							
				#if(DEBUG_SDO & DEBUG_ERROR)
				Serial.print("SDO: Tx Error!");
				Serial.print(" Idx: ");
				Serial.print(SDO->Idx, HEX);
				Serial.print(".");				
				Serial.print(SDO->SubIdx, HEX);
				Serial.print(" >> ");
				Serial.println(RxTxState, DEC);
				#endif				
			}
			break;
		default:
			//what's this? --> transit to eError
			RxTxState = eError;
				#if(DEBUG_SDO & DEBUG_ERROR)
				Serial.print("SDO: Rx wrong CMD Error!");
				#endif				
			break;
	}
}

/*----------------------------------------------------
 * void SetActTime(uint32_t time)
 * Soft-Update of the internal time in case of no HW timer being used.
 * use the updated time to check whether any of the responses
 * is timed out and call the OnTimeOut() if so.
 * 
 * 2020-11-18 AW Rev_A
 * 2021-04-22 AW removed reference to timer
 * -----------------------------------------------------------*/

void SDOHandler::SetActTime(uint32_t time)
{
	actTime = time;
	
	if((isTimerActive) && ((RequestSentAt + SDORespTimeOut) < actTime))	
	{	
		OnTimeOut();
		isTimerActive = false;
	}

}

/*----------------------------------------------------------
 * void OnTimeOut()
 * In case of a time-out either detected by the HW-tiemr or by the 
 * soft-timer swtich the communication either to a retry and increment
 * the retry counter or switch to final state eTimeout.
 * 
 * 2020-11-18 AW Rev_A
 * 2021-04-22 AW removed reference to timer
 * -------------------------------------------------------------*/

void SDOHandler::OnTimeOut()
{
	#if(DEBUG_SDO & DEBUG_TO)
	Serial.print("SDO: Timeout ");
	#endif
	
	
	if(TORetryCounter < TORetryMax)
	{
		RxTxState = eRetry;
		TORetryCounter++;
		
		if(hasMsgHandlerLocked)
		{
			Handler->UnLockHandler();
			hasMsgHandlerLocked = false;
		}


		#if(DEBUG_SDO & DEBUG_TO)
		Serial.println("retry");
		#endif
	}
	else
	{	
		RxTxState = eTimeout;
		TORetryCounter = 0;

		#if(DEBUG_SDO & DEBUG_TO)
		Serial.println("final");
		#endif
	}
}


