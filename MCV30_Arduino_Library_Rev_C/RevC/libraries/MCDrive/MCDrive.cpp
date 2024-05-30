/*---------------------------------------------------
 * MCNode.cpp
 * implements the class to handel NMT and access
 * to the CW/SW
 * MCNode does not implement the CiA 402 state machine
 *
 * 2020-05-24 AW Frame
 * 2021-04-21 removed reference to any timer service
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <MCDrive.h>

//--- local defines ---

#define DEBUG_RXMSG		0x0001
#define DEBUG_TO		0x0002
#define DEBUG_ERROR		0x0004
#define DEBUG_UPDATE	0x0010
#define DEBUG_MoveSpeed	0x0020
#define DEBUG_ENABLE	0x0040
#define DEBUG_DISABLE	0x0080
#define DEBUG_STOP		0x0100
#define DEBUG_MOVEPP	0x0200
#define DEBUG_HOME		0x0400
#define DEBUG_RWPARAM	0x0800
#define DEBUG_PULLSW    0x1000

#define DEBUG_DRIVE (DEBUG_TO | DEBUG_ERROR) 

//--- some definition to handle StatusWord and ControlWord of a drive ---

const uint16_t FSM402StatusMask = 0x6F;

const uint16_t FSM402_SwitchOnDisabled = 0x40;
const uint16_t FSM402_Ready2SwitchOn = 0x21;
const uint16_t FSM402_SwitchedOn = 0x23;
const uint16_t FSM402_Enabled = 0x27;
const uint16_t FSM402_Stopped = 0x07;
const uint16_t FSM402_FaultState = 0x08;
const uint16_t FSM402_FaultBit = 0x08;

const uint16_t StatusBit_PP_IsInPos = 0x0400;
const uint16_t StatusBit_PP_Ack = 0x1000;

const uint16_t StatusBit_PV_TargetReached = 0x0400;
const uint16_t StatusBit_PV_n0 = 0x1000;

const uint16_t StatusMask_Homing_Finished = 0x1400;

const uint16_t FSM402ControlMask = 0x000F;
const uint16_t FSM402_QSBit = 0x0004;
const uint16_t FSM402_HaltBit = 0x0100;

const uint16_t PP_StartBit 		= 0x0010;
const uint16_t PP_ImmediateBit 	= 0x0020; 
const uint16_t PP_RelativeBit	= 0x0040;
const uint16_t PP_ChangeOnSetP	= 0x0200;

//--- defines for the time-outs -------

const uint16_t MaxSWResponseDelay = 50;
const uint16_t PullSWCycleTime = 20;

//--- public functions ---

/*---------------------------------------------------------------------
 * MCDrive()
 * Not much to be dnone in the intializer
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

MCDrive::MCDrive()
{
	;
}

/*---------------------------------------------------------------------
 *void SetNodeId(uint8_t ThisNodeId)
 * Set the NodeId for this instance. Needs to be called before the
 * Msghandler can be registered
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

void MCDrive::SetNodeId(uint8_t ThisNodeId)
{
	ThisNode.SetNodeId(ThisNodeId);
}

/*---------------------------------------------------------------------
 * void Connect2MsgHandler(MsgHandler *ThisHandler)
 * In each system there is a single Msghandler but there can be
 * multiple drives. So the different instances of the MCNode and their
 * embeddd SDOHandlers need to be connected to the instance of the 
 * Msghandler by calling this method.
 * Also sets a default for this instances ComState
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

void MCDrive::Connect2MsgHandler(MsgHandler *ThisHandler)
{
	ThisNode.Connect2MsgHandler(ThisHandler);
	
	RxTxState = eMCIdle;
}

/*---------------------------------------------------------------------
 * void SetActTime(uint32_t time)
 * If no HW-timer is used this method needs to called cyclically
 * with the latest millis() value to check for any time-outs.
 * Does the same update for the MCNode and embedded SDOhandler.
 *  
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

void MCDrive::SetActTime(uint32_t time)
{
	actTime = time;
	ThisNode.SetActTime(time);
}

/*-------------------------------------------------------------------
 * CWCommStates CheckComState()
 * Check the ComState of the MCNode instance and update the ComState of then
 * Drive itself if requried.
 * Return the ComState
 *
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::CheckComState()
{
	//check  status will force the RxTxState into eCWError or eCWTimeout
	//if SDO failed. Otherwise we will simply read the actual RxTxState
	CWCommStates NodeState = ThisNode.CheckComState();
	if(NodeState == eCWTimeout)
		RxTxState = eMCTimeout;
	if(NodeState == eCWError)
		RxTxState = eMCError;
		
	return RxTxState;
}

/*---------------------------------------------------------------------
 * CWCommStates GetNodeState()
 * Return the ComState of the MCNode instance of this drive.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

CWCommStates MCDrive::GetNodeState()
{
	return ThisNode.CheckComState();
}

/*---------------------------------------------------------------------
 * SDOCommStates GetSDOState()
 * Return the ComState of teh SDOHandler instance of this drive.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

SDOCommStates MCDrive::GetSDOState()
{
	return ThisNode.CheckSDOState();
}

/*---------------------------------------------------------------------
 * uint16_t GetSW()
 * Return the last StatusWord received from the drive.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

uint16_t MCDrive::GetSW()
{
	return ThisNode.StatusWord;
}

/*---------------------------------------------------------------------
 * CWCommStates GetCWAccess()
 * Return the actual CWAccessState of this drive. this is for debugging
 * any of the step sequences where an access to the CW is used.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

CWCommStates MCDrive::GetCWAccess()
{
	return CWAccessState;
}

/*---------------------------------------------------------------------
 * uint8_t GetAccessStep()
 * Return the general AccessStep used in the step sequences for the
 * different actions. this is used to debug where we might be stuck.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

uint8_t MCDrive::GetAccessStep()
{
	return AccessStep;
}

/*---------------------------------------------------------------------
 * void ResetComState()
 * Reset the ComState of the Drive but do the same for teh MCNode instance
 * and reset the AccessSteps to 0 too.
 * Timeout and Retry counters are reset too to have a clean drive.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

void MCDrive::ResetComState()
{
	RxTxState = eMCIdle;
	ThisNode.ResetComState();
	SDOAccessState = eIdle;
	CWAccessState = eCWIdle;
	AccessStep = 0;

	TORetryCounter = 0;
	BusyRetryCounter = 0;
}

/*---------------------------------------------------------------------
 * void SetTORetryMax(uint8_t value)
 * Set a different value for the number of TO the drive can have before
 * the ComState will be eMCError. Default is set in the class definition.
 * 
 * This could be extended to the MCNode instance too but is nor so far.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

void MCDrive::SetTORetryMax(uint8_t value)
{
	ThisNode.SetTORetryMax(value);
}

/*---------------------------------------------------------------------
 * void SetBusyRetryMax(uint8_t value)
 * Set a different value for the number of subsequent retrys the drive 
 * can have before the ComState will be eMCError. 
 * Default is set in the class definition.
 * 
 * This could be extended to the MCNode instance too but is nor so far.
 *  * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

void MCDrive::SetBusyRetryMax(uint8_t value)
{
	ThisNode.SetBusyRetryMax(value);
}

//---------------------------------------------------------------------
// ----- real drive behavior ------------------------------------------

/*---------------------------------------------------------------------
 * DriveCommStates UpdateDriveStatus()
 * Update the local copy of the OpMode and the StatusWord.
 * Uses a step sequence based on AccessStep to create the sequence of
 * requests.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::UpdateDriveStatus()
{	
	switch(AccessStep)
	{
		case 0:
			switch(SDOAccessState)
			{
				case eDone:
					OpModeReported = (uint8_t)ThisNode.GetObjValue();
					AccessStep = 1;
					ThisNode.ResetSDOState();
					SDOAccessState = eIdle;
					
					#if(DEBUG_DRIVE & DEBUG_UPDATE)
					Serial.print("Drive: OpMode pulled ");
					Serial.println(OpModeReported, HEX);
					#endif
					break;
				case eIdle:
				case eRetry:
				case eWaiting:
					#if(DDEBUG_DRIVE & DEBUG_UPDATE)
					if(SDOAccessState == eIdle)
						Serial.print("Drive: OpMode Request ");
					else if(SDOAccessState == eRetry)
						Serial.print("Drive: OpMode Request Retry");
					#endif
					
					SDOAccessState = ThisNode.ReadSDO(0x6061, 0x00);
					RxTxState = eMCWaiting;
					break;				
			}
			break;
		case 1:
			switch(SDOAccessState)
			{
				case eDone:
					ThisNode.StatusWord = (uint16_t)ThisNode.GetObjValue();
					AccessStep = 0;
					ThisNode.ResetSDOState();
					SDOAccessState = eIdle;
					
					RxTxState = eMCDone;
					
					#if(DEBUG_DRIVE & DEBUG_UPDATE)
					Serial.print("Drive: SW pulled ");
					Serial.println(ThisNode.StatusWord, HEX);
					#endif
					break;
				case eIdle:
				case eRetry:
				case eWaiting:
					#if(DEBUG_DRIVE & DEBUG_UPDATE)
					if(SDOAccessState == eIdle)
						Serial.print("Drive: SW Request ");
					#endif
					
					SDOAccessState = ThisNode.ReadSDO(0x6041, 0x00);
					break;
			}
			break;
	}	
	//always check whether a communication is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * DriveCommStates UpdateActValues()
 * Update the local copy of the actual values
 * Uses a step sequence based on AccessStep to create the sequence of
 * requests.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2021-05-11 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::UpdateActValues()
{	
	switch(AccessStep)
	{
		case 0:
			//get actual postion
			switch(SDOAccessState)
			{
				case eDone:
					ActualPostion = (int32_t)ThisNode.GetObjValue();
					AccessStep = 1;
					ThisNode.ResetSDOState();
					SDOAccessState = eIdle;
					
					#if(DEBUG_DRIVE & DEBUG_ACTVAL)
					Serial.print("Drive: act position pulled ");
					Serial.println(ActualPostion, DEC);
					#endif
					break;
				case eIdle:
				case eRetry:
				case eWaiting:
					#if(DDEBUG_DRIVE & DEBUG_ACTVAL)
					if(SDOAccessState == eIdle)
						Serial.print("Drive: act position Request ");
					else if(SDOAccessState == eRetry)
						Serial.print("Drive: act position Retry");
					#endif
					
					SDOAccessState = ThisNode.ReadSDO(0x6064, 0x00);
					RxTxState = eMCWaiting;
					break;				
			}
			break;
		case 1:
			//get actual speed
			switch(SDOAccessState)
			{
				case eDone:
					ActualSpeed = (uint32_t)ThisNode.GetObjValue();
					AccessStep = 0;
					ThisNode.ResetSDOState();
					SDOAccessState = eIdle;
					
					RxTxState = eMCDone;
					
					#if(DEBUG_DRIVE & DEBUG_ACTVAL)
					Serial.print("Drive: act speed pulled ");
					Serial.println(ActualSpeed, DEC);
					#endif
					break;
				case eIdle:
				case eRetry:
				case eWaiting:
					#if(DEBUG_DRIVE & DEBUG_ACTVAL)
					if(SDOAccessState == eIdle)
						Serial.print("Drive: act speed Request ");
					#endif
					
					SDOAccessState = ThisNode.ReadSDO(0x606C, 0x00);
					break;
			}
			break;
	}	
	//always check whether a communication is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * int32_t GetActualPosition()
 * get a copy of the latest position
 * 
 * 2021-05-11 AW Done
 *--------------------------------------------------------------------*/

int32_t MCDrive::GetActualPosition()
{
	return ActualPostion;
}

/*---------------------------------------------------------------------
 * int32_t GetActualSpeed()
 * get a copy of the latest speed
 * 
 * 2021-05-11 AW Done
 *--------------------------------------------------------------------*/

int32_t MCDrive::GetActualSpeed()
{
	return ActualSpeed;
}

/*---------------------------------------------------------------------
 * DriveCommStates UpdateMotorTemp()
 * Update the local copy of the actual values - here winding temp
 * Uses a step sequence based on AccessStep to create the sequence of
 * requests.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2021-05-11 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::UpdateMotorTemp()
{	
	//get actual temperature
	switch(SDOAccessState)
	{
		case eDone:
			ActualMotorTemp = (int16_t)ThisNode.GetObjValue();
			ThisNode.ResetSDOState();
			SDOAccessState = eIdle;
			
			RxTxState = eMCDone;
			
			#if(DEBUG_DRIVE & DEBUG_ACTVAL)
			Serial.print("Drive: act motor temp pulled ");
			Serial.println(ActualMotorTemp, DEC);
			#endif
			break;
		case eIdle:
		case eRetry:
		case eWaiting:
			#if(DEBUG_DRIVE & DEBUG_ACTVAL)
			if(SDOAccessState == eIdle)
				Serial.print("Drive: act motor temp Request ");
			#endif
			
			SDOAccessState = ThisNode.ReadSDO(0x2326, 0x03);
			break;
	}
	//always check whether a communication is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * int16_t GetActualMotorTemp()
 * get a copy of the latest speed
 * 
 * 2021-05-11 AW Done
 *--------------------------------------------------------------------*/

int16_t MCDrive::GetActualMotorTemp()
{
	return ActualMotorTemp;
}

/*---------------------------------------------------------------------
 * DriveCommStates UpdateDriveErrors()
 * Update the local copy of the actual values - here the error register
 * Uses a step sequence based on AccessStep to create the sequence of
 * requests.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2021-05-11 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::UpdateDriveErrors()
{	
	//get actual temperature
	switch(SDOAccessState)
	{
		case eDone:
			ActualDriveErrors = (int16_t)ThisNode.GetObjValue();
			ThisNode.ResetSDOState();
			SDOAccessState = eIdle;
			
			RxTxState = eMCDone;
			
			#if(DEBUG_DRIVE & DEBUG_ACTVAL)
			Serial.print("Drive: act error state pulled ");
			Serial.println(ActualDriveErrors, HEX);
			#endif
			break;
		case eIdle:
		case eRetry:
		case eWaiting:
			#if(DEBUG_DRIVE & DEBUG_ACTVAL)
			if(SDOAccessState == eIdle)
				Serial.print("Drive: act drive errors Request ");
			#endif
			
			SDOAccessState = ThisNode.ReadSDO(0x2320, 0x00);
			break;
	}
	//always check whether a communication is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * uint16_t GetActualDriveErrors()
 * get a copy of the latest request to the error register
 * 
 * 2021-05-11 AW Done
 *--------------------------------------------------------------------*/

uint16_t MCDrive::GetActualDriveErrors()
{
	return ActualDriveErrors;
}

/*---------------------------------------------------------------------
 * DriveCommStates EnableDrive()
 * Enable the drive state machine.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::EnableDrive()
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;
	
	if((StatusWord & FSM402StatusMask) == FSM402_Enabled)
	{
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWDone))
		{
			ThisNode.ResetComState();
			CWAccessState = eCWIdle;
			RxTxState = eMCDone;
			
			#if(DEBUG_DRIVE & DEBUG_ENABLE)
			Serial.print("Drive: Enabled SW ");
			Serial.println(StatusWord, HEX);
			#endif
		}
		else
		{
			//CWAccess has to be finshed 
			CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);
		}
	}
	else
	{
		unsigned int newCW;

        if((StatusWord & FSM402StatusMask) == FSM402_Ready2SwitchOn) 
			newCW = (ControlWord & FSM402ControlMask) | 0x07;
		else if((StatusWord & FSM402StatusMask) == FSM402_SwitchedOn) 
			newCW = (ControlWord & FSM402ControlMask) | 0x0F;
		else if((StatusWord & FSM402StatusMask) == FSM402_Stopped) 
			newCW = (ControlWord & FSM402ControlMask) | 0x0F;
		else if((StatusWord & FSM402StatusMask) == FSM402_FaultState) 
			newCW = (ControlWord & FSM402ControlMask) | 0x80; 
		else
		    newCW = (ControlWord & FSM402ControlMask) | 0x06; 	
				
		//send a new request only, if that has not been done last time	
		#if(DEBUG_DRIVE & DEBUG_ENABLE)
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
		{
			Serial.print("Drive: Enable ");
			Serial.println(newCW, HEX);
		}
		#endif
			
		//would have to reflect the CWAccessState but that's done
		//inside the SendCW already
		CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);			
		RxTxState = eMCWaiting;
	}
	//always check whether a SDO is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * DriveCommStates DisableDrive()
 * Disable the drive state machine.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::DisableDrive()
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;


	if((StatusWord & FSM402StatusMask) == FSM402_SwitchOnDisabled)
	{
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWDone))
		{
			ThisNode.ResetComState();
			CWAccessState = eCWIdle;		
			RxTxState = eMCDone;
			
			#if(DEBUG_DRIVE & DEBUG_DISABLE)
			Serial.print("Drive: Disabled SW ");
			Serial.println(StatusWord, HEX);
			#endif
		}
		else
		{
			CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);
		}
		
	}
	else
	{
		uint16_t newCW = (ControlWord & ~FSM402ControlMask);
						
		//send a new request only, if that has not been done last time	
		#if(DEBUG_DRIVE & DEBUG_DISABLE)
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
		{
			Serial.print("Drive: Disable CW ");
			Serial.println(ControlWord, HEX);
		}
		#endif		
			
		//would have to reflect the CWAccessState but that's done
		//inside the SendCW already
		CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);
		RxTxState = eMCWaiting;
		
	}
	//always check whether a SDO is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * DriveCommStates StopDrive()
 * Stwich the drive state machine to QuickStop.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 *
 * 2020-11-22 AW untested
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::StopDrive()
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;

	if( ((StatusWord & FSM402StatusMask) == FSM402_Stopped) || ((StatusWord & FSM402StatusMask) == FSM402_SwitchOnDisabled) ) 
	{		
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWDone))
		{
			ThisNode.ResetComState();
			CWAccessState == eCWIdle;
			
			#if(DEBUG_DRIVE & DEBUG_STOP)
			Serial.print("Drive: Stopped SW ");
			Serial.println(StatusWord, HEX);
			#endif
		}
		else
		{
			CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);	
		}
	}
	else
	{
		//reset the QS bit
		uint16_t newCW = ControlWord & ~FSM402_QSBit;
					
		#if(DEBUG_DRIVE & DEBUG_STOP)
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
		{
			Serial.print("Disable ");
			Serial.println(ControlWord, HEX);
		}
		#endif
			
		//would have to reflect the CWAccessState but that's done
		//inside the SendCW already
		CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);
		RxTxState = eMCWaiting;
	}
	//always check whether a SDO is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * DriveCommStates WriteObject(uint16_t index, uint8_t Sub, int32_t value)
 * write whatever object
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be reset to eMCIdle after having registered the eMCDone
 *
 * 
 * 2021-05-11 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::WriteObject(uint16_t index, uint8_t sub, int32_t value, uint8_t size)
{
	if(SDOAccessState == eDone)
	{
		ThisNode.ResetComState();
		SDOAccessState = eIdle;
		RxTxState = eMCDone;
		
		#if(DEBUG_DRIVE & DEBUG_HOME)
		Serial.println("Drive: Parameter set ");
		#endif
	}
	else
	{
		#if(DEBUG_DRIVE & DEBUG_HOME)
		if(SDOAccessState == eIdle)
		{
			Serial.print("Drive: Write parameter");
			Serial.print(index,HEC);
			Serial.print(":");
			Serial.println(Sub);
		}			
		#endif
	
		SDOAccessState = ThisNode.WriteSDO(index, sub,(uint32_t *)&value,size);
		RxTxState = eMCWaiting;
	}
	//always check whether a SDO is stuck final 
	return CheckComState();				
}


/*---------------------------------------------------------------------
 * DriveCommStates SetOpMode(int8_t OpMode)
 * Set the requested OpMode by writing via SDO to 0x6060.00.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 *
 * Does not read the OpMode back 0x6061.00 but switches to mCDone as soon
 * as the wite access was successful.
 * 
 * Parameters are all supported OpModes of the drive.
 *
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::SetOpMode(int8_t OpMode)
{

	if(SDOAccessState == eDone)
	{
		ThisNode.ResetComState();
		SDOAccessState = eIdle;
		OpModeReported = OpModeRequested;
		RxTxState = eMCDone;
		
		#if(DEBUG_DRIVE & DEBUG_SetOpMode)
		Serial.println("Drive: OpMode set ");
		#endif
	}
	else
	{
		//skip the first access if not required
		if(OpModeReported == OpMode)
			RxTxState = eMCDone;
		else
		{
			OpModeRequested = OpMode;

			#if(DEBUG_DRIVE & DEBUG_SetOpMode)
			if(SDOAccessState == eIdle)
			{
				Serial.print("Drive: OpMode ");
				Seriel.print(OpMode, DEC);
				Serial.println(" Request");
			}
		
			#endif		
			SDOAccessState = ThisNode.WriteSDO(0x6060, 0x00,(uint32_t *)&OpModeRequested,1);
		}
		RxTxState = eMCWaiting;
	}					
	
	//always check whether a SDO is stuck final 
	return CheckComState();				
}

/*---------------------------------------------------------------------
 * DriveCommStates SetProfile(uint32_t ProfileACC, uint32_t ProfileDEC, uint32_t ProfileSpeed, int16_t ProfileType)
 * Set all the profile related parameters:
 * ProfileACC: acceleration in 1 ... 30000 1/s²
 * ProfileDEC: acceleration in 1 ... 30000 1/s²
 * ProfileSpeed in 1 ... given in whatever user units the speed in scaled
 *              by the FActorGroup. Default unit is 1/min (rpm)
 * ProfileType is 0: trapezoidal, 1: sin²
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone

 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::SetProfile(uint32_t ProfileACC, uint32_t ProfileDEC, uint32_t ProfileSpeed, int16_t ProfileType)
{
	
	switch(AccessStep)
	{
		case 0:
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				AccessStep = 1;
				
				#if(DEBUG_DRIVE & DEBUG_RWPARAM)
				Serial.println("Drive: ACC set ");
				#endif
			}
			else
			{
				#if(DDEBUG_DRIVE & DEBUG_RWPARAM)
				if(SDOAccessState == eIdle)
					Serial.print("Drive: Set ACC ");
				else if(SDOAccessState == eRetry)
					Serial.print("Drive: Set ACC Retry");
				#endif
				
				SDOAccessState = ThisNode.WriteSDO(0x6083, 0x00,(uint32_t *)&ProfileACC,4);
				RxTxState = eMCWaiting;
			}
			break;
		case 1:
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				AccessStep = 2;
				
				#if(DEBUG_DRIVE & DEBUG_RWPARAM)
				Serial.println("Drive: DEC set ");
				#endif
			}
			else
			{
				#if(DDEBUG_DRIVE & DEBUG_RWPARAM)
				if(SDOAccessState == eIdle)
					Serial.print("Drive: Set DEC ");
				else if(SDOAccessState == eRetry)
					Serial.print("Drive: Set DEC Retry");
				#endif
				
				SDOAccessState = ThisNode.WriteSDO(0x6084, 0x00,(uint32_t *)&ProfileDEC,4);
			}
			break;
		case 2:
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				AccessStep = 3;			
				
				#if(DEBUG_DRIVE & DEBUG_RWPARAM)
				Serial.println("Drive: Speed set ");
				#endif
			}
			else
			{
				#if(DDEBUG_DRIVE & DEBUG_RWPARAM)
				if(SDOAccessState == eIdle)
					Serial.print("Drive: Set Speed ");
				else if(SDOAccessState == eRetry)
					Serial.print("Drive: Set Speed Retry");
				#endif
				
				SDOAccessState = ThisNode.WriteSDO(0x6081, 0x00,(uint32_t *)&ProfileSpeed,4);
			}
			break;
		case 3:
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				AccessStep = 0;
				RxTxState = eMCDone;				
				
				#if(DEBUG_DRIVE & DEBUG_RWPARAM)
				Serial.println("Drive: P-Type set ");
				#endif
			}
			else
			{
				#if(DDEBUG_DRIVE & DEBUG_RWPARAM)
				if(SDOAccessState == eIdle)
					Serial.print("Drive: Set P-Type ");
				else if(SDOAccessState == eRetry)
					Serial.print("Drive: Set P-Type Retry");
				#endif
				
				SDOAccessState = ThisNode.WriteSDO(0x6086, 0x00,(uint32_t *)&ProfileSpeed,2);
			}
			break;
	}	//end of switch				

	//always check whether a SDO is stuck final 
	return CheckComState();				
}		

/*---------------------------------------------------------------------
 * DriveCommStates StartAbsMove(int32_t TargetPos, bool immeditate)
 * Switch the drive in PP mode and start pos-controlled move to the given 
 * absolute position based on the latest profile parameters.
 * Uses the internal MovePP to do so.
 * 
 * Parameters are
 * TargetPos: position in whatever the position has be sclaed by the
 *            FactorGroup. Default scaling is in position encoder increments
 * immeditate: 0: will wait for a preceding move to be finished
 *             1: start this move now
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 *
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::StartAbsMove(int32_t TargetPos, bool immeditate)
{
	return MovePP(TargetPos, immeditate, false);
}

/*---------------------------------------------------------------------
 * DriveCommStates StartRelMove(int32_t TargetPos, bool immeditate)
 * Switch the drive in PP mode and start pos-controlled move by the 
 * given distance based on the latest profile parameters.
 * Uses the internal MovePP to do so.
 * 
 * Parameters are
 * TargetPos: position in whatever the position has be sclaed by the
 *            FactorGroup. Default scaling is in position encoder increments
 * immeditate: 0: will wait for a preceding move to be finished
 *             1: start this move now
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::StartRelMove(int32_t TargetPos, bool immeditate)
{
	return MovePP(TargetPos, immeditate, true);
}

/*---------------------------------------------------------------------
 * DriveCommStates MoveAtSpeed(int32_t RefSpeed)
 * Switch the drive to PV mode and move at the given speed.
 * Parameter is:
 * RefSpeed: given in whatever user units the speed in scaled
 *              by the FactorGroup. Default unit is 1/min (rpm)
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::MoveAtSpeed(int32_t RefSpeed)
{
	
	switch(AccessStep)
	{
		case 0:
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				OpModeReported = 3;
				AccessStep = 1;			
				
				#if(DEBUG_DRIVE & DEBUG_MoveSpeed)
				Serial.println("Drive: OpMode set ");
				#endif
			}
			else
			{
				//skip the first access if not required
				if(OpModeReported == 3)
					AccessStep = 1;
				else
				{
					OpModeRequested = 3;

					#if(DEBUG_DRIVE & DEBUG_MoveSpeed)
					if(SDOAccessState == eIdle)
					{
						Serial.println("Drive: OpMode 3 Request");
					}
				
					#endif
				
					SDOAccessState = ThisNode.WriteSDO(0x6060, 0x00,(uint32_t *)&OpModeRequested,1);
				}
				RxTxState = eMCWaiting;
			}

			break;
		case 1:
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				AccessStep = 0;
				RxTxState = eMCDone;

				#if(DEBUG_DRIVE & DEBUG_MoveSpeed)
				Serial.println("Drive: TSpeed set");
				#endif
			}
			else 			
			{
				#if(DEBUG_DRIVE & DEBUG_MoveSpeed)
				if(SDOAccessState == eIdle)
				{
					Serial.print("Drive: Set TSpeed: ");
					Serial.println(RefSpeed, DEC);
				}
				#endif

				SDOAccessState = ThisNode.WriteSDO(0x60FF, 0x00,(uint32_t *)&RefSpeed,4);
			}

			break;
	}					
	//always check whether a SDO is stuck final 
	return CheckComState();				
}

/*---------------------------------------------------------------------
 * DriveCommStates ConfigureHoming(int8_t method)
 * Set the requested homing method by writing via SDO to 0x6098.00.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 *
 * Parameters are all supported homing modes of the drive.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::ConfigureHoming(int8_t method)
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;

	//set homing method
	if(SDOAccessState == eDone)
	{
		ThisNode.ResetComState();
		SDOAccessState = eIdle;
		RxTxState = eMCDone;
		
		#if(DEBUG_DRIVE & DEBUG_HOME)
		Serial.println("Drive: Homing set ");
		#endif
	}
	else
	{
		#if(DEBUG_DRIVE & DEBUG_HOME)
		if(SDOAccessState == eIdle)
		{
			Serial.println("Drive: Homing Set Mode");
		}			
		#endif
	
		SDOAccessState = ThisNode.WriteSDO(0x6098, 0x00,(uint32_t *)&method,1);
		RxTxState = eMCWaiting;
	}
	//always check whether a SDO is stuck final 
	return CheckComState();				
}

/*---------------------------------------------------------------------
 * DriveCommStates StartHoming()
 * Switch the drive to hmong mode and start the pre-configured homing
 * method.
 * Uses an internal step sequence based on AccessStep to do so.
 * No Parameters required.
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::StartHoming()
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;

	//at the very beginning AccessStep has to be == 0
	switch(AccessStep)
	{
		case 0:
			//ensure StartBit == 0
			//as soon as the CWAccessState == eCWDone we can proceed
			if(CWAccessState == eCWDone)
			{
				ThisNode.ResetComState();
				CWAccessState = eCWIdle;
				AccessStep = 1;

				#if(DEBUG_DRIVE & DEBUG_HOME)
				Serial.println("Drive: intial Force Bit4==0");
				#endif
			}
			else
			{
				uint16_t newCW = ControlWord & ~PP_StartBit;
				
				#if(DEBUG_DRIVE & DEBUG_HOME)
				if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
				{
					Serial.print("Drive: Reset Start Bit first: ");
					Serial.println(newCW, HEX);
				}
				#endif
					
				//no SW response required - 0 will avoid polling
				CWAccessState = ThisNode.SendCw(newCW,0);
				RxTxState = eMCWaiting;
			}
			break;
		case 1:
			//care for OpMode == 6
			OpModeRequested = 6;
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				AccessStep = 2;
				
				#if(DEBUG_DRIVE & DEBUG_HOME)
				Serial.println("Drive: OpMode set ");
				#endif
			}
			else
			{
				#if(DEBUG_DRIVE & DEBUG_HOME)
				if(SDOAccessState == eIdle)
				{
					Serial.println("Drive: OpMode 6 Request");
				}				
				#endif
			
				SDOAccessState = ThisNode.WriteSDO(0x6060, 0x00,(uint32_t *)&OpModeRequested,1);
			}
			break;
		case 2:
			switch(SDOAccessState)
			{
				case eDone:
					OpModeReported = (uint8_t)ThisNode.GetObjValue();
					ThisNode.ResetSDOState();
					SDOAccessState = eIdle;
					if(OpModeReported == OpModeRequested)
					{
						OpModeReported = OpModeRequested;
						AccessStep = 3;
						#if(DEBUG_DRIVE & DEBUG_HOME)
						Serial.print("Drive: OpMode pulled as expeted: ");
						Serial.println(OpModeReported, HEX);
						#endif
					}
					else
					{
						//try again
						AccessStep = 1;
						#if(DEBUG_DRIVE & DEBUG_HOME)
						Serial.print("Drive: OpMode pulled unexpected: ");
						Serial.println(OpModeReported, HEX);
						#endif
					}
						
					break;
				case eIdle:
				case eRetry:
				case eWaiting:
					#if(DDEBUG_DRIVE & DEBUG_HOME)
					if(SDOAccessState == eIdle)
						Serial.print("Drive: OpMode Request ");
					else if(SDOAccessState == eRetry)
						Serial.print("Drive: OpMode Request Retry");
					#endif
					
					SDOAccessState = ThisNode.ReadSDO(0x6061, 0x00);
					break;				
			}
			break;
		case 3:
			//set StartBit		
			if(CWAccessState == eCWDone)
			{
				ThisNode.ResetComState();
				CWAccessState = eCWIdle;
				AccessStep = 4;

				#if(DEBUG_DRIVE & DEBUG_HOME)
				Serial.println("Drive: Start Bit set");
				#endif
			}
			else
			{
				uint16_t newCW = ControlWord | PP_StartBit;
								
				#if(DEBUG_DRIVE & DEBUG_HOME)
				if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
				{
					Serial.print("Drive: Set Start Bit: ");
					Serial.println(newCW, HEX);
				}
				#endif
					
				//no SW response required - 0 will avoid polling
				CWAccessState = ThisNode.SendCw(newCW,0);			
			}
			break;
		case 4:
			//reset StartBit again
			if(CWAccessState == eCWDone)
			{
				ThisNode.ResetComState();
				CWAccessState = eCWIdle;
				AccessStep = 5;

				#if(DEBUG_DRIVE & DEBUG_HOME)
				Serial.println("Drive: reset Start-bit again Bit4==0");
				#endif
			}
			else
			{
				uint16_t newCW = ControlWord & ~PP_StartBit;
								
				#if(DEBUG_DRIVE & DEBUG_HOME)
				if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
				{
					Serial.print("Drive: Reset Start Bit again: ");
					Serial.println(newCW, HEX);
				}
				#endif
					
				//no SW response required - 0 will avoid polling
				CWAccessState = ThisNode.SendCw(newCW,0);			
			}
			break;
		case 5:
			//we are done and wait for the calling sequence to reset the ComState
			RxTxState = eMCDone;
			break;
			
	}
	//always check whether a SDO is stuck final 
	return CheckComState();				
}

/*---------------------------------------------------------------------
 * DriveCommStates IsInPos()
 * Check the StatusWord of the drive and test it for the target reached
 * bit being set. Will update the Statusword cyclically using the internal
 * Wait4Status.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::IsInPos()
{				
	return Wait4Status(StatusBit_PP_IsInPos,PullSWCycleTime);	
}

/*---------------------------------------------------------------------
 * DriveCommStates IsHomingFinished()
 * Check the StatusWord of the drive and test it for pattern indicating a 
 * successfully completed homing sequence being set. 
 * Will update the Statusword cyclically using the internal Wait4Status.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 *
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::IsHomingFinished()
{	
	return Wait4Status(StatusMask_Homing_Finished,PullSWCycleTime);
}

/*---------------------------------------------------------------------
 * bool IsLive()
 * Check whether a boot Msg of the drive has been received
 * Please note: in net-mode of multiple drives no boot messages
 * will be sent at all.
 *
 * 2020-11-22 AW untested
 *--------------------------------------------------------------------*/

bool MCDrive::IsLive()
{
	return ThisNode.IsLive();
}
		
/*---------------------------------------------------------------------
 * uint16_t GetLastError()
 * Read the last EMCY code. EMCY service is not available in net mode.
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

uint16_t MCDrive::GetLastError()
{
	return ThisNode.GetLastError();
}

//-------------------------------------------------------------------
//---- private functions --------
//-------------------------------------------------------------------

/*---------------------------------------------------------------------
 * void OnTimeOut()
 * Handler for whatever TO might be implemented during complex actions
 * As of now not used.
 * 
 * 2020-11-22 AW untested
 *--------------------------------------------------------------------*/

void MCDrive::OnTimeOut()
{
	#if(DEBUG_DRIVE & DEBUG_TO)
	Serial.print("Drive: Timeout ");
	#endif
}

/*---------------------------------------------------------------------
 * DriveCommStates Wait4Status(uint16_t mask, uint16_t CycleTime)
 * Check the StatusWord of teh drive for the given pattern.
 * Will only return with eMCDone when the pattern is found. Will update
 * the StatusWord cyclically.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::Wait4Status(uint16_t mask, uint16_t CycleTime)
{	
	if((ThisNode.StatusWord & mask) == mask)
	{
		if( ((CWAccessState == eCWIdle) || (CWAccessState == eCWDone)) &&
			(SDOAccessState == eIdle))
		{
			ThisNode.ResetComState();
			CWAccessState = eCWIdle;
			RxTxState = eMCDone;
				
			#if(DEBUG_DRIVE & DEBUG_PULLSW)
			Serial.print("Drive: mask ");
			Serial.print(mask, HEX);
			Serial.println(" found");
			#endif
		}
		else
		{
			CWAccessState = ThisNode.PullSW(CycleTime);			
		}
	}
	else
	{						
		#if(DEBUG_DRIVE & DEBUG_PULLSW)
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
		{
			Serial.println("Drive: pull SW: ");
		}
		#endif
			
		CWAccessState = ThisNode.PullSW(CycleTime);
		RxTxState = eMCWaiting;			
	}
	//always check whether a SDO is stuck final 
	return CheckComState();					
}

/*---------------------------------------------------------------------
 * DriveCommStates MovePP(int32_t TargetPos, bool immeditate, bool relative)
 * Internal function to start an either absolute or relative move in PP mode.
 * Switches the drive to PP and handles the immediate bit.
 * Uses an internal step sequence based on AccessStep.
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 *
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCDrive::MovePP(int32_t TargetPos, bool immeditate, bool relative)
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;

	//at the very beginning AccessStep has to be == 0
	switch(AccessStep)
	{
		case 0:
			//care for OpMode == 1
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				OpModeReported = 1;
				AccessStep = 1;
				
				#if(DEBUG_DRIVE & DEBUG_MOVEPP)
				Serial.println("Drive: OpMode set ");
				#endif
			}
			else
			{
				if(OpModeReported == 1)
					AccessStep = 1;
				else
				{
					OpModeRequested = 1;

					#if(DEBUG_DRIVE & DEBUG_MOVEPP)
					if(SDOAccessState == eIdle)
					{
						Serial.println("Drive: OpMode 1 Request");
					}
				
					#endif
				
					SDOAccessState = ThisNode.WriteSDO(0x6060, 0x00,(uint32_t *)&OpModeRequested,1);
				}
				RxTxState = eMCWaiting;
			}
			break;
		case 1:
			//ensure StartBit == 0
			//of course the simpler way as this is done in StartHoming will do
			//in most cases. Here the readyness of the drive is checked too,
			if( ((ControlWord & PP_StartBit) == 0) &&
				((StatusWord & StatusBit_PP_Ack) == 0) )
			{
				if( ((CWAccessState == eCWIdle) || (CWAccessState == eCWDone)) &&
					(SDOAccessState == eIdle))
				{
					ThisNode.ResetComState();
					CWAccessState = eCWIdle;
					AccessStep = 2;

					#if(DEBUG_DRIVE & DEBUG_MOVEPP)
					Serial.println("Drive: intial Force Bit4==0");
					#endif
				}
				else
				{
					CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);			
				}

			}
			else
			{
				uint16_t newCW = ControlWord & ~PP_StartBit;
								
				#if(DEBUG_DRIVE & DEBUG_MOVEPP)
				if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
				{
					Serial.print("Drive: Reset Start Bit first: ");
					Serial.println(newCW, HEX);
				}
				#endif
					
				CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);			
			}
			break;
		case 2:
			//set the TPos
			if(SDOAccessState == eDone)
			{
				ThisNode.ResetComState();
				SDOAccessState = eIdle;
				AccessStep = 3;
				
				#if(DEBUG_DRIVE & DEBUG_MOVEPP)
				Serial.println("Drive: TPos set ");
				#endif
			}
			else
			{
				#if(DEBUG_DRIVE & DEBUG_MOVEPP)
				if(SDOAccessState == eIdle)
				{
					Serial.print("Drive: TPos Request ");
					Serial.println(TargetPos, DEC);
				}
				#endif
				
				SDOAccessState = ThisNode.WriteSDO(0x607A, 0x00,(uint32_t *)&TargetPos,4);
			}
			break;
		case 3:
			//set StartBit				
			//of course the simpler way as this is done in StartHoming will do
			//in most cases. Here the drive rreally starting to move is checeked too.
			if((StatusWord & StatusBit_PP_Ack) == StatusBit_PP_Ack)
			{
				if( ((CWAccessState == eCWIdle) || (CWAccessState == eCWDone)) &&
					(SDOAccessState == eIdle))
				{
					ThisNode.ResetComState();
					CWAccessState = eCWIdle;
					AccessStep = 4;
					
					#if(DEBUG_DRIVE & DEBUG_MOVEPP)
					Serial.println("Drive: PP Started");
					#endif
				}
				else
				{
					CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);			
				}
			}
			else
			{
				uint16_t newCW = ControlWord | PP_StartBit;
				
				if(immeditate)
					newCW |= PP_ImmediateBit;
				if(relative)
					newCW |= PP_RelativeBit;
									
				#if(DEBUG_DRIVE & DEBUG_MOVEPP)
				if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
				{
					Serial.print("Drive: Set Start Bit ");
					Serial.println(newCW, HEX);
				}
				#endif

				CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);
			}
			break;
		case 4:
			//reset StartBit again
			//of course the simpler way as this is done in StartHoming will do
			//in most cases. Here the readyness of the drive is checked too,
			if((StatusWord & StatusBit_PP_Ack) == 0)
			{ 
				if( ((CWAccessState == eCWIdle) || (CWAccessState == eCWDone)) &&
					(SDOAccessState == eIdle))
				{
					ThisNode.ResetComState();
					CWAccessState = eCWIdle;
					RxTxState = eMCDone;
					AccessStep = 0;
				
					#if(DEBUG_DRIVE & DEBUG_MOVEPP)
					Serial.println("Node PP Start Done");
					#endif
				}
				else
				{
					CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);			
				}
			}
			else
			{
				uint16_t newCW = ControlWord & ~(PP_StartBit | PP_ImmediateBit | PP_RelativeBit);
								
				#if(DEBUG_DRIVE & DEBUG_MOVEPP)
				if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
				{
					Serial.print("Drive: Reset Start Bit again ");
					Serial.println(newCW, HEX);
				}
				#endif

				CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);			
			}
			break;			
	}
	//always check whether a SDO is stuck final 
	return CheckComState();				
}
