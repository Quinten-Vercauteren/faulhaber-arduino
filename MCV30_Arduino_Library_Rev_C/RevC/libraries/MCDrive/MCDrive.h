#ifndef MCDRIVE_H
#define MCDRIVE_H

/*--------------------------------------------------------------
 * class MCDrive
 * offers high level commands to interact with a servo drive
 *
 * 2020-07-17 AW Frame
 * 2021-04-21 removed reference to any timer service
 *
 *-------------------------------------------------------------*/
 
//--- inlcudes ----
 
#include <MCNode.h>
#include <stdint.h>

//--- service define ---


typedef enum DriveCommStates {
	eMCIdle,
	eMCWaiting,
	eMCBusy,
	eMCDone,
	eMCError,
	eMCTimeout,
}
 DriveCommStates;

	
class MCDrive {
	public:
		MCDrive();
		void Connect2MsgHandler(MsgHandler *);
		void SetNodeId(uint8_t);
		void SetActTime(uint32_t);

		DriveCommStates CheckComState();
		void ResetComState(); 
		void SetTORetryMax(uint8_t);
		void SetBusyRetryMax(uint8_t);
		
		DriveCommStates UpdateDriveStatus();		
		DriveCommStates SendReset();

		DriveCommStates EnableDrive();
		DriveCommStates DisableDrive();
		DriveCommStates StopDrive();

		DriveCommStates WriteObject(uint16_t, uint8_t, int32_t, uint8_t);
		DriveCommStates UpdateActValues();
		int32_t GetActualPosition();
		int32_t GetActualSpeed();
		DriveCommStates UpdateMotorTemp();
		int16_t GetActualMotorTemp();
		DriveCommStates UpdateDriveErrors();
		uint16_t GetActualDriveErrors();

		DriveCommStates SetOpMode(int8_t);
		DriveCommStates SetProfile(uint32_t, uint32_t, uint32_t, int16_t);		
		
		DriveCommStates StartAbsMove(int32_t, bool);
		DriveCommStates StartRelMove(int32_t, bool);
		DriveCommStates ConfigureHoming(int8_t);
		DriveCommStates StartHoming();
		DriveCommStates MoveAtSpeed(int32_t);
		DriveCommStates IsInPos();
		DriveCommStates IsHomingFinished();
		
		CWCommStates GetNodeState();
		SDOCommStates GetSDOState();
		
		uint8_t GetAccessStep();
		
		uint16_t GetSW();
		CWCommStates GetCWAccess();
		
		bool IsLive();
		uint16_t GetLastError();
		
		//hander to be registered at the OsTimer
		static void OnTimeOutCb(void *p) {
			((MCDrive *)p)->OnTimeOut();
		};
	
		MCNode ThisNode;

	private:
		void OnTimeOut();
		DriveCommStates Wait4Status(uint16_t, uint16_t);
		DriveCommStates MovePP(int32_t,bool, bool);
		
		DriveCommStates RxTxState = eMCIdle;
		
		uint8_t AccessStep = 0;
		
		int8_t OpModeRequested;
		int8_t OpModeReported;	
		
		int32_t ActualPostion = 0;
		int32_t ActualSpeed = 0;
		int16_t ActualMotorTemp = 22;	
		uint16_t ActualDriveErrors = 0;
		
		SDOCommStates SDOAccessState = eIdle;
		CWCommStates CWAccessState = eCWIdle;

		uint8_t TORetryCounter = 0;
		uint8_t TORetryMax = 1;
		uint8_t BusyRetryCounter = 0;
		uint8_t BusyRetryMax = 1;
		
		bool isTimerActive = false;

		uint32_t actTime;

		bool isLive = false;
};

#endif
