
//--- defines -----------------------------

typedef struct DriveParameters {
int16_t DriveId;
uint16_t driveStep;
uint32_t stepTime;
uint32_t incrementTime;
uint32_t actAcc;
uint32_t actDec;
uint32_t actSpeed;

uint32_t deltaSpeed;
uint32_t deltaAccDec;

int8_t DriveHomingMethod;
} DriveParameters;

//--- control of the behavior -------------

#define UpdateDriveA 1
#define UpdateDriveB 1
#define UpdateDriveC 1
#define UpdateDriveD 1

#define DEBUG_A 0
#define DEBUG_B 0
#define DEBUG_C 0

#define RESTART_NODES 1

//--- includes -----------------------------

#include <MsgHandler.h>
#include <MCDrive.h>
#include <stdint.h>

//--- globals -----------------------------

const uint32_t maxAcc = 2500;
const uint32_t minAccDec = 250;
const uint32_t maxDec = 2500;
const uint32_t maxSpeed = 4000;
const uint32_t minSpeed = 500;
const uint32_t StatusCycle = 10000;  //every 10 secs

uint32_t LastStatusUpdateTime;


//---- The Msghandler----------------------

MsgHandler MCMsgHandler;

//---- Drive A -----------------------------

MCDrive Drive_A;
DriveParameters Drive_A_Param;

//---- Drive B -----------------------------

MCDrive Drive_B;
DriveParameters Drive_B_Param;

//---- Drive C -----------------------------

MCDrive Drive_C;
DriveParameters Drive_C_Param;

//---- Drive D -----------------------------

MCDrive Drive_D;
DriveParameters Drive_D_Param;

void setDriveDefaults(DriveParameters *drive, int16_t Id, int8_t homing)
{
  drive->DriveId =Id;
  drive->driveStep = 0;
  drive->stepTime;
  drive->incrementTime;
  drive->actAcc = maxAcc;
  drive->actDec = maxDec;
  drive->actSpeed = minSpeed;

  drive->deltaSpeed = minSpeed;
  drive->deltaAccDec = minAccDec;
  drive->DriveHomingMethod = homing;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  // Debug Port
  Serial.begin(500000);

  //start the MSG-Handler
  MCMsgHandler.Open(115200);

  //init drive A
  setDriveDefaults(&Drive_A_Param,4,33);
  Drive_A.SetNodeId(Drive_A_Param.DriveId);
  Drive_A.Connect2MsgHandler(&MCMsgHandler);

  //init drive B
  setDriveDefaults(&Drive_B_Param,3,33);
  Drive_B.SetNodeId(Drive_B_Param.DriveId);
  Drive_B.Connect2MsgHandler(&MCMsgHandler);

  //init drive C
  setDriveDefaults(&Drive_C_Param,2,33);
  Drive_C.SetNodeId(Drive_C_Param.DriveId);
  Drive_C.Connect2MsgHandler(&MCMsgHandler);

  //init drive D
  setDriveDefaults(&Drive_D_Param,1,33);
  Drive_D.SetNodeId(Drive_D_Param.DriveId);
  Drive_D.Connect2MsgHandler(&MCMsgHandler);

  LastStatusUpdateTime = millis();
}


void loop() {
   // put your main code here, to run repeatedly:
   DriveCommStates NodeState, StateA, StateB, StateC, StateD;
   uint32_t currentMillis = millis();
   
   Drive_A.SetActTime(currentMillis);
   Drive_B.SetActTime(currentMillis);
   Drive_C.SetActTime(currentMillis);
   Drive_D.SetActTime(currentMillis);
   
   MCMsgHandler.Update(currentMillis); 

   #if UpdateDriveA
   //operate Drive A
   switch(Drive_A_Param.driveStep)
   {
      case 0:
        //first get a copy of the drive status
        if((Drive_A.UpdateDriveStatus()) == eMCDone)
        {
          Drive_A_Param.driveStep = 1;
          Drive_A.ResetComState();
          #if DEBUG_A
          Serial.println("Main: A -->1");
          #endif
        }
        break;
      case 1:
        //disable the drive first
        if((Drive_A.DisableDrive()) == eMCDone)
        {
          Drive_A_Param.driveStep = 2;
          Drive_A.ResetComState();
          #if DEBUG_A
          Serial.println("Main: A -->2");
          #endif
        }
        break;
      case 2:
        //enable next
        if((Drive_A.EnableDrive()) == eMCDone)
        {
          Drive_A_Param.driveStep = 3;
          Drive_A.ResetComState();
          #if DEBUG_A
          Serial.println("Main: A -->3: Config Homing");
          #endif
        }
        break;
      case 3:
        //config homing
        if((Drive_A.ConfigureHoming(Drive_A_Param.DriveHomingMethod)) == eMCDone)
        {
          Drive_A_Param.driveStep = 4;
          Drive_A.ResetComState();
          #if DEBUG_A
          Serial.println("Main: A -->4: Start Homing"); 
          #endif         
        }
        break;
      case 4:
        //start homing
        if((Drive_A.StartHoming()) == eMCDone)
        {
          Drive_A_Param.driveStep = 5;
          Drive_A.ResetComState();
          #if DEBUG_A
          Serial.println("Main: A -->5: Wait 4 Homing"); 
          #endif        
        }
        break;
      case 5:
        //wait for homing done
        if(Drive_A.IsHomingFinished() == eMCDone)
        {
          Drive_A_Param.driveStep = 6;
          Drive_A.ResetComState();
          #if DEBUG_A
          Serial.println("Main: A -->6: PV100"); 
          #endif        
        }     
        break; 
      case 6:
        //move at speed
        if((Drive_A.MoveAtSpeed(100)) == eMCDone)
        {
          Drive_A_Param.driveStep = 7;
          Drive_A.ResetComState();
          Drive_A_Param.stepTime = currentMillis;
          Drive_A_Param.incrementTime = currentMillis;
          #if DEBUG_A
          Serial.println("Main: A -->7");
          #endif
        }
        break;
      case 7:
        //wait some time
        if(currentMillis > (Drive_A_Param.stepTime + 2000))
        {
          Drive_A_Param.driveStep = 8;
          #if DEBUG_A
          Serial.println("Main: A -->8: PV-100");
          #endif
        }
        else if(currentMillis > (Drive_A_Param.incrementTime + 100))
        {
          #if DEBUG_A
          Serial.print(".");
          #endif
          Drive_A_Param.incrementTime = currentMillis;
        }
        break;
      case 8:
        //move at speed
        if((Drive_A.MoveAtSpeed(-100)) == eMCDone)
        {
          Drive_A_Param.driveStep = 9;
          Drive_A.ResetComState();
          Drive_A_Param.stepTime = currentMillis;
          Drive_A_Param.incrementTime = currentMillis;
          #if DEBUG_A
          Serial.println("Main: A -->9");
          #endif
        }
        break;
      case 9:
        //wait some time
        if(currentMillis > (Drive_A_Param.stepTime + 2000))
        {
          Drive_A_Param.driveStep = 10;
          #if DEBUG_A
          Serial.println("Main: A -->10: PP@50000");
          #endif
        }
        else if(currentMillis > (Drive_A_Param.incrementTime + 100))
        {
          #if DEBUG_A
          Serial.print(".");
          #endif
          Drive_A_Param.incrementTime = currentMillis;
        }
        break;
       case 10:
         //move to 0
         if((Drive_A.StartAbsMove(50000,false)) == eMCDone)
         {
           Drive_A_Param.driveStep = 11;
           Drive_A.ResetComState();
           #if DEBUG_A
           Serial.println("Main: A -->11");
           #endif
         }
        break;
       case 11:
         //wait for pos
         if(Drive_A.IsInPos() == eMCDone)
         {
           Drive_A_Param.driveStep = 12;
           Drive_A.ResetComState();
           #if DEBUG_A
           Serial.println("Main: A -->12: PP@0");
           #endif
         }
         break;
       case 12:
         //move to 0
         if((Drive_A.StartAbsMove(0,false)) == eMCDone)
         {
           Drive_A_Param.driveStep = 13;
           Drive_A.ResetComState();
           #if DEBUG_A
           Serial.println("Main: A -->13");
           #endif
         }
         break;
       case 13:
         if(Drive_A.IsInPos() == eMCDone)
         {
           Drive_A_Param.driveStep = 14;
           Drive_A.ResetComState();
           #if DEBUG_A
           Serial.println("Main: A -->14");
           #endif
         }
         break;
       case 14:
         if((Drive_A.SetProfile(Drive_A_Param.actAcc,Drive_A_Param.actDec,Drive_A_Param.actSpeed,0)) == eMCDone)
         {
           Drive_A_Param.driveStep = 1;
           Drive_A_Param.actSpeed += Drive_A_Param.deltaSpeed;
           if((Drive_A_Param.actSpeed <= minSpeed) || (Drive_A_Param.actSpeed >= maxSpeed))
             Drive_A_Param.deltaSpeed = Drive_A_Param.deltaSpeed * (-1);
          
           #if DEBUG_A
           Serial.print("Main: A Loop -->1 @");
           Serial.println(Drive_A_Param.actSpeed, DEC);
           #endif
        }
        break;
   }
   #endif

   #if UpdateDriveB
   //operate Drive B
   switch(Drive_B_Param.driveStep)
   {
      case 0:
        //first get a copy of the drive status
        if((Drive_B.UpdateDriveStatus()) == eMCDone)
        {
          Drive_B_Param.driveStep = 1;
          Drive_B.ResetComState();
          #if DEBUG_B
          Serial.println("Main: B -->1");
          #endif
        }
        break;
      case 1:
        //disable the drive first
        if((Drive_B.DisableDrive()) == eMCDone)
        {
          Drive_B_Param.driveStep = 2;
          Drive_B.ResetComState();
          #if DEBUG_B
          Serial.println("Main: B -->2");
          #endif
        }
        break;
      case 2:
        //enable next
        if((Drive_B.EnableDrive()) == eMCDone)
        {
          Drive_B_Param.driveStep = 3;
          Drive_B.ResetComState();
          #if DEBUG_B
          Serial.println("Main: B -->3: Config Homing");
          #endif
        }
        break;
      case 3:
        //config homing
        if((Drive_B.ConfigureHoming(Drive_B_Param.DriveHomingMethod)) == eMCDone)
        {
          Drive_B_Param.driveStep = 4;
          Drive_B.ResetComState();
          #if DEBUG_B
          Serial.println("Main: B -->4: Start Homing"); 
          #endif         
        }
        break;
      case 4:
        //start homing
        if((Drive_B.StartHoming()) == eMCDone)
        {
          Drive_B_Param.driveStep = 5;
          Drive_B.ResetComState();
          #if DEBUG_B
          Serial.println("Main: B -->5: Wait 4 Homing"); 
          #endif        
        }
        break;
      case 5:
        //wait for homing done
        if(Drive_B.IsHomingFinished() == eMCDone)
        {
          Drive_B_Param.driveStep = 6;
          Drive_B.ResetComState();
          #if DEBUG_B
          Serial.println("Main: B -->6: PV100"); 
          #endif        
        }     
        break; 
      case 6:
        //move at speed
        if((Drive_B.MoveAtSpeed(100)) == eMCDone)
        {
          Drive_B_Param.driveStep = 7;
          Drive_B.ResetComState();
          Drive_B_Param.stepTime = currentMillis;
          Drive_B_Param.incrementTime = currentMillis;
          #if DEBUG_B
          Serial.println("Main: B -->7");
          #endif
        }
        break;
      case 7:
        //wait some time
        if(currentMillis > (Drive_B_Param.stepTime + 2000))
        {
          Drive_B_Param.driveStep = 8;
          #if DEBUG_B
          Serial.println("Main: B -->8: PV-100");
          #endif
        }
        else if(currentMillis > (Drive_B_Param.incrementTime + 100))
        {
          #if DEBUG_B
          Serial.print(".");
          #endif
          Drive_B_Param.incrementTime = currentMillis;
        }
        break;
      case 8:
        //move at speed
        if((Drive_B.MoveAtSpeed(-100)) == eMCDone)
        {
          Drive_B_Param.driveStep = 9;
          Drive_B.ResetComState();
          Drive_B_Param.stepTime = currentMillis;
          Drive_B_Param.incrementTime = currentMillis;
          #if DEBUG_B
          Serial.println("Main: B -->9");
          #endif
        }
        break;
      case 9:
        //wait some time
        if(currentMillis > (Drive_B_Param.stepTime + 2000))
        {
          Drive_B_Param.driveStep = 10;
          #if DEBUG_B
          Serial.println("Main: B -->10: PP@50000");
          #endif
        }
        else if(currentMillis > (Drive_B_Param.incrementTime + 100))
        {
          #if DEBUG_B
          Serial.print(".");
          #endif
          Drive_B_Param.incrementTime = currentMillis;
        }
        break;
       case 10:
         //move to 0
         if((Drive_B.StartAbsMove(50000,false)) == eMCDone)
         {
           Drive_B_Param.driveStep = 11;
           Drive_B.ResetComState();
           #if DEBUG_B
           Serial.println("Main: B -->11");
           #endif
         }
        break;
       case 11:
         //wait for pos
         if(Drive_B.IsInPos() == eMCDone)
         {
           Drive_B_Param.driveStep = 12;
           Drive_B.ResetComState();
           #if DEBUG_B
           Serial.println("Main: B -->12: PP@0");
           #endif
         }
         break;
       case 12:
         //move to 0
         if((Drive_B.StartAbsMove(0,false)) == eMCDone)
         {
           Drive_B_Param.driveStep = 13;
           Drive_B.ResetComState();
           #if DEBUG_B
           Serial.println("Main: B -->13");
           #endif
         }
         break;
       case 13:
         if(Drive_B.IsInPos() == eMCDone)
         {
           Drive_B_Param.driveStep = 14;
           Drive_B.ResetComState();
           #if DEBUG_B
           Serial.println("Main: B -->14");
           #endif
         }
         break;
       case 14:
         if((Drive_B.SetProfile(Drive_B_Param.actAcc,Drive_B_Param.actDec,Drive_B_Param.actSpeed,0)) == eMCDone)
         {
           Drive_B_Param.driveStep = 1;
           Drive_B_Param.actSpeed += Drive_B_Param.deltaSpeed;
           if((Drive_B_Param.actSpeed <= minSpeed) || (Drive_B_Param.actSpeed >= maxSpeed))
             Drive_B_Param.deltaSpeed = Drive_B_Param.deltaSpeed * (-1);
          
           #if DEBUG_B
           Serial.print("Main: B Loop -->1 @");
           Serial.println(Drive_B_Param.actSpeed, DEC);
           #endif
        }
        break;
   }
   #endif

   #if UpdateDriveC
   //operate Drive C
   switch(Drive_C_Param.driveStep)
   {
      case 0:
        //first get a copy of the drive status
        if((Drive_C.UpdateDriveStatus()) == eMCDone)
        {
          Drive_C_Param.driveStep = 1;
          Drive_C.ResetComState();
          #if DEBUG_C
          Serial.println("Main: C -->1");
          #endif
        }
        break;
      case 1:
        //disable the drive first
        if((Drive_C.DisableDrive()) == eMCDone)
        {
          Drive_C_Param.driveStep = 2;
          Drive_C.ResetComState();
          #if DEBUG_C
          Serial.println("Main: C -->2");
          #endif
        }
        break;
      case 2:
        //enable next
        if((Drive_C.EnableDrive()) == eMCDone)
        {
          Drive_C_Param.driveStep = 3;
          Drive_C.ResetComState();
          #if DEBUG_C
          Serial.println("Main: C -->3: Config Homing");
          #endif
        }
        break;
      case 3:
        //config homing
        if((Drive_C.ConfigureHoming(Drive_C_Param.DriveHomingMethod)) == eMCDone)
        {
          Drive_C_Param.driveStep = 4;
          Drive_C.ResetComState();
          #if DEBUG_C
          Serial.println("Main: C -->4: Start Homing"); 
          #endif         
        }
        break;
      case 4:
        //start homing
        if((Drive_C.StartHoming()) == eMCDone)
        {
          Drive_C_Param.driveStep = 5;
          Drive_C.ResetComState();
          #if DEBUG_C
          Serial.println("Main: C -->5: Wait 4 Homing"); 
          #endif        
        }
        break;
      case 5:
        //wait for homing done
        if(Drive_C.IsHomingFinished() == eMCDone)
        {
          Drive_C_Param.driveStep = 6;
          Drive_C.ResetComState();
          #if DEBUG_C
          Serial.println("Main: C -->6: PV100"); 
          #endif        
        }     
        break; 
      case 6:
        //move at speed
        if((Drive_C.MoveAtSpeed(100)) == eMCDone)
        {
          Drive_C_Param.driveStep = 7;
          Drive_C.ResetComState();
          Drive_C_Param.stepTime = currentMillis;
          Drive_C_Param.incrementTime = currentMillis;
          #if DEBUG_C
          Serial.println("Main: C -->7");
          #endif
        }
        break;
      case 7:
        //wait some time
        if(currentMillis > (Drive_C_Param.stepTime + 2000))
        {
          Drive_C_Param.driveStep = 8;
          #if DEBUG_C
          Serial.println("Main: C -->8: PV-100");
          #endif
        }
        else if(currentMillis > (Drive_C_Param.incrementTime + 100))
        {
          #if DEBUG_C
          Serial.print(".");
          #endif
          Drive_C_Param.incrementTime = currentMillis;
        }
        break;
      case 8:
        //move at speed
        if((Drive_C.MoveAtSpeed(-100)) == eMCDone)
        {
          Drive_C_Param.driveStep = 9;
          Drive_C.ResetComState();
          Drive_C_Param.stepTime = currentMillis;
          Drive_C_Param.incrementTime = currentMillis;
          #if DEBUG_C
          Serial.println("Main: C -->9");
          #endif
        }
        break;
      case 9:
        //wait some time
        if(currentMillis > (Drive_C_Param.stepTime + 2000))
        {
          Drive_C_Param.driveStep = 10;
          #if DEBUG_C
          Serial.println("Main: C -->10: PP@50000");
          #endif
        }
        else if(currentMillis > (Drive_C_Param.incrementTime + 100))
        {
          #if DEBUG_C
          Serial.print(".");
          #endif
          Drive_C_Param.incrementTime = currentMillis;
        }
        break;
       case 10:
         //move to 0
         if((Drive_C.StartAbsMove(50000,false)) == eMCDone)
         {
           Drive_C_Param.driveStep = 11;
           Drive_C.ResetComState();
           #if DEBUG_C
           Serial.println("Main: C -->11");
           #endif
         }
        break;
       case 11:
         //wait for pos
         if(Drive_C.IsInPos() == eMCDone)
         {
           Drive_C_Param.driveStep = 12;
           Drive_C.ResetComState();
           #if DEBUG_C
           Serial.println("Main: C -->12: PP@0");
           #endif
         }
         break;
       case 12:
         //move to 0
         if((Drive_C.StartAbsMove(0,false)) == eMCDone)
         {
           Drive_C_Param.driveStep = 13;
           Drive_C.ResetComState();
           #if DEBUG_C
           Serial.println("Main: C -->13");
           #endif
         }
         break;
       case 13:
         if(Drive_C.IsInPos() == eMCDone)
         {
           Drive_C_Param.driveStep = 14;
           Drive_C.ResetComState();
           #if DEBUG_C
           Serial.println("Main: C -->14");
           #endif
         }
         break;
       case 14:
         if((Drive_C.SetProfile(Drive_C_Param.actAcc,Drive_C_Param.actDec,Drive_C_Param.actSpeed,0)) == eMCDone)
         {
           Drive_C_Param.driveStep = 1;
           Drive_C_Param.actSpeed += Drive_C_Param.deltaSpeed;
           if((Drive_C_Param.actSpeed <= minSpeed) || (Drive_C_Param.actSpeed >= maxSpeed))
             Drive_C_Param.deltaSpeed = Drive_C_Param.deltaSpeed * (-1);
          
           #if DEBUG_C
           Serial.print("Main: C Loop -->1 @");
           Serial.println(Drive_C_Param.actSpeed, DEC);
           #endif
        }
        break;
   }
   #endif

   #if UpdateDriveD
   //operate Drive D
   switch(Drive_D_Param.driveStep)
   {
      case 0:
        //first get a copy of the drive status
        if((Drive_D.UpdateDriveStatus()) == eMCDone)
        {
          Drive_D_Param.driveStep = 1;
          Drive_D.ResetComState();
          #if DEBUG_D
          Serial.println("Main: D -->1");
          #endif
        }
        break;
      case 1:
        //disable the drive first
        if((Drive_D.DisableDrive()) == eMCDone)
        {
          Drive_D_Param.driveStep = 2;
          Drive_D.ResetComState();
          #if DEBUG_D
          Serial.println("Main: D -->2");
          #endif
        }
        break;
      case 2:
        //enable next
        if((Drive_D.EnableDrive()) == eMCDone)
        {
          Drive_D_Param.driveStep = 3;
          Drive_D.ResetComState();
          #if DEBUG_D
          Serial.println("Main: D -->3: Config Homing");
          #endif
        }
        break;
      case 3:
        //config homing
        if((Drive_D.ConfigureHoming(Drive_D_Param.DriveHomingMethod)) == eMCDone)
        {
          Drive_D_Param.driveStep = 4;
          Drive_D.ResetComState();
          #if DEBUG_D
          Serial.println("Main: D -->4: Start Homing"); 
          #endif         
        }
        break;
      case 4:
        //start homing
        if((Drive_D.StartHoming()) == eMCDone)
        {
          Drive_D_Param.driveStep = 5;
          Drive_D.ResetComState();
          #if DEBUG_D
          Serial.println("Main: D -->5: Wait 4 Homing"); 
          #endif        
        }
        break;
      case 5:
        //wait for homing done
        if(Drive_D.IsHomingFinished() == eMCDone)
        {
          Drive_D_Param.driveStep = 6;
          Drive_D.ResetComState();
          #if DEBUG_D
          Serial.println("Main: D -->6: PV100"); 
          #endif        
        }     
        break; 
      case 6:
        //move at speed
        if((Drive_D.MoveAtSpeed(100)) == eMCDone)
        {
          Drive_D_Param.driveStep = 7;
          Drive_D.ResetComState();
          Drive_D_Param.stepTime = currentMillis;
          Drive_D_Param.incrementTime = currentMillis;
          #if DEBUG_D
          Serial.println("Main: D -->7");
          #endif
        }
        break;
      case 7:
        //wait some time
        if(currentMillis > (Drive_D_Param.stepTime + 2000))
        {
          Drive_D_Param.driveStep = 8;
          #if DEBUG_D
          Serial.println("Main: D -->8: PV-100");
          #endif
        }
        else if(currentMillis > (Drive_D_Param.incrementTime + 100))
        {
          #if DEBUG_D
          Serial.print(".");
          #endif
          Drive_D_Param.incrementTime = currentMillis;
        }
        break;
      case 8:
        //move at speed
        if((Drive_D.MoveAtSpeed(-100)) == eMCDone)
        {
          Drive_D_Param.driveStep = 9;
          Drive_D.ResetComState();
          Drive_D_Param.stepTime = currentMillis;
          Drive_D_Param.incrementTime = currentMillis;
          #if DEBUG_D
          Serial.println("Main: D -->9");
          #endif
        }
        break;
      case 9:
        //wait some time
        if(currentMillis > (Drive_D_Param.stepTime + 2000))
        {
          Drive_D_Param.driveStep = 10;
          #if DEBUG_D
          Serial.println("Main: D -->10: PP@50000");
          #endif
        }
        else if(currentMillis > (Drive_D_Param.incrementTime + 100))
        {
          #if DEBUG_D
          Serial.print(".");
          #endif
          Drive_D_Param.incrementTime = currentMillis;
        }
        break;
       case 10:
         //move to 0
         if((Drive_D.StartAbsMove(50000,false)) == eMCDone)
         {
           Drive_D_Param.driveStep = 11;
           Drive_D.ResetComState();
           #if DEBUG_D
           Serial.println("Main: D -->11");
           #endif
         }
        break;
       case 11:
         //wait for pos
         if(Drive_D.IsInPos() == eMCDone)
         {
           Drive_D_Param.driveStep = 12;
           Drive_D.ResetComState();
           #if DEBUG_D
           Serial.println("Main: D -->12: PP@0");
           #endif
         }
         break;
       case 12:
         //move to 0
         if((Drive_D.StartAbsMove(0,false)) == eMCDone)
         {
           Drive_D_Param.driveStep = 13;
           Drive_D.ResetComState();
           #if DEBUG_D
           Serial.println("Main: D -->13");
           #endif
         }
         break;
       case 13:
         if(Drive_D.IsInPos() == eMCDone)
         {
           Drive_D_Param.driveStep = 14;
           Drive_D.ResetComState();
           #if DEBUG_D
           Serial.println("Main: D -->14");
           #endif
         }
         break;
       case 14:
         if((Drive_D.SetProfile(Drive_D_Param.actAcc,Drive_D_Param.actDec,Drive_D_Param.actSpeed,0)) == eMCDone)
         {
           Drive_D_Param.driveStep = 1;
           Drive_D_Param.actSpeed += Drive_D_Param.deltaSpeed;
           if((Drive_D_Param.actSpeed <= minSpeed) || (Drive_D_Param.actSpeed >= maxSpeed))
             Drive_D_Param.deltaSpeed = Drive_D_Param.deltaSpeed * (-1);
          
           #if DEBUG_D
           Serial.print("Main: D Loop -->1 @");
           Serial.println(Drive_D_Param.actSpeed, DEC);
           #endif
        }
        break;
   }
   #endif

   //check node state
   NodeState = Drive_A.CheckComState();
   StateA = NodeState;
   if((NodeState == eMCError) || (NodeState == eMCTimeout))
   {
      #if RESTART_NODES
        Serial.println("Main: Reset NodeA State");
        Drive_A.ResetComState();
      #endif
      //should be avoided in the end
      Drive_A_Param.driveStep = 0;
   }

   NodeState = Drive_B.CheckComState();
   StateB = NodeState;
   if((NodeState == eMCError) || (NodeState == eMCTimeout))
   {
      #if RESTART_NODES
        Serial.println("Main: Reset NodeB State");
        Drive_B.ResetComState();
      #endif
      //should be avoided in the end
      Drive_B_Param.driveStep = 0;
   }

   NodeState = Drive_C.CheckComState();
   StateC = NodeState;
   if((NodeState == eMCError) || (NodeState == eMCTimeout))
   {
      #if RESTART_NODES
        Serial.println("Main: Reset NodeC State");
        Drive_C.ResetComState();
      #endif
      //should be avoided in the end
      Drive_C_Param.driveStep = 0;
   }

   NodeState = Drive_D.CheckComState();
   StateD = NodeState;
   if((NodeState == eMCError) || (NodeState == eMCTimeout))
   {
      #if RESTART_NODES
        Serial.println("Main: Reset NodeD State");
        Drive_D.ResetComState();
      #endif
      //should be avoided in the end
      Drive_D_Param.driveStep = 0;
   }
}
