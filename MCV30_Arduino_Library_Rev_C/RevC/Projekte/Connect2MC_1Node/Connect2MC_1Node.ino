
//--- defines ---

//--- includes ---
#include <MsgHandler.h>
#include <MCDrive.h>
#include <stdint.h>

//--- globals ---

const uint32_t maxAcc = 2500;
const uint32_t minAccDec = 250;
const uint32_t maxDec = 2500;
const uint32_t maxSpeed = 4000;
const uint32_t minSpeed = 500;

MsgHandler MCMsgHandler;
MCDrive Drive_A;

uint8_t toggle = 0;

const int16_t DriveIdA = 1;
uint16_t driveStep = 0;
uint32_t stepTime;
uint32_t incrementTime;
uint32_t actAcc = maxAcc;
uint32_t actDec = maxDec;
uint32_t actSpeed = minSpeed;

uint32_t deltaSpeed = minSpeed;
uint32_t deltaAccDec = minAccDec;

int8_t DriveAHomingMethod = 33;

int8_t LED_State = 0;
int32_t LED_Time;


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  // Debug Port
  Serial.begin(500000);
  //while(!Serial);
  //here we really start
  
  MCMsgHandler.Open(115200);
  Drive_A.SetNodeId(DriveIdA);
  Drive_A.Connect2MsgHandler(&MCMsgHandler);
  LED_Time = millis();
}


void loop() {
   // put your main code here, to run repeatedly:
   DriveCommStates NodeState;
   uint32_t currentMillis = millis();
   
   Drive_A.SetActTime(currentMillis);
   MCMsgHandler.Update(currentMillis); 
   //check the status of the MsgHandler here too

   switch(driveStep)
   {
      case 0:
        //first get a copy of the drive status
        if((Drive_A.UpdateDriveStatus()) == eMCDone)
        {
          driveStep = 1;
          Drive_A.ResetComState();
          Serial.println("Main: -->1");
        }
        break;
      case 1:
        //disable the drive first
        if((Drive_A.DisableDrive()) == eMCDone)
        {
          driveStep = 2;
          Drive_A.ResetComState();
          Serial.println("Main: -->2");
        }
        break;
      case 2:
        //enable next
        if((Drive_A.EnableDrive()) == eMCDone)
        {
          driveStep = 3;
          Drive_A.ResetComState();
          Serial.println("Main: -->3: Config Homing");
        }
        break;
      case 3:
        //config homing
        if((Drive_A.ConfigureHoming(DriveAHomingMethod)) == eMCDone)
        {
          driveStep = 4;
          Drive_A.ResetComState();
          Serial.println("Main: -->4: Start Homing");          
        }
        break;
      case 4:
        //start homing
        if((Drive_A.StartHoming()) == eMCDone)
        {
          driveStep = 5;
          Drive_A.ResetComState();
          Serial.println("Main: -->5: Wait 4 Homing");         
        }
        break;
      case 5:
        //wait for homing done
        if(Drive_A.IsHomingFinished() == eMCDone)
        {
          driveStep = 6;
          Drive_A.ResetComState();
          Serial.println("Main: -->6: PV100");         
        }     
        break; 
      case 6:
        //move at speed
        if((Drive_A.MoveAtSpeed(100)) == eMCDone)
        {
          driveStep = 7;
          Drive_A.ResetComState();
          stepTime = currentMillis;
          incrementTime = currentMillis;
          Serial.println("Main: -->7");
        }
        break;
      case 7:
        //wait some time
       if(currentMillis > (stepTime + 2000))
       {
          driveStep = 8;
          Serial.println("Main: -->8: PV-100");
       }
       else if(currentMillis > (incrementTime + 100))
       {
          Serial.print(".");
          incrementTime = currentMillis;
       }
       break;
      case 8:
        //move at speed
        if((Drive_A.MoveAtSpeed(-100)) == eMCDone)
        {
          driveStep = 9;
          Drive_A.ResetComState();
          stepTime = currentMillis;
          incrementTime = currentMillis;
          Serial.println("Main: -->9");
        }
        break;
      case 9:
        //wait some time
        if(currentMillis > (stepTime + 2000))
        {
          driveStep = 10;
          Serial.println("Main: -->10: PP@50000");
        }
        else if(currentMillis > (incrementTime + 100))
        {
          Serial.print(".");
          incrementTime = currentMillis;
        }
        break;
       case 10:
        //move to 0
        if((Drive_A.StartAbsMove(50000,false)) == eMCDone)
        {
          driveStep = 11;
          Drive_A.ResetComState();
          Serial.println("Main: -->11");
        }
        break;
       case 11:
         //wait for pos
         if(Drive_A.IsInPos() == eMCDone)
         {
           driveStep = 12;
           Drive_A.ResetComState();
           Serial.println("Main: -->12: PP@0");
         }
         break;
       case 12:
        //move to 0
        if((Drive_A.StartAbsMove(0,false)) == eMCDone)
        {
          driveStep = 13;
          Drive_A.ResetComState();
          Serial.println("Main: -->13");
        }
        break;
       case 13:
         if(Drive_A.IsInPos() == eMCDone)
         {
           driveStep = 14;
           Drive_A.ResetComState();
           Serial.println("Main: -->14");
         }
         break;
       case 14:
        if((Drive_A.SetProfile(actAcc,actDec,actSpeed,0)) == eMCDone)
        {
          driveStep = 1;
          actSpeed += deltaSpeed;
          if((actSpeed <= minSpeed) || (actSpeed >= maxSpeed))
            deltaSpeed = deltaSpeed * (-1);
          
          Serial.print("Main: Loop -->1 @");
          Serial.println(actSpeed, DEC);
        }
        break;
   }
   NodeState = Drive_A.CheckComState();
   if((NodeState == eMCError) || (NodeState == eMCTimeout))
   {
      //Serial.println("Main: Reset NodeA State");
      //Drive_A.ResetComState();
      //should be avoided in the end
      driveStep = 0;
   }
   if((millis() - LED_Time) > 500)
   {
    LED_Time = millis();
    if(LED_State)
    {
      digitalWrite(LED_BUILTIN,LOW);
      LED_State = 0;
    }
    else
    {
      digitalWrite(LED_BUILTIN,HIGH);
      LED_State = 1;
    }
   }


}
