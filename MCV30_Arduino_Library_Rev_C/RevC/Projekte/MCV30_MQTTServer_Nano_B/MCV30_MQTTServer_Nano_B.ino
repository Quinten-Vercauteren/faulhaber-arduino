/****************************************************************************************************************************************************
 *  TITLE: MCV3.0 MQTT Server @ Nano 33 IoT
 *  DESCRIPTION: 
 *
 ****************************************************************************************************************************************************/

//--- defines ---
#define LED_PIN   LED_BUILTIN

//--- includes ---
#include <WiFiNINA.h> 
#include <PubSubClient.h>
#include "WiFiAccess.h"
#include <MsgHandler.h>
#include <MCDrive.h>
#include <stdint.h>

//--- globals ---
//--- 4 WiFi / MQTT ----

const char* ssid = networkSSID;
const char* password = networkPASSWORD;

const char* mqttServer = mqttSERVER;
const char* mqttUsername = mqttUSERNAME;
const char* mqttPassword = mqttPASSWORD;

char subTopic[] = "Nano/ledControl";     //payload[0] will control/set LED
char pubTopic[] = "Nano/ledState";       //payload[0] will have ledState value

char MCControlSubTopicAll[] = "Nano/MC/Ctrl/+";            //subscribe to all /MC/* topics
char MCControlSubTopic[] =    "Nano/MC/Ctrl/Control";      //payload[0] will be the requested state
char MCControlTargetSpeed[] = "Nano/MC/Ctrl/TargetSpeed";  //payload[] is a numeric value
char MCControlTargetPos[] =   "Nano/MC/Ctrl/TargetPos";    //payload[] is a numeric value

char MCStatePubTopic[] =      "Nano/MC/State";        //payload will be "EN" or "DI"
char pubTopicDriveError[] =   "Nano/MC/DriveErrors";
char pubTopicPosition[] =     "Nano/MC/ActPosition";
char pubTopicSpeed[] =        "Nano/MC/ActSpeed";
char pubTopicMotorTemp[] =    "Nano/MC/MotorTemp";


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//--- LED handling -----
int ledState = 0;  //store the stae of the led to be able to report it

//--- 4MCDrive ----

MsgHandler MCMsgHandler;
MCDrive Drive_A;

const int16_t DriveIdA = 1;

uint8_t requestedDriveStep;
uint8_t actDriveStep = 0;
uint32_t updateRate = 10000;

uint8_t RequestedDriveState = 0;
uint8_t ActualDriveState = 0;
int32_t TargetSpeed = 100;
int32_t TargetPos = 0;


void setup_drive()
{
  MCMsgHandler.Open(115200);
  Drive_A.SetNodeId(DriveIdA);
  Drive_A.Connect2MsgHandler(&MCMsgHandler);
}

void setup_wifi() 
{
  int status = WL_IDLE_STATUS;
  delay(10);
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) 
  {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) 
  {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, password);

    // wait 10 seconds for connection:
    delay(10000);
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // print the received signal strength:
  Serial.print("signal strength (RSSI):");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

}

bool identifyTopic(char *RxTopic,char *Reference)
{
  if(strcmp(RxTopic,Reference) == 0)
    return true;
  else
    return false; 
}

int32_t extractValue(byte* payload, unsigned int length)
{
  uint8_t idx = 0;
  int32_t value = 0;
  
  if(payload[idx] == '-')
    idx++;
    
  while(idx < length)
  {
    value = value * 10;
    value = value + (payload[idx] - '0');
    idx++;
  }
  if(payload[0] == '-')
    value = -1 * value;

  return value;
}
void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if(identifyTopic(topic,subTopic))
  {
    // Switch on the LED if 1 was received as first character
    if ((char)payload[0] == '1') 
    {
      digitalWrite(LED_PIN, HIGH);   
      ledState = 1;
      char payLoad[1];
      itoa(ledState, payLoad, 10);
      mqttClient.publish(pubTopic, payLoad,true);
    } 
    else 
    {
      digitalWrite(LED_PIN, LOW); 
      ledState = 0;
      char payLoad[1];
      itoa(ledState, payLoad, 10);
      mqttClient.publish(pubTopic, payLoad,true);
    }
  }
  if(identifyTopic(topic,MCControlSubTopic))
  {
    requestedDriveStep = (uint16_t)extractValue(payload,length);
  } 
  if(identifyTopic(topic,MCControlTargetSpeed))
  {
    TargetSpeed = extractValue(payload,length);
  }
  if(identifyTopic(topic,MCControlTargetPos))
  {
    TargetPos = extractValue(payload,length);
  }
  
}

void reconnect() 
{
  // Loop until we're reconnected
  while (!mqttClient.connected()) 
  {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    //if (client.connect(clientId.c_str(), mqttUsername, mqttPassword)) 
    if (mqttClient.connect("Arduino_Nano_IoT",NULL,NULL,"Nano/MC/Will",1,true,"is Offline"))
    {
      Serial.println("connected");
      // ... and resubscribe
      mqttClient.subscribe(subTopic);
      mqttClient.subscribe(MCControlSubTopicAll);
      mqttClient.publish("Nano/MC/Will","is Online",true);
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void updateDriveComm()
{
static bool isAutoUpdate;
static uint32_t lastAutoUpdate = 0;
uint32_t currentMillis = millis();
DriveCommStates NodeState = Drive_A.CheckComState();
   
   Drive_A.SetActTime(currentMillis);
   MCMsgHandler.Update(currentMillis); 

   if((NodeState == eMCError) || (NodeState == eMCTimeout))
   {
      //Serial.println("Main: Reset NodeA State");
      Drive_A.ResetComState();
      //should be avoided in the end
      actDriveStep = 0;
   }


   switch(actDriveStep)
   {
      case 0:
        //do nothing - that's the idle state
        if(requestedDriveStep > 0)
        {
          //start handling of the request
          actDriveStep = requestedDriveStep;
          //reset request again          
          requestedDriveStep = 0;
          isAutoUpdate = false;
          Serial.print("Start ");
          Serial.println(actDriveStep);
        }
        else
        {
          //no request pending
          //check for timeout of auto-update
          if(lastAutoUpdate + updateRate < currentMillis)
          {
            actDriveStep = 20;
            lastAutoUpdate = currentMillis;
            isAutoUpdate = true;
          }
        }
        break;
      case 1:
        //get a copy of the drive status
        if((Drive_A.UpdateDriveStatus()) == eMCDone)
        {
          //switch back to idle state
          actDriveStep = 0;
          Drive_A.ResetComState();
          Serial.println("Main: Status updated");
        }
        break;
      case 2:
        //disable the drive
        if((Drive_A.DisableDrive()) == eMCDone)
        {
          //switch back to idle state
          actDriveStep = 0;
          Drive_A.ResetComState();
          Serial.println("Main: Drive disabled");
          mqttClient.publish(MCStatePubTopic,"Disabled");
        }
        break;
      case 3:
        //enable the drive
        if((Drive_A.EnableDrive()) == eMCDone)
        {
          //switch back to idle state
          actDriveStep = 0;
          Drive_A.ResetComState();
          Serial.println("Main: Drive enabled");
          mqttClient.publish(MCStatePubTopic,"Enabled");
        }
        break;
      case 4:
        //move at speed
        if((Drive_A.MoveAtSpeed(0)) == eMCDone)
        {
          //switch back to idle state
          actDriveStep = 0;
          Drive_A.ResetComState();
          Serial.println("Main: PV @ 0");
        }
        break;
      case 5:
        //move at speed
        if((Drive_A.MoveAtSpeed(TargetSpeed)) == eMCDone)
        {
          //switch back to idle state
          actDriveStep = 0;
          Drive_A.ResetComState();
          Serial.print("Main: PV @ ");
          Serial.println(TargetSpeed);
        }
        break;
       case 6:
        //move to pos
        if((Drive_A.StartAbsMove(0,false)) == eMCDone)
        {
          //switch back to idle state
          actDriveStep = 0;
          Drive_A.ResetComState();
          Serial.println("Main: Move to 0");
        }
        break;
       case 7:
        //move to pos
        if((Drive_A.StartAbsMove(TargetPos,false)) == eMCDone)
        {
          //switch back to idle state
          actDriveStep = 0;
          Drive_A.ResetComState();
          Serial.print("Main: Move to ");
          Serial.println(TargetPos);
        }
        break;
       case 8:
         //wait for pos
         if(Drive_A.IsInPos() == eMCDone)
         {
           //switch back to idle state
           actDriveStep = 0;
           Drive_A.ResetComState();
           Serial.println("Main: Drive is in Pos");
         }
         break;
       case 20:
         //Update ActValues
         if(Drive_A.UpdateActValues() == eMCDone)
         {
           char payload[20];
           //switch back to idle state
           actDriveStep = 0;
           Drive_A.ResetComState();
           if(!isAutoUpdate)
              Serial.println("Main: updates pos/speed");
           
           itoa(Drive_A.GetActualPosition(), payload, 10);
           mqttClient.publish(pubTopicPosition, payload,true);

           itoa(Drive_A.GetActualSpeed(), payload, 10);
           mqttClient.publish(pubTopicSpeed, payload,true);

         }
         break;
       default:
           //switch back to idle state
           actDriveStep = 0;
           Serial.println("Main: Unexpected command");
           break;
   }
}

void setup() 
{
  pinMode(LED_PIN, OUTPUT);     
  Serial.begin(500000);
  /*
  while(!Serial)
    ; */
  setup_wifi();
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);

  setup_drive();
}

void loop() 
{
long lastMsg = 0;

  if (!mqttClient.connected()) 
  {
    reconnect();
  }
  mqttClient.loop();

  updateDriveComm();
  
  long now = millis();
  if (now - lastMsg > 5000) 
  {
    lastMsg = now;
    char payLoad[1];
    itoa(ledState, payLoad, 10);
    mqttClient.publish(pubTopic, payLoad,true);
  }
}
