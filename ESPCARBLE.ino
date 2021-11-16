
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Ticker.h> 

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

Ticker Timer10Ms;
//////////////////////
// Port Definitions //
//////////////////////
#define  NO_130MOTOR    1
#define  OLED_RED    25
#define  OLED_GREEN  26
#define  OLED_BLUE   27
#define  OLEFT_MTRP 35
#define  OLEFT_MTRM 36
#define  ORIGHT_MTRP 32
#define  ORIGHT_MTRM 33
//////////////////////
// Timer Definition //
//////////////////////
#define  TIMER100MS  (10)
#define  TIMER5000MS (500)

/////////////////////
// Struct          //
/////////////////////
struct LedCommad {
  byte Command;
  byte Red;
  byte Green;
  byte Blue;
  byte Dummy;
};

struct MotorCommand {
  byte Command;
  byte LeftP;
  byte LeftM;
  byte RightP;
  byte RightM;
};
/////////////////////
// Data            //
/////////////////////
byte packetReq;
byte packet[256]; //buffer to hold incoming and outgoing packets
byte Status;
word BaseTimer;
word Blink;
word MotorTimer;
word InitTimer;
byte RedLed;
byte BlueLed;
byte GreenLed;
word TLeftMotorP;
word TLeftMotorM;
word TRightMotorP;
word TRightMotorM;
word LeftMotorP;
word LeftMotorM;
word RightMotorP;
word RightMotorM;
word SLeftMotorP;
word SLeftMotorM;
word SRightMotorP;
word SRightMotorM;


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/////////////////////////////////////////
/////// 10ms Base Timer //////////////////
/////////////////////////////////////////
void Int10ms( void )
{
  if (--BaseTimer == 0)
  { /*** 500ms Blink Timer & Flag ****/
    BaseTimer = 50;
    Blink = ~Blink;
  }
  if(MotorTimer) 
           MotorTimer--;
  if(InitTimer){
              InitTimer--;
    if(InitTimer == 0)
         Serial.println("Timer  End..");
  }
}

/////////////////////////////////////////
/////// Nible2Pwm  //////////////////////
/////////////////////////////////////////
word ToPwm( byte Val )
{
    return (((word)Val) << 5);    //max fff-min000
}

/////////////////////////////////////////
/////// Led Main            /////////////
/////////////////////////////////////////
void LedMain(void)
{
  if(InitTimer)
  {
     GreenLed= BlueLed =RedLed = Blink;
  }
  if (RedLed )  digitalWrite(OLED_RED, HIGH);
  else            digitalWrite(OLED_RED, LOW );

  if (BlueLed ) digitalWrite(OLED_BLUE, HIGH);
  else            digitalWrite(OLED_BLUE, LOW );

  if (GreenLed)  digitalWrite(OLED_GREEN, HIGH);
  else            digitalWrite(OLED_GREEN, LOW );

}
/////////////////////////////////////////
/////// Target ... up/down  /////////////
/////////////////////////////////////////
void TargetUpDown( word* Target , word* Now )
{
#if  NO_130MOTOR
   *Now = *Target;
#else
   if( *Target > *Now ) 
  {
    if( (*Target - *Now) >= 0x60 ) *Now += 0x60;  
    else                           *Now = *Target; 
   }else{
     if( (*Now - *Target) >= 0x60 ) *Now -= 0x60;
     else                           *Now = *Target;
   }
#endif
}
/////////////////////////////////////////
/////// Motor Main          /////////////
/////////////////////////////////////////
void MotorMain(void)
{
  if(!MotorTimer)
  {
     LeftMotorP   = 0;
     LeftMotorM   = 0;
     RightMotorP  = 0;
     RightMotorM  = 0;
  }
  else
  {
    noInterrupts();
     LeftMotorP   = TLeftMotorP;
     LeftMotorM   = TLeftMotorM;
     RightMotorP  = TRightMotorP;
     RightMotorM  = TRightMotorM;
     interrupts();
  }
  
  if ( SLeftMotorP != LeftMotorP )
  {
    TargetUpDown( &LeftMotorP , &SLeftMotorP );
    analogWrite(OLEFT_MTRP , SLeftMotorP );
  }
  if ( SLeftMotorM != LeftMotorM )
  {
    TargetUpDown( &LeftMotorM , &SLeftMotorM );
    analogWrite(OLEFT_MTRM , SLeftMotorM );
  }
  if ( SRightMotorP != RightMotorP )
  {
    TargetUpDown( &RightMotorP , &SRightMotorP );
    analogWrite(ORIGHT_MTRP, SRightMotorP );
  }
  if ( SRightMotorM != RightMotorM )
  {
    TargetUpDown( &RightMotorM , &SRightMotorM );
    analogWrite(ORIGHT_MTRM, SRightMotorM );
  }
}



class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
        
       if ( rxValue.length() >= 5 )
        {
          switch (rxValue[0] )
          {
              case  'L' :
                RedLed   = rxValue[1];
                GreenLed = rxValue[2];
                BlueLed  = rxValue[3];
                break;
              case  'M' :
                TLeftMotorP  =  ToPwm( rxValue[1]);
                TLeftMotorM  =  ToPwm( rxValue[2]);
                TRightMotorP =  ToPwm( rxValue[3]);
                TRightMotorM =  ToPwm( rxValue[4]);
                MotorTimer  = TIMER100MS;
              default:
                break;
         }
      }  
      if (rxValue.length() > 0) { 
        Serial.print("RV:");
        Serial.print(rxValue.length(),HEX);
        Serial.print(":");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i],HEX);

        Serial.println();
      }
    }
};

String mac2String(byte ar[]) {
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%02X", ar[i]); // J-M-L: slight modification, added the 0 in the format for padding 
    s += buf;
    if (i < 5) s += ':';
  }
  return s;
}



void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  BaseTimer = 49;
  InitTimer = TIMER5000MS;
  uint64_t espChipID = ESP.getEfuseMac();

//////////////////////////////////////
  pinMode(OLED_RED   , OUTPUT);
  pinMode(OLED_GREEN , OUTPUT);
  pinMode(OLED_BLUE  , OUTPUT);
  pinMode(OLEFT_MTRP  , OUTPUT);
  pinMode(OLEFT_MTRM  , OUTPUT);
  pinMode(ORIGHT_MTRP , OUTPUT);
  pinMode(ORIGHT_MTRM , OUTPUT);

  RedLed  = 0;
  BlueLed = 0;
  GreenLed = 0;
  LeftMotorP = 0;
  LeftMotorM = 0;
  RightMotorP = 0;
  RightMotorM = 0;
  SLeftMotorP = 1;
  SLeftMotorM = 1;
  SRightMotorP = 1;
  SRightMotorM = 1;
  Blink = 0;
  MotorMain();
  Timer10Ms.attach(0.010f, Int10ms);
  Serial.println("start Program");

  // Create the BLE Device
  String  str =  "UART Service";
  str = String(str+mac2String((byte*) &espChipID)),
  BLEDevice::init("EspCarBle");
  Serial.println(str);
  
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
									CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
								);
                     
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());


  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

///////////////////////////////////////////////////////////
void BleMain(void)
{
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(100);// give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}
///////////////////////////////////////////////////////////
void loop() {
  MotorMain();
  LedMain();
  BleMain();
}
