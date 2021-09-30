#include "EPICLibrary.h"
#include "EPICComm.h"

//Teensy 3.6 specific SD card config
#define USE_SDIO 1

#define DEBUG_SERIAL Serial
//#define RECORD_LENGTH 20 //4 GPS ints + 4 TSEN Ints + 1 Status int - ie twice this in bytes (18KB)
//#define EFU_RECORDS 1 // Number of records to hold in memory

enum AnalogChannel : uint8_t {
		TEENSY,
		BATT,
		TSEN	
	};

/* Variables that can be reconfigured */
float HeaterSetpoint = -5.00;
float BoostSetpoint = 30.00;
int DataRate = 20; //Period at which to collect GPS/TSEN data
int StatusRate = 60; //Period at which to collect housekeeping data
int TransmitPower = 15;

/* Other */
float DeadBand = 1.0;
long startTime = 0; // milliseconds to start next measurement
time_t GPSStartTime;  //Unix time_t of starttime of profile from PU clock
float GPSStartLat;  // Initial Latitude decimal degrees (float32) = 1m
float GPSStartLon; //Initial Longitude decimal degrees (float32) = 1m
uint16_t GPSAlt; 

float V_Battery;
float V_DCDC;
float V_3V3;

uint16_t VBatt_tx;
uint16_t Vdcdc_tx;
uint16_t V3_tx;
float T_batt_tx;
uint16_t TSEN_tx; 
uint8_t H_tx;

float TBatt_val;
float TSpare_val;
float TPCB_val;

bool HeaterBoost = false;
bool HeaterStatus = false;
byte UBLOXsettingsArray[] = {0x06, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};

String FileName = "EPIC_FL.txt";

EPICLibrary EPIC;
LoopbackStream LoRaBuff;
TinyGPSPlus gps;
SdFatSdio SD;
EPICComm EC(&LoRaBuff);

TSensor1Bus  TempPCB(TPCB); //initialize temperautre sensors
TSensor1Bus  TempBatt(TBATT);
TSensor1Bus  TempSpare(TSPARE);

int counter = 0;
long HKcounter = 0;

void setup() {
  EPIC.Setup(); //initialize pins and ports
  delay(2000);  //wait for serial port

  DEBUG_SERIAL.println("EPIC V2");

/*Set Up the watchdog with a 60s timeout */
  int countdownMS = Watchdog.enable(60000);
  DEBUG_SERIAL.print("Enabled the watchdog with max countdown of ");
  DEBUG_SERIAL.print(countdownMS, DEC);
  DEBUG_SERIAL.println(" milliseconds!");

     //Setup UBLOX GPS reciever
   EPIC.GPSreset();
   EPIC.configureUblox(UBLOXsettingsArray);
      
  if (!EC.Begin(868E6, 19, 18, 30)) {
    DEBUG_SERIAL.println("Starting LoRa failed!");
    while (1);
  }
  EC.SetModulation(11, 250E3);
  EC.SetTXPower(TransmitPower);

  DEBUG_SERIAL.println("SetUp Complete");  

  EPIC.SolarOn();
  EPIC.TSENOff();
  HKcounter = millis();
}

void loop() {
  
  checkForSerial();
  TempPCB.ManageState(TPCB_val);
  TempBatt.ManageState(TBatt_val); 
  TempSpare.ManageState(TSpare_val);
  V_3V3 = 1000*EPIC.GetAnalogVoltage(TEENSY); //Check teensy voltage to handle low power situations
  V_Battery = 1000*EPIC.GetAnalogVoltage(BATT);

  if(V_3V3>=3050){

    if(millis()>startTime-500){ //wakeup TSEN 500 milliseconds prior to measurement
      
      EPIC.TSENOn();
      EPIC.ListenForTSEN();
          
    }

    if(millis()>startTime){

      startTime = millis() + DataRate*1000;
      EPIC.RequestTSEN();
    
      checkForSerial();
      EPIC.ListenForTSEN();
      V_DCDC = 1000*EPIC.GetAnalogVoltage(TSEN);
    
      SaveRecord();
      WriteToSD(FileName);
      checkForSerial();
      updateRTCfromGPS();
      EPIC.TSENOff();
     
    }
  }else if(V_3V3<3050 && !-999){
    EPIC.TSENOff();
    digitalWrite(BATT_HEATER, LOW);
  }

  
  if(millis()-HKcounter>=(StatusRate*1000)){


    setHeater();
    SendHK();
    HKcounter = millis();
   
  }
}


bool SerComRX()
 {
     /* Recieves commands from the PIB and parses them.  
     */
     int8_t tmp1;
     uint8_t tmp2;
     
     switch (EC.RX()) {
         case ASCII_MESSAGE:
            DEBUG_SERIAL.print("Received message: "); DEBUG_SERIAL.println(EC.ascii_rx.msg_id);
            DEBUG_SERIAL.print("Message Content: "); DEBUG_SERIAL.println(EC.ascii_rx.buffer);
            switch (EC.ascii_rx.msg_id){
                case EFU_SET_HEATERS:
                    tmp1 = EC.RX_SetHeaters(&HeaterSetpoint,&BoostSetpoint);
                    EC.TX_Ack(EFU_SET_HEATERS,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("Received EFU_SET_HEATER: ");
                        DEBUG_SERIAL.print(HeaterSetpoint);
                        DEBUG_SERIAL.print(",");
                        DEBUG_SERIAL.println(BoostSetpoint);
                        
                    }
                    return false;
                case EFU_SET_DATA_RATE:
                    tmp1 = EC.RX_SetDataRate(&DataRate);
                    EC.TX_Ack(EFU_SET_DATA_RATE,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("Received EFU_SET_DATA_RATE: ");
                        DEBUG_SERIAL.println(DataRate); 
                    }
                    return false;
                case EFU_SET_TX_RATE:
                    tmp1 = EC.RX_SetTxRate(&StatusRate);
                    EC.TX_Ack(EFU_SET_TX_RATE,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("Received EFU_SET_TX_RATE: ");
                        DEBUG_SERIAL.println(StatusRate); 
                    }
                    return false;
                case EFU_RESET:
                    EC.TX_Ack(EFU_RESET,true);
                    DEBUG_SERIAL.println("[warning] Rebooting in 2 seconds");
                    delay(2000);
                    WRITE_RESTART(0x5FA0004);
                    return false;
                case TX_PWR:
                    tmp1 = EC.RX_SetTXPower(&tmp2);
                    EC.TX_Ack(TX_PWR,tmp1);
                    TransmitPower = (float)tmp2;
                    EC.SetTXPower(TransmitPower);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("Received TX_PWR:");
                        DEBUG_SERIAL.println(TransmitPower); 
                    }
                    return false;
                    
                default:
                    EC.TX_Ack(EC.ascii_rx.msg_id,false);
                    return false;   
              }
        case ACK_MESSAGE:
                DEBUG_SERIAL.print("ACK/NAK for msg: "); DEBUG_SERIAL.println(EC.ack_id);
                DEBUG_SERIAL.print("Value: ");
                EC.ack_value ? DEBUG_SERIAL.println("ACK") : DEBUG_SERIAL.println("NAK");
                return false;
//        case BIN_MESSAGE:
//              DEBUG_SERIAL.print("Binary message: "); Serial.println(EC.binary_rx.bin_id);
//              DEBUG_SERIAL.print("Buffer: ");
//              for (int i = 0; i < EC.binary_rx.bin_length; i++) {
//                DEBUG_SERIAL.print((char) EC.binary_rx.bin_buffer[i]);
//              }
//              DEBUG_SERIAL.println();
             
              //break;
        case NO_MESSAGE:
          default:
                return false;
        return false;                    

        }
 }

int SaveRecord()
{

  GPSStartTime = now();
  GPSStartLat = gps.location.lat();
  GPSStartLon = gps.location.lng();
  GPSAlt = (uint16_t)(gps.altitude.meters()); 
  VBatt_tx =  (uint16_t)V_Battery;
  Vdcdc_tx = (uint16_t)V_DCDC;
  TSEN_tx = (uint16_t)EPIC.TSEN_T;
  //T_batt_tx = TBatt_val;
  H_tx = (uint8_t)HeaterStatus;

 // EC.TX_Status(GPSStartTime, GPSStartLat, GPSStartLon, GPSAlt, TSEN_tx, EPIC.TSEN_P, EPIC.TSEN_TP,TBatt_val,VBatt_tx,Vdcdc_tx,H_tx);
    EC.TX_Data(GPSStartTime, GPSStartLat, GPSStartLon, GPSAlt, TSEN_tx, EPIC.TSEN_P, EPIC.TSEN_TP);
}

int SendHK(){

  GPSStartTime = now();
  VBatt_tx =  (uint16_t)V_Battery;
  Vdcdc_tx = (uint16_t)V_DCDC;
  V3_tx = (uint16_t)V_3V3;
  H_tx = (uint8_t)HeaterStatus;

  EC.TX_HK(GPSStartTime,VBatt_tx,Vdcdc_tx,V3_tx,TBatt_val,TPCB_val,H_tx);
}

bool checkForSerial()
{
  /* Checks for any new bytes on the various serial ports
   *  and processes them when necessary. 
   */
    Watchdog.reset(); 

  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps.encode(c);
  }
   if (LoRa.parsePacket())
  {
      DEBUG_SERIAL.println("LoRa Waiting");
      SerComRX();
      DEBUG_SERIAL.println("Done with SerComRX");
      Watchdog.reset();
  }

  return false;
}

bool updateRTCfromGPS()
{
    if ((gps.time.age() < 1500) && (gps.satellites.value() > 4))  //we have a GPS time fix within the last 1.5s
    {
        if(timeStatus() != timeSet) //if the time is not set, set it
        {
            setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
            DEBUG_SERIAL.println("Setting RTC to GPS time "); 
            return true;
        }

        if (abs(minute() * 60 + second() - (gps.time.minute()*60 +gps.time.second())) > 2) //if the clock is more than 1 second off
        {
            DEBUG_SERIAL.printf("Updating RTC to GPS time from %d:%d:%d to %u:%u:%u\n", hour(), minute(), second(), gps.time.hour(), gps.time.minute(), gps.time.second());
            DEBUG_SERIAL.println((gps.time.hour()*3600 + gps.time.minute()*60 + gps.time.second()) - (hour()*3600 + minute()*60 + second()));
            setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
            return true;
        }

        return false;

    }
    return false;

}

int setHeater()
{
  /* Thermostat for both heaters, bases on global variables
   *  Heater1Setpoint, Heater2Setpoint and DeadBand
   *  Returns true if either heater is on
   */

   if (V_Battery > 4100)
    {
        HeaterBoost = true;
        //DEBUG_SERIAL.println("Activating Boost Heat");
    }
    if (V_Battery < 3950)
    {   
        HeaterBoost = false;
        //DEBUG_SERIAL.println("Deactivating Boost Heat");
    }

    if (TBatt_val > -999) //if battery temperautre is working, control based on that
    {  

        if(HeaterBoost){ //if heater boost active

          if(TBatt_val < BoostSetpoint)
          {
          //Serial.println("heater on");
            digitalWrite(BATT_HEATER, HIGH);
            HeaterStatus = true;
          }

          if(TBatt_val > (BoostSetpoint + DeadBand))
          {
          //Serial.println("heater off");
            digitalWrite(BATT_HEATER, LOW);
            HeaterStatus = false;
          }
          return HeaterStatus;
        }
 
        if(!HeaterBoost){ //if heater boost isn't active

          if(TBatt_val < HeaterSetpoint)
          {
          //Serial.println("heater on");
            digitalWrite(BATT_HEATER, HIGH);
            HeaterStatus = true;
          }

          if(TBatt_val > (HeaterSetpoint + DeadBand))
          {
          //Serial.println("heater off");
            digitalWrite(BATT_HEATER, LOW);
            HeaterStatus = false;
          }
          return HeaterStatus;
        }
  }

  else if (TPCB_val > -999) //if PCB temperature is working, control based on that
  {
    if(TPCB_val < HeaterSetpoint)
    {
        digitalWrite(BATT_HEATER, HIGH);
        HeaterStatus = true;
    }

    if(TPCB_val > (HeaterSetpoint + DeadBand))
    {
        digitalWrite(BATT_HEATER, LOW);
        HeaterStatus = false;
    }
    return HeaterStatus;
  }

  else //if both thermistors fail go to 50% duty cycle
  {
      digitalWrite(BATT_HEATER, !digitalRead(BATT_HEATER)); 
  }
return HeaterStatus;
  
}


bool WriteToSD(String FName)
{
File EFUFile;

if (!SD.begin()) {
    DEBUG_SERIAL.println("initialization failed!");
    return false;
  }
char filename[100];
FName.toCharArray(filename, 100);
EFUFile = SD.open(filename, FILE_WRITE);
EFUFile.printf("%ld,%8.5f,%8.5f,%8.5f,%d,%d,%d,%4.3f,%4.3f,%4.3f,%5.2f,%5.2f,%d\n", now(), gps.location.lat(), gps.location.lng(), gps.altitude.meters(), EPIC.TSEN_T, EPIC.TSEN_P, EPIC.TSEN_TP, V_Battery, V_DCDC, V_3V3, TBatt_val,TPCB_val,HeaterStatus);
//DEBUG_SERIAL.printf("%ld,%8.5f,%8.5f,%8.5f,%d,%d,%d,%4.3f,%4.3f,%4.3f,%5.2f,%5.2f,%d\n", now(), gps.location.lat(), gps.location.lng(), gps.altitude.meters(), EPIC.TSEN_T, EPIC.TSEN_P, EPIC.TSEN_TP, V_Battery, V_DCDC, V_3V3, TBatt_val,TPCB_val,HeaterStatus);
EFUFile.close();

//DEBUG_SERIAL.printf("Wrote: %ld,%8.5f,%8.5f,%d,%d,%d,%d,%4.3f,%4.3f,%4.3f,%5.2f,%5.2f\n ", now(), gps.location.lat(), gps.location.lng(), gps.altitude.meters(), TSEN_T, TSEN_P, TSEN_TP, V_Battery, V_DCDC, V_3V3, TBatt_val,TPCB_val );

return true;


}
