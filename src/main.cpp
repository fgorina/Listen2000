#define ESP32_CAN_TX_PIN GPIO_NUM_16
#define ESP32_CAN_RX_PIN GPIO_NUM_17

#include <Arduino.h>
#include <time.h>
#include <Preferences.h>
#include <NMEA2000_esp32.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kDeviceList.h>
#include "State.h"
#include "Utils.h"

tN2kDeviceList *pN2kDeviceList;

const unsigned long ReceiveMessages[] PROGMEM = {126208L, // Request, Command and "Reconocer?"
                                                 127245L, // Rudder Angle
                                                 127250L, // * Rhumb - Vessel heading
                                                 127258L, // * Magnetic Variation
                                                 128259L, // Speed over water
                                                 129026L, // * Fast COG, SOG update
                                                 129029L, // * Position GNSS (Date, time, lat, lon)
                                                 129283L, // * XTE
                                                 129284L, // * Route information
                                                 129285L, // * Active Waypoint data
                                                 130306L, // * Wind data

                                                 // Other not in EVO-1 Document

                                                 127237L, // Heading Track Control
                                                 126720L, // Seatalk1 Pilot Mode
                                                 61184L,  // Seatalk: Wireless Keypad  Control
                                                 65288L,  // Seatalk Alarm
                                                 65379L,  // Seatalk Pilot Mode
                                                 65360L,  // Seatalk Pilot Locked Heading
                                                 0};


const tNMEA2000::tProductInformation LogProductInformation PROGMEM = {
    1300,       // N2kVersion
    202,        // Manufacturer's product code
    "LIS-001",  // Manufacturer's Model ID
    "0.0.1",    // Manufacturer's Software version code
    "LIS-001",  // Manufacturer's Model version
    "00000001", // Manufacturer's Model serial code
    1,          // CertificationLevel
    4           // LoadEquivalency
};

const char LogManufacturerInformation[] PROGMEM = "Paco Gorina, fgorina@gmail.com";
const char LogInstallationDescription1[] PROGMEM = "Listen what happens in your NMEA 2000 bus";
const char LogInstallationDescription2[] PROGMEM = "Select NMEA 2000, SignalK and WiFi and format settings";

const unsigned long AutopilotSerialNumber PROGMEM = 13;
const unsigned char LogDeviceFunction PROGMEM = 140; // Log Recorder
const unsigned char LogtDeviceClass = 20;            // Safety Systems
const uint16_t LogManufacturerCode = 2046;           // Free?
const unsigned char LogIndustryGroup = 4;            // Marine

// State

tState *state{new tState()};

// Preferences
Preferences preferences;
void writePreferences();
void readPreferences();

void writePreferences()
{
  preferences.begin("Listener", false);
  preferences.remove("SOURCES");
  preferences.remove("PGNS");
  
// Write arrays
    preferences.end();
}

void readPreferences()
{
  preferences.begin("Listener", true);
   

  preferences.end();

}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{

    state->HandleNMEA2000Msg(N2kMsg);
}
void OnN2kOpen()
{
  // Start schedulers now.
  Serial.println("NMEA2000 Bus Opened - Starting N2K Task");
}

void setup_NMEA2000()
{

  NMEA2000.SetProductInformation(&LogProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(LogManufacturerInformation, LogInstallationDescription1, LogInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(AutopilotSerialNumber, // Unique number. Use e.g. Serial number.
                                LogDeviceFunction,     // Device function=Autopìlot. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                LogtDeviceClass,       // Device class=Steering and Control Surfaces. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                LogManufacturerCode,   // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                LogIndustryGroup       // Industry Group
  );

  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on theTransmitTransmit  bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 25);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);  ttttrtt   // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)

  //  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple example, limit buffer size to 2, since we are only sending data
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.

  // NMEA2000.ExtendTransmitMessages(TransmitMessages); //We don't transmit messages

  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  // Set Group Handlers
  /*
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN65379(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN127250(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN127245(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN65360(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN65345(&NMEA2000, &pypilot));
    NMEA2000.SetN2kSource(204);

    

    */
   NMEA2000.SetOnOpen(OnN2kOpen);

  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);
  if (NMEA2000.Open()){
    Serial.println("NMEA2000 Bus Opened"); // Enable some LED
  } else {
    Serial.println("NMEA2000 Bus NOT Opened"); // Enable some LED
  }
}


TaskHandle_t taskN2K;

void n2KTask(void *parameter)
{
  while (true)
  {
 
    NMEA2000.ParseMessages();
    vTaskDelay(10);
  }
}

void doCommand(char *s){
  if(strncmp(s,"on", 2)==0){
      state->trackData = true;
      Serial.println("Tracking data started. of to stop");
    } 
    else if(strncmp(s,"of", 2)==0){
      state->trackData = false;
      Serial.println("Tracking data stopped. on to restart");
    } 
    else if(strncmp(s,"cls", 3)==0){
      state->clearAllSources();
      Serial.println("Cleared all sources ");
    } 
    else if (strncmp(s,"clp", 3)==0){
       state->clearAllPGNs();
      Serial.println("Cleared all PGNs");
    } 
    else if (strncmp(s,"shows", 5)==0){
      Serial.println("Following sources:");
      for(int i=0;i<MAX_SOURCES;i++){
        if(state->checkSource(i)){
          
          Serial.print(i);
          Serial.print(" ");
        }
      }
      Serial.println();
    } else if (strncmp(s,"showp", 5)==0){
      Serial.println("Following PGNs:");
      for(int i=0;i<MAX_PGNS;i++){
        unsigned long pgn=state->pgnAt(i);
        if(pgn != 0){
          Serial.print(pgn);
          Serial.print(" ");
        }
      }
      Serial.println();
    } else if (strncmp(s,"fs ",3)==0){
      int src=atoi(s+3);
      if(src>=0 && src<MAX_SOURCES){
        state->followSource(src);
        Serial.print("Following source "); Serial.println(src);
      } else {
        Serial.println("Invalid source");
      }
    } else if (strncmp(s,"us ",3)==0){
      int src=atoi(s+3);
      if(src>=0 && src<MAX_SOURCES){
        state->unfollowSource(src);
        Serial.print("Unfollowing source "); Serial.println(src);
      } else {
        Serial.println("Invalid source");
      }
    } else if (strncmp(s,"fp ",3)==0){
      unsigned long pgn=atol(s+3);
      if(pgn>0){
        state->followPGN(pgn);
        Serial.print("Following PGN "); Serial.println(pgn);
      } else {
        Serial.println("Invalid PGN");
      }
    } else if (strncmp(s,"up ",3)==0){
      unsigned long  pgn=atol(s+3);
      if(pgn>0 ){
        state->unfollowPGN(pgn);
        Serial.print("Unfollowing PGN "); Serial.println(pgn);
      } else {
        Serial.println("Invalid PGN");
      }
    } else {
      Serial.println("Commands are:");
      Serial.println("  cls          Clear all followed sources");
      Serial.println("  clp          Clear all followed PGNs");
      Serial.println("  shows        Show followed sources");
      Serial.println("  showp        Show followed PGNs");
      Serial.println("  fs <source>  Follow source");
      Serial.println("  us <source>  Unfollow source");
      Serial.println("  fp <pgn>     Follow PGN");
      Serial.println("  up <pgn>     Unfollow PGN");
      Serial.println("  of           Turn off data tracking");
      Serial.println("  on           Turn on data tracking");
    }
}



void setup() {
 Serial.begin(115200);
 state->initArrays();
 setup_NMEA2000();
 Serial.println("Starting Tasks");

xTaskCreate(n2KTask, "N2kTask",4000,NULL,1,&taskN2K);
Serial.println("N2K Task Created");
  
}

void loop() {
  char s[80];
  if (Serial.available() ){
    int l = Serial.readBytesUntil('\n', s, sizeof(s)-1);
    s[l] = 0;
    Serial.print(">"); Serial.println(s);
    doCommand(s);

  }
}

