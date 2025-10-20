
#include <Arduino.h>
#include <string.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include "PyTypes.h"
#include "pgnsToString.h"
#include "State.h"


// Given a time_t updates the RTC

void tState::initArrays(){
  for(int i = 0; i < MAX_SOURCES; i++){
    sources[i] = false;
    
  } 
  for(int i = 0; i< MAX_PGNS; i++){
    pgns[i] = 0;
  }

}

 void tState::unfollowSource(int source){
    sources[source] = false;
 }
  void tState::followSource(int source){
    sources[source] = true;
  }

  bool tState::checkSource(int source){
    return sources[source];
  }
 void tState::clearAllSources(){
    for(int i = 0; i < MAX_SOURCES; i++){
      sources[i] = false;
    }
 }

 void tState::followPGN(unsigned long pgn){
    for(int i = 0; i < MAX_PGNS; i++){
      if(pgns[i] == 0){
        pgns[i] = pgn;
        break;
      }
    }
 }

  void tState::unfollowPGN(unsigned long pgn){
   for(int i = 0; i < MAX_PGNS; i++){
      if(pgns[i] == pgn){
        pgns[i] = 0;
        break;
      }
    }

  }

  bool tState::checkPGN(unsigned long pgn){

    for(int i = 0; i < MAX_PGNS; i++){
      if(pgns[i] == pgn){
        return true;
      }
    }
    return false;
  }

  void tState::clearAllPGNs(){
    for(int i = 0; i < MAX_PGNS; i++){
      pgns[i] = 0;
    }
 }

/* NMEA 2000 support */

/* Route WP Information. Not used for log */

bool tState::ParseN2kPGN129285(const tN2kMsg &N2kMsg, uint16_t &Start, uint16_t &nItems, uint16_t &Database, uint16_t &Route,
                               tN2kNavigationDirection &NavDirection, char *RouteName, size_t RouteNameBufSize, tN2kGenericStatusPair &SupplementaryData,
                               uint16_t wptArraySize, t_waypoint *waypoints)
{

  if (N2kMsg.PGN != 129285L)
    return false;

  int index = 0;
  unsigned char c;
  Start = N2kMsg.Get2ByteUInt(index);
  nItems = N2kMsg.Get2ByteUInt(index);
  Database = N2kMsg.Get2ByteUInt(index);
  Route = N2kMsg.Get2ByteUInt(index);

  c = N2kMsg.GetByte(index);
  // Use flags to set values
  SupplementaryData = tN2kGenericStatusPair((c & 0x18) >> 3);
  NavDirection = tN2kNavigationDirection(c && 0x07);
  N2kMsg.GetVarStr(RouteNameBufSize, RouteName, index);
  c = N2kMsg.GetByte(index); // Reserved

  uint16_t wpid;
  char wpname[20] = "";
  double lat;
  double lon;
  unsigned int nameSize = 20;

  for (int i = 0; i < min(nItems, wptArraySize); i++)
  {
    nameSize = 20;
    waypoints[i].id = N2kMsg.Get2ByteUInt(index);
    N2kMsg.GetVarStr(nameSize, waypoints[i].name, index);
    waypoints[i].latitude = N2kMsg.Get4ByteDouble(1.0e-7, index);
    waypoints[i].longitude = N2kMsg.Get4ByteDouble(1.0e-7, index);
  }

  return true;
}

void tState::handleWindDatum(const tN2kMsg &N2kMsg){
  if (N2kMsg.PGN != 65345)
    return ;
  int index = 0;
 unsigned int prefix  = N2kMsg.Get2ByteUInt(index); // Raymarine

 double windDatum = N2kMsg.Get2ByteDouble(0.0001, index) / PI * 180.0;
 double rollingAverage = N2kMsg.Get2ByteDouble(0.0001, index) / PI * 180.0;;
 
 Serial.print("Wind Datum: "); Serial.println(windDatum, 2);
 Serial.print("Rolling Average: "); Serial.println(rollingAverage);
  
}
// PGN direct handles

void tState::handleSeatalkPilotMode(const tN2kMsg &N2kMsg){

  if (N2kMsg.PGN != 65379)
    return ;

  int index = 0;
  unsigned char c;
  unsigned int prefix  = N2kMsg.Get2ByteUInt(index);
  unsigned int pilotMode = N2kMsg.Get2ByteUInt(index);
  
  Serial.print("Pilot Mode: ");
  Serial.print(pilotMode); 
  switch(pilotMode){
    case 0 : 
      Serial.println(" Standby");
      break;

   case 64: 
      Serial.println(" Auto (compass)");
      break;

   case 256: 
      Serial.println("  Wind");
      break;

   case 384 : 
      Serial.println(" Track");
      break;

   case 385 : 
      Serial.println(" No Drift");
      break;

    default:
      break;



  }

}

void tState::handleSeatalkLockedheading(const tN2kMsg &N2kMsg){

  if (N2kMsg.PGN != 65360)
    return ;

  int index = 0;
  unsigned char c;
  unsigned int SID  = N2kMsg.GetByte(index);
  double targetHeadingTrue = N2kMsg.Get2ByteDouble(0.0001, index);
  double targetHeadingMag = N2kMsg.Get2ByteDouble(0.0001, index);
  
  Serial.print("SID: ");
  Serial.println(SID); 
  Serial.print("Heading true: ");
  Serial.println(targetHeadingTrue / PI * 180, 1);
  Serial.print("Heading magnetic: ");
  Serial.println(targetHeadingMag / PI * 180, 1);

}

 
/* We receive this message every second.
Will use to set the RTC
SystemDate is Days since 1 January 1970.
SystemsTime is seconds since midnight.

// Probably would be better do it once for session

*/

void tState::handleSystemDateTime(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  uint16_t SystemDate;
  double SystemTime;
  tN2kTimeSource TimeSource;

  ParseN2kSystemTime(N2kMsg, SID, SystemDate,
                     SystemTime, TimeSource);

  // Compute the tiome since epoch

  Serial.print("SystemDate: "); Serial.println(SystemDate);
  Serial.print("SystemTime: "); Serial.println(SystemTime);
  switch (TimeSource)
  {
    case N2ktimes_GPS:
      Serial.println("Source: GPS");
      break;
      
    case N2ktimes_GLONASS:
      Serial.println("Source: GLONASS");
      break;

    case N2ktimes_RadioStation:
      Serial.println("Source: RadioStation");
      break;

    case N2ktimes_LocalCesiumClock:
      Serial.println("Source: LocalCesiumClock");
      break;

    case N2ktimes_LocalRubidiumClock:
      Serial.println("Source: LocalRubidiumClock");
      break;


    case N2ktimes_LocalCrystalClock:

      Serial.println("Source: LocalCrystalClock");
      break;

    default:
      Serial.println("Source: Unknown");

  }

}

void tState::handleProductInfo(const tN2kMsg &N2kMsg)
{
unsigned short N2kVersion;
unsigned short ProductCode;
int ModelIDSize;
char *ModelID;
int SwCodeSize;
char *SwCode;
int ModelVersionSize;
char *ModelVersion;
int ModelSerialCodeSize;
char *ModelSerialCode;
unsigned char CertificationLevel;
unsigned char LoadEquivalency;

  if (ParseN2kPGN126996(N2kMsg, N2kVersion, ProductCode,
                     ModelIDSize, ModelID, SwCodeSize, SwCode,
                     ModelVersionSize, ModelVersion, ModelSerialCodeSize, ModelSerialCode,
                     CertificationLevel, LoadEquivalency))
  {
    Serial.print("N2K Version: "); Serial.println(N2kVersion);
    Serial.print("Product Code: "); Serial.println(ProductCode);
    Serial.print("Model ID: "); Serial.println(ModelID);
    Serial.print("Software Code: "); Serial.println(SwCode);
    Serial.print("Model Version: "); Serial.println(ModelVersion);
    Serial.print("Model Serial Code: "); Serial.println(ModelSerialCode);
    Serial.print("Certification Level: "); Serial.println(CertificationLevel);
    Serial.print("Load Equivalency: "); Serial.println(LoadEquivalency);
    
  }
} 

// MOB message (127233)
void tState::handleMOB(const tN2kMsg &N2kMsg){

  unsigned char SID;
  uint32_t MobEmitterId;
  tN2kMOBStatus MOBStatus;
  double ActivationTime;
  tN2kMOBPositionSource PositionSource;
  uint16_t PositionDate;
  double PositionTime;
  double Latitude;
  double Longitude;
  tN2kHeadingReference COGReference;
  double COG;
  double SOG;
  uint32_t MMSI;
  tN2kMOBEmitterBatteryStatus MOBEmitterBatteryStatus;

  if(ParseN2kMOBNotification(N2kMsg,SID,MobEmitterId,MOBStatus,ActivationTime,PositionSource,PositionDate,PositionTime,Latitude,Longitude,COGReference,COG,SOG,MMSI,MOBEmitterBatteryStatus))
  {
    Serial.print("MOB Emitter ID: "); Serial.println(MobEmitterId);
    Serial.print("MOB Status: "); Serial.println(MOBStatus);
    Serial.print("Activation Time: "); Serial.println(ActivationTime);
    Serial.print("Position Source: "); Serial.println(PositionSource);
    Serial.print("Position Date: "); Serial.println(PositionDate);
    Serial.print("Position Time: "); Serial.println(PositionTime);
    Serial.print("Latitude: "); Serial.println(Latitude);
    Serial.print("Longitude: "); Serial.println(Longitude);
    Serial.print("COG Reference: "); Serial.println(COGReference);
    Serial.print("COG: "); Serial.println(COG);
    Serial.print("SOG: "); Serial.println(SOG);
    Serial.print("MMSI: "); Serial.println(MMSI);
    Serial.print("MOB Emitter Battery Status: "); Serial.println(MOBEmitterBatteryStatus);

  }

}


// Heading and Track Control.  We only use VesselHeading
void tState::handleHeadingTrackControl(const tN2kMsg &N2kMsg)
{
  tN2kOnOff RudderLimitExceeded;
  tN2kOnOff OffHeadingLimitExceeded;
  tN2kOnOff OffTrackLimitExceeded;
  tN2kOnOff Override;
  tN2kSteeringMode SteeringMode;
  tN2kTurnMode TurnMode;
  tN2kHeadingReference HeadingReference;
  tN2kRudderDirectionOrder CommandedRudderDirection;
  double CommandedRudderAngle;
  double HeadingToSteerCourse;
  double Track;
  double RudderLimit;
  double OffHeadingLimit;
  double RadiusOfTurnOrder;
  double RateOfTurnOrder;
  double OffTrackLimit;
  double VesselHeading;

  if (ParseN2kHeadingTrackControl(N2kMsg, RudderLimitExceeded, OffHeadingLimitExceeded, OffTrackLimitExceeded, Override, SteeringMode,
                                  TurnMode, HeadingReference, CommandedRudderDirection, CommandedRudderAngle, HeadingToSteerCourse, Track, RudderLimit,
                                  OffHeadingLimit, RadiusOfTurnOrder, RateOfTurnOrder, OffTrackLimit, VesselHeading))

  {

    Serial.print("Rudder limit exceeded: ");
    Serial.println(onOffValues[RudderLimitExceeded]);
    Serial.print("Off heading limit exceeded: ");
    Serial.println(onOffValues[OffHeadingLimitExceeded]);
    Serial.print("Off Track limit exceeded: ");
    Serial.println(onOffValues[OffTrackLimitExceeded]);
    Serial.print("Override: ");
    Serial.println(onOffValues[Override]);

    Serial.print("Steering Mode: ");
    Serial.println(steeringModeValues[SteeringMode]);
    Serial.print("Turn Mode: ");
    Serial.println(turnModeValues[TurnMode]);
    Serial.print("Heading Reference: ");
    Serial.println(headingReferenceValues[HeadingReference]);
    Serial.print("Rudder Direction: ");
    Serial.println(rudderDirectionValues[CommandedRudderDirection]);

    Serial.print("Command Rudder Angle: ");
    Serial.println(CommandedRudderAngle);
    Serial.print("Heading to steer course: ");
    Serial.println(HeadingToSteerCourse / PI * 180);

    Serial.print("Track: ");
    Serial.println(Track);

    Serial.print("Rudder limit: ");
    Serial.println(RudderLimit);

    Serial.print("Off Heading limit: ");
    Serial.println(OffHeadingLimit);

    Serial.print("Radius of Turn Order: ");
    Serial.println(RadiusOfTurnOrder);

    Serial.print("Rate of Turn Order: ");
    Serial.println(RateOfTurnOrder);

    Serial.print("Off Track limit: ");
    Serial.println(OffTrackLimit);
    Serial.print("Vessel heading: ");
    Serial.println(VesselHeading / PI * 180);
  }
}


// Rudder command is not used for the moment
void tState::handleRudderCommand(const tN2kMsg &N2kMsg)
{

  double RudderPosition;
  unsigned char Instance;
  tN2kRudderDirectionOrder RudderDirectionOrder;
  double AngleOrder;
  if (ParseN2kRudder(N2kMsg, RudderPosition, Instance, RudderDirectionOrder, AngleOrder)){
    Serial.print("Rudder Position: ");
    Serial.println(RudderPosition);
    Serial.print("Instance: ");
    Serial.println(Instance);
    Serial.print("Rudder Direction Order: ");
    Serial.println(rudderDirectionValues[RudderDirectionOrder]);
    Serial.print("Angle Order: ");
    Serial.println(AngleOrder);
  }  
}

void tState::handleBatteryStatus(const tN2kMsg &N2kMsg)
{
unsigned char BatteryInstance;
double BatteryVoltage;
double BatteryCurrent;
double BatteryTemperature;
unsigned char SID;

  if (ParseN2kDCBatStatus(N2kMsg, BatteryInstance, BatteryVoltage, BatteryCurrent, BatteryTemperature, SID))
  {
    Serial.print("Battery Instance: "); Serial.println(BatteryInstance);
    Serial.print("Battery Voltage: "); Serial.println(BatteryVoltage);
    Serial.print("Battery Current: "); Serial.println(BatteryCurrent);
    Serial.print("Battery Temperature: "); Serial.println(BatteryTemperature);
    Serial.print("SID: "); Serial.println(SID); 
  }
}

// Navigation Info. Not used in Logbook for the moment

void tState::handleNavigationInfo(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double DistanceToWaypoint;
  tN2kHeadingReference BearingReference;
  bool PerpendicularCrossed;
  bool ArrivalCircleEntered;
  tN2kDistanceCalculationType CalculationType;
  double ETATime;
  int16_t ETADate;
  double BearingOriginToDestinationWaypoint;
  double BearingPositionToDestinationWaypoint;
  uint32_t OriginWaypointNumber;
  uint32_t DestinationWaypointNumber;
  double DestinationLatitude;
  double DestinationLongitude;
  double WaypointClosingVelocity;

  if (ParseN2kNavigationInfo(N2kMsg, SID, DistanceToWaypoint, BearingReference, PerpendicularCrossed, ArrivalCircleEntered, CalculationType,
                             ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint,
                             OriginWaypointNumber, DestinationWaypointNumber, DestinationLatitude, DestinationLongitude, WaypointClosingVelocity))
  {



    Serial.print("SID: ");
    Serial.println(SID);
    Serial.print("DistanceToWaypoint: ");
    Serial.println(DistanceToWaypoint);
    Serial.print("BearingReference: ");
    Serial.println(headingReferenceValues[BearingReference]);
    Serial.print("PerpendicularCrossed: ");
    Serial.println(PerpendicularCrossed);
    Serial.print("ArrivalCircleEntered: ");
    Serial.println(ArrivalCircleEntered);
    Serial.print("CalculationType: ");
    Serial.println(CalculationType);
    Serial.print("ETATime: ");
    Serial.println(ETATime);
    Serial.print("ETADate: ");
    Serial.println(ETADate);
    Serial.print("BearingOriginToDestinationWaypoint: ");
    Serial.println(BearingOriginToDestinationWaypoint / 3.14192 * 180.0, 4);
    Serial.print("BearingPositionToDestinationWaypoint: ");
    Serial.println(BearingPositionToDestinationWaypoint / 3.14192 * 180.0, 4);
    Serial.print("OriginWaypointNumber: ");
    Serial.println(OriginWaypointNumber);
    Serial.print("DestinationWaypointNumber: ");
    Serial.println(DestinationWaypointNumber);
    Serial.print("DestinationLatitude: ");
    Serial.println(DestinationLatitude, 6);
    Serial.print("DestinationLongitude: ");
    Serial.println(DestinationLongitude, 6);
    Serial.print("WaypointClosingVelocity: ");
    Serial.println(WaypointClosingVelocity);
  }
}

// Cross Track Error. Not used in log for the moment
void tState::handleXTE(const tN2kMsg &N2kMsg)
{

  unsigned char SID;
  tN2kXTEMode XTEMode;
  bool NavigationTerminated;
  double XTE;

  if (ParseN2kXTE(N2kMsg, SID, XTEMode, NavigationTerminated, XTE))
  {
    Serial.print("SID: ");
    Serial.println(SID);
    Serial.print("XTEMode: ");
    Serial.println(xTEModeValues[XTEMode]);
    Serial.print("NavigationTerminated: ");
    Serial.println(NavigationTerminated);
    Serial.print("XTE: ");
    Serial.println(XTE);
   }
}

// Route Info. See parrseN2kPGN!29285. Not used for the moment

void tState::handleRouteInfo(const tN2kMsg &N2kMsg)
{

  uint16_t Start;
  uint16_t nItems;
  uint16_t Database;
  uint16_t Route;
  t_waypoint waypoints[10];

  tN2kNavigationDirection NavDirection;
  char RouteName[21] = "";
  tN2kGenericStatusPair SupplementaryData;

  if (ParseN2kPGN129285(N2kMsg, Start, nItems, Database, Route, NavDirection, RouteName, 20, SupplementaryData, 10, waypoints))
  {

    Serial.print("Received Route Packet (129285)");
    Serial.print(" from: ");
    Serial.println(N2kMsg.Source);
    Serial.print("Start: ");
    Serial.println(Start);
    Serial.print("nItems: ");
    Serial.println(nItems);
    Serial.print("Database: ");
    Serial.println(Database);
    Serial.print("Route: ");
    Serial.println(Route);
    Serial.print("NavDirection: ");
    Serial.println(NavDirection / PI * 180.0);
    Serial.print("RouteName: ");
    Serial.println(RouteName);
    Serial.print("SupplementaryData: ");
    Serial.println(SupplementaryData);

    for (int i = 0; i < nItems; i++)
    {
      Serial.print(".   wp ");
      Serial.print(waypoints[i].id);
      Serial.print(" ");
      Serial.print(waypoints[i].name);
      Serial.print(" Lat ");
      Serial.print(waypoints[i].latitude, 6);
      Serial.print(" Lon ");
      Serial.println(waypoints[i].longitude, 6);
    }
  }
}

// Wind

void tState::handleWind(const tN2kMsg &N2kMsg)
{
  double windSpeed;
  double windAngle;
  unsigned char SID;
  tN2kWindReference windReference;

  if(ParseN2kPGN130306(N2kMsg, SID, windSpeed, windAngle, windReference)){
    Serial.print("SID: "); Serial.println(SID);
    Serial.print("Wind Speed: "); Serial.println(windSpeed);
    Serial.print("Wind Angle: "); Serial.println(windAngle / PI * 180.0);
    Serial.print("Wind Reference: "); Serial.println(windReferenceValues[windReference]); 
  }
}



// Heading. Also updates deviation and variation

void tState::handleHeading(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double Heading;
  double Deviation;
  double Variation;
  tN2kHeadingReference ref;

  if(ParseN2kHeading(N2kMsg, SID, Heading, Deviation, Variation, ref)){
    Serial.print("SID: "); Serial.println(SID);
    Serial.print("Heading: "); Serial.println(Heading / PI * 180.0);
    Serial.print("Deviation: "); Serial.println(Deviation / PI * 180.0);
    Serial.print("Variation: "); Serial.println(Variation / PI * 180.0);
    Serial.print("Reference: "); Serial.println(headingReferenceValues[ref]); 
  }
}

// Rate Of Turn

void tState::handleRateOfTurn(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double RateOfTurn;

  if (ParseN2kRateOfTurn(N2kMsg, SID, RateOfTurn)){
    Serial.print("SID: "); Serial.println(SID);
    Serial.print("RateOfTurn: "); Serial.println(RateOfTurn); 
  }
}

// Heave, 127252
void tState::handleHeave(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
double Heave;
double Delay;
tN2kDelaySource DelaySource;

  if (ParseN2kHeave(N2kMsg, SID, Heave, Delay, DelaySource)){
    Serial.print("SID: "); Serial.println(SID);
    Serial.print("Heave: "); Serial.println(Heave / PI * 180.0); 
    Serial.print("Delay: "); Serial.println(Delay); 
    Serial.print("Delay Source: "); Serial.println(DelaySource);  
  }
}

// Attitude. Usually Yaw is the same as Heading (for a constant)
void tState::handleAttitude(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double Yaw;
  double Pitch;
  double Roll;

  if (ParseN2kAttitude(N2kMsg, SID, Yaw, Pitch, Roll)){
    Serial.print("SID: "); Serial.println(SID);
    Serial.print("Yaw: "); Serial.println(Yaw / PI * 180.0);
    Serial.print("Pitch: "); Serial.println(Pitch / PI * 180.0);
    Serial.print("Roll: "); Serial.println(Roll / PI * 180.0); 
  } 
}

// Variation

void tState::handleMagneticVariation(const tN2kMsg &N2kMsg)
{

  unsigned char SID;
  tN2kMagneticVariation Source;
  uint16_t DaysSince1970;
  double Variation;

  if (ParseN2kMagneticVariation(N2kMsg, SID, Source, DaysSince1970, Variation)){
    Serial.print("SID: "); Serial.println(SID);
    Serial.print("Source: "); Serial.println(Source);
    Serial.print("DaysSince1970: "); Serial.println(DaysSince1970);
    Serial.print("Variation: "); Serial.println(Variation / PI * 180.0);
  }
}

// Position Rapid Update

void tState::handlePositionRapidUpdate(const tN2kMsg &N2kMsg)
{

  double Latitude;
  double Longitude;
  if(ParseN2kPositionRapid(N2kMsg, Latitude, Longitude)){

    Serial.print(" Latitude: ");
    Serial.println(Latitude, 6);
    Serial.print(" Longitude: ");
    Serial.println(Longitude, 6);
  }

}

// COGSOG Rapid Update

void tState::handleCOGSOGRapidUpdate(const tN2kMsg &N2kMsg)
{

  unsigned char SID;
  tN2kHeadingReference ref;
  double COG;
  double SOG;
  if(ParseN2kCOGSOGRapid(N2kMsg, SID, ref, COG, SOG)){
    
    Serial.print("SID: "); Serial.println(SID);
    Serial.print(" Reference: ");
    Serial.println(headingReferenceValues[ref]);
    Serial.print(" COG: ");
    Serial.println(COG / PI * 180.0);
    Serial.print(" SOG: ");
    Serial.println(SOG);
    
  }

}

// GNSS . Only use position for the moment. Other data may be interesting if an accident or erroneus position

void tState::handleGNSS(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  uint16_t DaysSince1970;
  double SecondsSinceMidnight;

  double Latitude;
  double Longitude;
  double Altitude;

  tN2kGNSStype GNSStype;
  tN2kGNSSmethod GNSSmethod;

  unsigned char nSatellites;
  double HDOP;
  double PDOP;
  double GeoidalSeparation;

  unsigned char nReferenceStations;
  tN2kGNSStype ReferenceStationType;
  uint16_t ReferenceSationID;
  double AgeOfCorrection;

  if(ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
               Latitude, Longitude, Altitude,
               GNSStype, GNSSmethod,
               nSatellites, HDOP, PDOP, GeoidalSeparation,
               nReferenceStations, ReferenceStationType, ReferenceSationID,
               AgeOfCorrection)){

    Serial.print("SID: "); Serial.println(SID);
    Serial.print("DaysSince1970: "); Serial.println(DaysSince1970);
    Serial.print("SecondsSinceMidnight: "); Serial.println(SecondsSinceMidnight);
    Serial.print(" Latitude: ");
    Serial.println(Latitude, 6);
    Serial.print(" Longitude: ");
    Serial.println(Longitude, 6);
    Serial.print(" Altitude: ");
    Serial.println(Altitude, 2);
    Serial.print(" GNSStype: ");
    Serial.println(GNSStype);
    Serial.print(" GNSSmethod: ");
    Serial.println(GNSSmethod);
    Serial.print(" nSatellites: ");
    Serial.println(nSatellites);
    Serial.print(" HDOP: ");
    Serial.println(HDOP);
    Serial.print(" PDOP: ");
    Serial.println(PDOP);
    Serial.print(" GeoidalSeparation: ");
    Serial.println(GeoidalSeparation);
    Serial.print(" nReferenceStations: ");
    Serial.println(nReferenceStations);
    Serial.print(" ReferenceStationType: ");
    Serial.println(ReferenceStationType);
    Serial.print(" ReferenceSationID: ");
    Serial.println(ReferenceSationID);
    Serial.print(" AgeOfCorrection: ");
    Serial.println(AgeOfCorrection);

  }
}

// Handle rapig engine parameters

void tState::handleEngineParamRapid(const tN2kMsg &N2kMsg)
{
  unsigned char EngineInstance;
  double EngineSpeed;
  double EngineBoostPressure;
  int8_t EngineTiltTrim;

  if(ParseN2kEngineParamRapid(N2kMsg,EngineInstance,EngineSpeed,EngineBoostPressure,EngineTiltTrim)){
    Serial.print("Engine Instance: "); Serial.println(EngineInstance);
    Serial.print("Engine Speed: "); Serial.println(EngineSpeed);
    Serial.print("Engine Boost Pressure: "); Serial.println(EngineBoostPressure);
    Serial.print("Engine Tilt Trim: "); Serial.println(EngineTiltTrim);
  }

}

void tState::handleEngineDynamicParameters(const tN2kMsg &N2kMsg)
{
unsigned char EngineInstance;
double EngineOilPress;
double EngineOilTemp;
double EngineCoolantTemp;
double AltenatorVoltage;
double FuelRate;
double EngineHours;
double EngineCoolantPress;
double EngineFuelPress;
int8_t EngineLoad;
int8_t EngineTorque;

  if(ParseN2kEngineDynamicParam(N2kMsg, EngineInstance, EngineOilPress,
                      EngineOilTemp, EngineCoolantTemp, AltenatorVoltage,
                      FuelRate, EngineHours,EngineCoolantPress, EngineFuelPress,
                      EngineLoad, EngineTorque)){
      Serial.print("Engine Instance: "); Serial.println(EngineInstance);
      Serial.print("Engine Oil Pressure: "); Serial.println(EngineOilPress);
      Serial.print("Engine Oil Temperature: "); Serial.println(EngineOilTemp);
      Serial.print("Engine Coolant Temperature: "); Serial.println(EngineCoolantTemp);
      Serial.print("Alternator Voltage: "); Serial.println(AltenatorVoltage);
      Serial.print("Fuel Rate: "); Serial.println(FuelRate);
      Serial.print("Engine Hours: "); Serial.println(EngineHours);
      Serial.print("Engine Coolant Pressure: "); Serial.println(EngineCoolantPress);
      Serial.print("Engine Fuel Pressure: "); Serial.println(EngineFuelPress);
      Serial.print("Engine Load: "); Serial.println(EngineLoad);
      Serial.print("Engine Torque: "); Serial.println(EngineTorque);
                      }


} 

void tState::handleTemperature(const tN2kMsg &N2kMsg)
{
unsigned char SID;
unsigned char TempInstance;
tN2kTempSource TempSource;
double ActualTemperature;
double SetTemperature;

  if(ParseN2kTemperature(N2kMsg, SID, TempInstance, TempSource, ActualTemperature, SetTemperature)){
    Serial.print("SID: "); Serial.println(SID);
    Serial.print("Temp Instance: "); Serial.println(TempInstance);
    Serial.print("Temp Source: "); Serial.println(temperatureSourceValues[TempSource]);
    Serial.print("Actual Temperature: "); Serial.println(ActualTemperature);
    Serial.print("Set Temperature: "); Serial.println(SetTemperature);  
  }
}
void tState::HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
  if (!trackData) {
    return;
  } 
  
  if(!checkSource(N2kMsg.Source) || !checkPGN(N2kMsg.PGN)){ // So we are not inundated!!!!
    return;
  } 
    

    Serial.print("- PGN ");
    Serial.print(N2kMsg.PGN);
    Serial.print(" ");
    Serial.print(toStringPgn(N2kMsg.PGN));
    Serial.print(" from: ");
    Serial.println(N2kMsg.Source);


  switch (N2kMsg.PGN)
  {
    case 65345:
      handleWindDatum(N2kMsg);
      break;

    case 65360:
      handleSeatalkLockedheading(N2kMsg);
      break;

    case 65379:
      handleSeatalkPilotMode(N2kMsg);
      break;

  case 126992:
    handleSystemDateTime(N2kMsg);
    break;
  
    case 126996:
    // Serial.println("Received Product Info (126996)");
    handleProductInfo(N2kMsg);
    break;  

    case 127237:  
    handleHeadingTrackControl(N2kMsg);
    break;

  case 127245:
    // Serial.println("Received rudder angle info (127245  )");
    handleRudderCommand(N2kMsg);
    break;

  case 127250:
    handleHeading(N2kMsg);
    break;

  case 127251:
    handleRateOfTurn(N2kMsg);
    break;

   case 127252:
    handleHeave(N2kMsg);
    break;
 
  case 127257:
    handleAttitude(N2kMsg);
    break;

  case 127258:
    handleMagneticVariation(N2kMsg);
    break;

  case 127488:
    handleEngineParamRapid(N2kMsg);
    break;

  case 127489:
    handleEngineDynamicParameters(N2kMsg);
    break;
    
  case 127508:
    handleBatteryStatus(N2kMsg);
    break;

  case 128259:
    // Serial.println("Received water speed (127259)");
    break;

  case 129025:
    handlePositionRapidUpdate(N2kMsg);
    break;

  case 129026:
    handleCOGSOGRapidUpdate(N2kMsg);
    break;

  case 129029:
    handleGNSS(N2kMsg);
    break;

  case 129283:
    handleXTE(N2kMsg);
    break;

  case 129284:
    handleNavigationInfo(N2kMsg);
    break;

  case 129285:
    handleRouteInfo(N2kMsg);
    break;

  case 130306:
    handleWind(N2kMsg);
    break;

  case 130312:
    handleTemperature(N2kMsg);
    break;

  default:
    Serial.print(" Unhandled PGN "); Serial.println(N2kMsg.PGN);
  }
  Serial.println("-----------------------------------");
}
