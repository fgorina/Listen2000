/* State Definition */

#ifndef _State_H_
#define _State_H_

#include "NMEA2000.h"
#include "N2kTypes.h"
#include "PyTypes.h"

 #define MAX_SOURCES 256
 #define MAX_PGNS 30
class tState
{
protected:


    bool sources[MAX_SOURCES];
    unsigned long pgns[MAX_SOURCES];     

    void handleSystemDateTime(const tN2kMsg &N2kMsg);
    void handleProductInfo(const tN2kMsg &N2kMsg);
    void handleMOB(const tN2kMsg &N2kMsg);
    void handleHeadingTrackControl(const tN2kMsg &N2kMsg);
    void handleNavigationInfo(const tN2kMsg &N2kMsg);
    void handleXTE(const tN2kMsg &N2kMsg);
    void handleRouteInfo(const tN2kMsg &N2kMsg);
    void handleWind(const tN2kMsg &N2kMsg);
    void handleRudderCommand(const tN2kMsg &N2kMsg);
    void handleHeading(const tN2kMsg &N2kMsg);
    void handleRateOfTurn(const tN2kMsg &N2kMsg);
    void handleHeave(const tN2kMsg &N2kMsg);
    void handleAttitude(const tN2kMsg &N2kMsg);
    void handleMagneticVariation(const tN2kMsg &N2kMsg);
    void handleBatteryStatus(const tN2kMsg &N2kMsg);
    void handleTemperature(const tN2kMsg &N2kMsg);
    void handlePositionRapidUpdate(const tN2kMsg &N2kMsg);
    void handleCOGSOGRapidUpdate(const tN2kMsg &N2kMsg);
    void handleGNSS(const tN2kMsg &N2kMsg);
    void handleEngineParamRapid(const tN2kMsg &N2kMsg);
    void handleEngineDynamicParameters(const tN2kMsg &N2kMsg);

    bool ParseN2kPGN129285(const tN2kMsg &N2kMsg, uint16_t &Start, uint16_t &nItems, uint16_t &Database, uint16_t &Route,
                                   tN2kNavigationDirection &NavDirection, char *RouteName, size_t RouteNameBufSize, tN2kGenericStatusPair &SupplementaryData,
                                   uint16_t wptArraySize, t_waypoint *waypoints);


   
public:  
   
    bool trackData = true;

    void initArrays();
    void unfollowSource(int source);
    void followSource(int source);
    bool checkSource(int source);
    void clearAllSources();

    void followPGN(unsigned long  pgn);
    void unfollowPGN(unsigned long pgn);
    bool checkPGN(unsigned long pgn);
    void clearAllPGNs();
    unsigned long pgnAt(int i){ return pgns[i]; }

    void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
    
};

#endif