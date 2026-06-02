#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_SwarmMesh_config.h"

#define LOG_IDS_FROM_SWARMMESH \
    LOG_SWARMMESH_MSG,         \
    LOG_SWARMMESH_HB_MSG,      \
    LOG_SWARMMESH_SS_MSG,      \
    LOG_SWARMMESH_GP_MSG,      \
    LOG_SWARMMESH_LP_MSG
    // TODO: Add more log types

// @LoggerMessage: SMST
// @Description: SwarmMesh connection stats
// @Field: TimeUS: Time since system startup
// @Field: CRCFail: RX packets which failed header CRC
// @Field: Stale: RX packets passed deadline threshold
// @Field: TTL: RX packets forwarding expired
// @Field: Dedup: RX packet duplicate count
// @Field: Drop: RX packets dropped
// @Field: TXseq: TX packets sent (original)
// @Field: TXfwd: TX packets sent (forwarded)

struct PACKED log_SwarmMesh {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t crc_fail;
    uint16_t stale;
    uint16_t ttl;
    uint16_t dedup;
    uint16_t drop;
    uint16_t txseq;
    uint16_t txfwd;
    uint16_t txdrop;
};

// @LoggerMessage: SMHB
// @Description: SwarmMesh RX heartbeats
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: VType: Vehicle type
// @Field: Mode: Mode
// @Field: Arm: Armed state

struct PACKED log_SwarmMesh_HB {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    uint8_t  vehicle_type;
    uint8_t  mode;
    uint8_t  armed_state;
};

// @LoggerMessage: SMSS
// @Description: SwarmMesh RX system status
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: BVol: Battery voltage

struct PACKED log_SwarmMesh_SS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    uint16_t bat_voltage;
};

// @LoggerMessage: SMGP
// @Description: SwarmMesh RX global position
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: Lat: Latitude in degE7
// @Field: Lon: Longitude in degE7
// @Field: Alt: Altitude above MSL in mm

struct PACKED log_SwarmMesh_GP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    int32_t  lat;
    int32_t  lon;
    int32_t  alt;
};

// @LoggerMessage: SMLP
// @Description: SwarmMesh RX local position
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: x: x distance in m
// @Field: y: y distance in m
// @Field: z: z distance in m

struct PACKED log_SwarmMesh_LP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    float    x;
    float    y;
    float    z;
};

#if AP_SWARMMESH_ENABLED
#define LOG_STRUCTURE_FROM_SWARMMESH \
    { LOG_SWARMMESH_MSG, sizeof(log_SwarmMesh), \
        "SMST", "QHHHHHHHH",  "TimeUS,CRCFail,Stale,TTL,Dedup,Drop,TXseq,TXfwd,TXdrop", "s--------", "F--------", true },  \
    { LOG_SWARMMESH_HB_MSG, sizeof(log_SwarmMesh_HB), \
        "SMHB", "QBBBB",  "TimeUS,SysID,VType,Mode,Arm", "s----", "F----", true },  \
    { LOG_SWARMMESH_SS_MSG, sizeof(log_SwarmMesh_SS), \
        "SMSS", "QBH",  "TimeUS,SysID,BVol", "s--", "F--", true },  \
    { LOG_SWARMMESH_GP_MSG, sizeof(log_SwarmMesh_GP), \
        "SMGP", "QBLLi",  "TimeUS,SysID,Lat,Lon,Alt", "s----", "F----", true },  \
    { LOG_SWARMMESH_LP_MSG, sizeof(log_SwarmMesh_LP), \
        "SMLP", "QBfff",  "TimeUS,SysID,x,y,z", "s-mmm", "F-000", true },  \                                 
#else
#define LOG_STRUCTURE_FROM_SWARMMESH
#endif