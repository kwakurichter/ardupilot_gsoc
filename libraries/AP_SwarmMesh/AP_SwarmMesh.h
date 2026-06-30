/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_SwarmMesh_config.h"

#if AP_SWARMMESH_ENABLED
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

class AP_SwarmMesh_Backend;

class AP_SwarmMesh
{
public:
    friend class AP_SwarmMesh_Backend;

    AP_SwarmMesh();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_SwarmMesh);

    // get singleton instance
    static AP_SwarmMesh *get_singleton() { return _singleton; }

    // external hardware backend types (used by _TYPE parameter)
    enum class Type : uint8_t {
        None    = 0,
        Serial  = 1,
#if AP_SWARMMESH_SITL_ENABLED
        SITL    = 10
#endif
    };

    // The AP_SwarmMesh structure is filled in by the backend driver
    struct PeerState {
        // Peer identity
        uint8_t  sysid;         // unique ID of original peer
        uint8_t  vehicle_type;  // 0: copter, 1: plane, 2: sub, 3: blimp, 4: rover
        uint8_t  prev_id;       // ID of peer which forwarded message
        // Liveness / Link quality
        uint64_t last_heard;    // system time of last update from this peer for staleness detection (unix)
        uint16_t last_seq;      // for dedup ring buffer
        uint32_t seq_seen_mask; // bitmask of the 32 seq numbers behind last_seq
        uint8_t  rssi;          // signal strength
        uint16_t rx_count;      // received message count
        uint16_t drop_count;    // dropped message count
        bool     freshness;     // true: FRESH, false: STALE
        // Kinematic state
        Vector3f local_pos_NED; // offset from origin [x, y, z] in meters
        Vector3f global_pos;    // GPS [lat (degE7), lon (degE7). alt (mm)]
        float    pos_covariance[9];
        Vector3f attitude;      // [pitch, roll, yaw] in rads
        float    att_covariance[9];
        // Vehicle state
        uint8_t  mode;
        bool     armed_state;   // true: armed, false: disarmed
        uint8_t  landed_state;  // MAV_LANDED_STATE: 0 undefined, 1 on ground, 2 in air, 3 takeoff, 4 landing
        uint32_t failsafe_flags; // bitmask of unhealthy sensors (present & enabled & !health), MAV_SYS_STATUS_SENSOR bits
        uint16_t battery_voltage;
        uint8_t  health_flags;
        // Coordination state
        uint8_t  role;
        uint8_t  task_id;
        uint8_t  formation_slot;
        Vector3f target_pos;    // [lat (degE7), lon (degE7). alt (mm)]
        uint8_t  priority;
    };

    // initialise
    void init(void);

    // return true if mesh feature is enabled
    bool enabled(void) const;

    // return true if mesh is basically healthy (we are receiving data)
    bool healthy(void) const;

    // update state of all peers
    void update(void);

    // return number of known peers
    uint8_t count() const;

    // return data for a specific peer by index
    bool get_peer_data(uint8_t peer_id, struct PeerState& state) const;

    static const struct AP_Param::GroupInfo var_info[];

    // a method for vehicles to call to make onboard log messages:
    void log();

    // number of SR stream buckets (must match the SR_* param entries in var_info)
    static constexpr uint8_t NUM_BUCKETS = 3;

private:

    // return true if driver is instantiated and type is not None
    bool device_ready(void) const;

    // find an existing peer entry by sysid, or allocate a new one.
    // returns nullptr if the table is full and the peer is not already present.
    PeerState *find_or_alloc_peer(uint8_t peer_sysid);

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    // periodically rewrite the on-disk peer-table snapshot (filled + fresh entries only)
    void save_peer_snapshot();

    // called once from init(): restore the peer table from the on-disk snapshot, if one exists and is valid. Restored peers always start with freshness == false.
    void load_peer_snapshot();
#endif

    // periodically deletes stale peer table entries
    void prune_peer_table();

    static AP_SwarmMesh *_singleton;

    // parameters
    AP_Enum<Type> _type;
    AP_Int8  stream_rate[NUM_BUCKETS];  // SR_POSITION, SR_EXT_STAT, SR_EXTRA1 (Hz; 0 = disabled)
    AP_Int8  swarm_size;
    AP_Int8  destination_id;
    AP_Int8  sysid;
    AP_Int8  ttl;
    AP_Int8  hardware_mask;
    AP_Int16 log_rate_hz;  // max combined rate (Hz) of RX dataflash log writes; 0 disables
    AP_Int32 log_mask;     // bitmask of which RX message types are logged (see AP_SwarmMesh_Serial::LogMsg)
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    AP_Int8  save_rate_hz; // rate (Hz) at which the on-disk peer snapshot is rewritten; 0 disables
#endif
    AP_Int8  prune_timeout;

    // external references
    AP_SwarmMesh_Backend *_driver;

    // individual peer data
    uint8_t num_peers = 0;
    PeerState peer_state[AP_SWARMMESH_MAX_PEERS];

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    uint32_t _last_save_ms;     // last time the peer snapshot was written
    bool     _save_dir_checked; // true once we've attempted to create AP_SWARMMESH_PEER_DIR
    bool     _save_dir_ok;      // true if the directory exists/was created successfully
#endif
    uint32_t _last_check_ms;    // last time the peer table was checked for pruning
};

namespace AP {
    AP_SwarmMesh *swarmmesh();
};

#endif  // AP_SWARMMESH_ENABLED
