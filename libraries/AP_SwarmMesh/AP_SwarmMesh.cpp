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

#include "AP_SwarmMesh.h"

#if AP_SWARMMESH_ENABLED

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_SERIAL_ENABLED
#include "AP_SwarmMesh_Serial.h"
#endif

#if AP_SWARMMESH_SITL_ENABLED
#include "AP_SwarmMesh_SITL.h"
#endif

#include <AP_Logger/AP_Logger.h>

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
#include "AP_SwarmMesh_PeerStorage.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#endif

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_SwarmMesh::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Communication backend
    // @Description: Which communication backend are you using
    // @Values: 0:None,1:Serial,10:SITL
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_SwarmMesh, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SR_POSITION
    // @DisplayName: Position stream rate
    // @Description: Rate at which GLOBAL_POSITION_INT and LOCAL_POSITION_NED are broadcast (Hz). 0 disables.
    // @Units: Hz
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("SR_POSITION", 1, AP_SwarmMesh, stream_rate[0], 0),

    // @Param: SR_EXT_STAT
    // @DisplayName: Extended status stream rate
    // @Description: Rate at which SYS_STATUS, NAV_CONTROLLER_OUTPUT, POSITION_TARGET_GLOBAL_INT and MISSION_CURRENT are broadcast (Hz). 0 disables.
    // @Units: Hz
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("SR_EXT_STAT", 2, AP_SwarmMesh, stream_rate[1], 0),

    // @Param: SR_EXTRA1
    // @DisplayName: Extra 1 stream rate
    // @Description: Rate at which ATTITUDE and EKF_STATUS_REPORT are broadcast (Hz). 0 disables.
    // @Units: Hz
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("SR_EXTRA1", 3, AP_SwarmMesh, stream_rate[2], 0),

    // @Param: _SWARM_SIZE
    // @DisplayName: Swarm size
    // @Description: Size of swarm (peers + GCS)
    // @Increment: 1
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_SWARM_SIZE", 4, AP_SwarmMesh, swarm_size, 0),

    // @Param: _DESTID
    // @DisplayName: Destination ID
    // @Description: SysID of intended destination for transmitted messages
    // @Increment: 1
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_DESTID", 5, AP_SwarmMesh, destination_id, 0),

    // @Param: _SYSID
    // @DisplayName: System ID
    // @Description: Unique system ID of this drone
    // @Increment: 1
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_SYSID", 6, AP_SwarmMesh, sysid, 0),

    // @Param: _TTL
    // @DisplayName: Time-to-Live
    // @Description: Number of hops a forwarded packet can take before being discarded
    // @Increment: 1
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_TTL", 7, AP_SwarmMesh, ttl, 255),

    // @Param: _HW_MASK
    // @DisplayName: Radio hardware capability
    // @Description: Bitmask describing the radio hardware attached. Bit 0: Full capacity radio. If clear, Lite radio assumed. The FC CPU class may further restrict to Lite regardless of radio.
    // @Bitmask: 0:Full radio
    // @User: Advanced
    AP_GROUPINFO("_HW_MASK", 8, AP_SwarmMesh, hardware_mask, 0),

    // @Param: _LOG_HZ
    // @DisplayName: RX log write rate
    // @Description: Maximum combined rate at which RX peer telemetry is written to the dataflash log, across all peers and message types. 0 disables RX logging entirely.
    // @Units: Hz
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("_LOG_HZ", 9, AP_SwarmMesh, log_rate_hz, 50),

    // @Param: _LOG_MASK
    // @DisplayName: RX log message mask
    // @Description: Bitmask of which RX message types are written to the dataflash log (still subject to LOG_HZ). Bits 8-31 are reserved for future message types.
    // @Bitmask: 0:Heartbeat,1:SysStatus,2:GlobalPositionInt,3:LocalPositionNED,4:PositionTargetGlobalInt,5:ExtendedSysState,6:Attitude,7:EkfStatusReport
    // @User: Advanced
    AP_GROUPINFO("_LOG_MASK", 10, AP_SwarmMesh, log_mask, 0xFF),

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    // @Param: _SAVE_HZ
    // @DisplayName: Peer snapshot save rate
    // @Description: Rate at which the current peer table (filled and fresh entries only) is rewritten to APM/PEERS/peers.dat on the SD card, so it can be reloaded after a reset. 0 disables.
    // @Units: Hz
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("_SAVE_HZ", 11, AP_SwarmMesh, save_rate_hz, 1),
#endif

    AP_GROUPEND
};

AP_SwarmMesh::AP_SwarmMesh()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_SwarmMesh must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_SwarmMesh class
void AP_SwarmMesh::init(void)
{
    if (_driver != nullptr) {
        // init called a 2nd time?
        return;
    }

    // create backend
    switch ((Type)_type) {
    case Type::Serial:
#if AP_SWARMMESH_SERIAL_ENABLED
        _driver = NEW_NOTHROW AP_SwarmMesh_Serial(*this);
#endif
        break;
#if AP_SWARMMESH_SITL_ENABLED
    case Type::SITL:
        _driver = NEW_NOTHROW AP_SwarmMesh_SITL(*this);
        break;
#endif
    case Type::None:
        break;
    }
}

// return true if swarm feature is enabled
bool AP_SwarmMesh::enabled(void) const
{
    return (_type != Type::None);
}

// return true if radio is basically healthy (we are receiving data)
bool AP_SwarmMesh::healthy(void) const
{
    if (!device_ready()) {
        return false;
    }
    return _driver->healthy();
}

// update state. This should be called often from the main loop
void AP_SwarmMesh::update(void)
{
    if (!device_ready()) {
        return;
    }
    _driver->update();

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    const uint8_t rate_hz = MAX(0, (int8_t)save_rate_hz);
    if (rate_hz != 0) {
        const uint32_t now_ms = AP_HAL::millis();
        const uint32_t interval_ms = 1000U / ((rate_hz <= 10) ? rate_hz : 10);  // Max snapshot rate 10Hz
        if (now_ms - _last_save_ms >= interval_ms) {
            _last_save_ms = now_ms;
            save_peer_snapshot();
        }
    }
#endif
}

// return the number of peers
uint8_t AP_SwarmMesh::count() const
{
    if (!device_ready()) {
        return 0;
    }
    return num_peers;
}

// return all peer data
bool AP_SwarmMesh::get_peer_data(uint8_t peer_id, struct PeerState& state) const
{
    if (!device_ready() || peer_id >= num_peers) {
        return false;
    }
    state = peer_state[peer_id];
    return true;
}

// check if the device is ready
bool AP_SwarmMesh::device_ready(void) const
{
    return ((_driver != nullptr) && (_type != Type::None));
}

// find an existing peer entry by sysid, or allocate a new zeroed entry.
// returns nullptr if the table is full and the peer is not already present.
AP_SwarmMesh::PeerState *AP_SwarmMesh::find_or_alloc_peer(uint8_t peer_sysid)
{
    // respect swarm_size if set, otherwise fall back to compile-time max
    const uint8_t limit = (swarm_size > 0) ? MIN((uint8_t)swarm_size, (uint8_t)AP_SWARMMESH_MAX_PEERS) : AP_SWARMMESH_MAX_PEERS;

    for (uint8_t i = 0; i < num_peers; i++) {
        if (peer_state[i].sysid == peer_sysid) {
            return &peer_state[i];
        }
    }
    if (num_peers >= limit) {
        return nullptr;
    }
    PeerState &ps = peer_state[num_peers++];
    memset(&ps, 0, sizeof(ps));
    ps.sysid = peer_sysid;
    return &ps;
}

// periodically rewrite the on-disk peer-table snapshot (filled and fresh entries only).
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
void AP_SwarmMesh::save_peer_snapshot()
{
    if (!_save_dir_checked) {
        _save_dir_checked = true;
        EXPECT_DELAY_MS(3000);
        struct stat st;
        int ret = AP::FS().stat(AP_SWARMMESH_PEER_DIR, &st);
        if (ret == -1) {
            ret = AP::FS().mkdir(AP_SWARMMESH_PEER_DIR);
        }
        _save_dir_ok = (ret == 0) || (ret == -1 && errno == EEXIST);
        if (!_save_dir_ok) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SwarmMesh: failed to create %s", AP_SWARMMESH_PEER_DIR);
        }
    }
    if (!_save_dir_ok) {
        return;
    }

    // count eligible (filled + fresh) peers first, since the header needs the count up front and we don't want to buffer all peers on the stack
    uint16_t eligible = 0;
    for (uint8_t i = 0; i < num_peers; i++) {
        if (peer_state[i].freshness) {
            eligible++;
        }
    }

    EXPECT_DELAY_MS(3000);
    const int fd = AP::FS().open(AP_SWARMMESH_PEER_FILE, O_WRONLY | O_CREAT | O_TRUNC);
    if (fd == -1) {
        return;
    }

    const AP_SwarmMesh_PeerFileHeader_t hdr{
        magic          : AP_SWARMMESH_PEER_FILE_MAGIC,
        version        : AP_SWARMMESH_PEER_FILE_VERSION,
        snapshot_size  : sizeof(AP_SwarmMesh_PeerSnapshot_t),
        snapshot_count : eligible,
        saved_time_us  : AP_HAL::micros64()
    };
    AP::FS().write(fd, &hdr, sizeof(hdr));

    for (uint8_t i = 0; i < num_peers; i++) {
        const PeerState &ps = peer_state[i];
        if (!ps.freshness) {
            continue;
        }
        const AP_SwarmMesh_PeerSnapshot_t rec{
            sysid           : ps.sysid,
            vehicle_type    : ps.vehicle_type,
            mode            : ps.mode,
            armed_state     : (uint8_t)ps.armed_state,
            landed_state    : ps.landed_state,
            failsafe_flags  : ps.failsafe_flags,
            battery_voltage : ps.battery_voltage,
            local_pos_NED   : { ps.local_pos_NED.x, ps.local_pos_NED.y, ps.local_pos_NED.z },
            global_pos      : { ps.global_pos.x, ps.global_pos.y, ps.global_pos.z },
            attitude        : { ps.attitude.x, ps.attitude.y, ps.attitude.z },
            role            : ps.role,
            task_id         : ps.task_id,
            formation_slot  : ps.formation_slot,
            target_pos      : { ps.target_pos.x, ps.target_pos.y, ps.target_pos.z },
            priority        : ps.priority
        };
        AP::FS().write(fd, &rec, sizeof(rec));
    }

    AP::FS().close(fd);
}
#endif  // AP_FILESYSTEM_FILE_WRITING_ENABLED

#if HAL_LOGGING_ENABLED
void AP_SwarmMesh::log()
{
    if (!device_ready()) {
        return;
    }
    _driver->log_stats();
}
#endif

// singleton instance
AP_SwarmMesh *AP_SwarmMesh::_singleton;

namespace AP {

AP_SwarmMesh *swarmmesh()
{
    return AP_SwarmMesh::get_singleton();
}

}

#endif