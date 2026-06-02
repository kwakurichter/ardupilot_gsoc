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

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_SwarmMesh::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Communication backend
    // @Description: Which communication backend are you using
    // @Values: 0:None,1:Serial,10:SITL
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE",    0, AP_SwarmMesh, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _STREAM
    // @DisplayName: Stream type
    // @Description: Stream type
    // @Values: 0:None,1:Lite,2:Full
    // @User: Advanced
    AP_GROUPINFO("_STREAM", 1, AP_SwarmMesh, stream, 0),

    // @Param: _LITE_MASK
    // @DisplayName: Bitmask for Lite stream
    // @Description: 
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("_LITE_MASK", 2, AP_SwarmMesh, lite_mask, 0),

    // @Param: _FULL_MASK
    // @DisplayName: Bitmask for Full stream
    // @Description:
    // @Units: m
    // @Increment: 1
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("_FULL_MASK", 3, AP_SwarmMesh, full_mask, 0),

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