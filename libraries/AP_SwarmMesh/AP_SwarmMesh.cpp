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
    // @Values: 0:None,1:MAVLink,10:SITL
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
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_SWARM_SIZE", 4, AP_SwarmMesh, swarm_size, 0),

    // @Param: _DESTID
    // @DisplayName: Destination ID
    // @Description: SysID of intended destination for transmitted messages
    // @Increment: 1
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_DESTID", 5, AP_SwarmMesh, destination_id, 0),

    // @Param: _SYSID
    // @DisplayName: System ID
    // @Description: Unique system ID of this drone
    // @Increment: 1
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_SYSID", 6, AP_SwarmMesh, sysid, 0),

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
        // TODO: instantiate AP_SwarmMesh_Serial once implemented
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

#if HAL_LOGGING_ENABLED
// Write incoming peer data
void AP_SwarmMesh::log()
{
    if (!enabled()) {
        return;
    }
    // position
    //Vector3f pos;
    //float accuracy = 0.0f;
    //get_vehicle_position_ned(pos, accuracy);

    //const struct log_Beacon pkt_beacon{
    //   LOG_PACKET_HEADER_INIT(LOG_BEACON_MSG),
    //   time_us         : AP_HAL::micros64(),
    //   health          : (uint8_t)healthy(),
    //   count           : (uint8_t)count(),
    //   dist0           : beacon_distance(0),
    //   dist1           : beacon_distance(1),
    //   dist2           : beacon_distance(2),
    //   dist3           : beacon_distance(3),
    //   posx            : pos.x,
    //   posy            : pos.y,
    //   posz            : pos.z
    //};
    //AP::logger().WriteBlock(&pkt_beacon, sizeof(pkt_beacon));
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