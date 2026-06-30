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

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_ENABLED

// debug
#include <stdio.h>
#include <AP_SerialManager/AP_SerialManager.h>

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_SwarmMesh_Backend::AP_SwarmMesh_Backend(AP_SwarmMesh &frontend) :
    _frontend(frontend)
{
    const AP_SerialManager &serialmanager = AP::serialmanager();
    uart = serialmanager.find_serial(AP_SerialManager::SerialProtocol_SwarmMesh, 0);
    if (uart == nullptr) {
        return;
    }

    uart->begin(serialmanager.find_baudrate(AP_SerialManager::SerialProtocol_SwarmMesh, 0));
}

// sysid accessor
uint8_t AP_SwarmMesh_Backend::frontend_sysid() const
{
    return (uint8_t)_frontend.sysid;
}

// destination_id accessor
uint8_t AP_SwarmMesh_Backend::frontend_dest_id() const
{
    return (uint8_t)_frontend.destination_id;
}

// ttl accessor
uint8_t AP_SwarmMesh_Backend::frontend_ttl() const
{
    return (uint8_t)_frontend.ttl;
}

// find or alloc a peer entry in the frontend table by sysid
AP_SwarmMesh::PeerState *AP_SwarmMesh_Backend::frontend_peerstate(uint8_t peer_sysid)
{
    return _frontend.find_or_alloc_peer(peer_sysid);
}

// number of known peers in the frontend table
uint8_t AP_SwarmMesh_Backend::frontend_peer_count() const
{
    return _frontend.num_peers;
}

// peer entry at the given index, or nullptr if out of range
AP_SwarmMesh::PeerState *AP_SwarmMesh_Backend::frontend_peer_at(uint8_t index)
{
    if (index >= _frontend.num_peers) {
        return nullptr;
    }
    return &_frontend.peer_state[index];
}

// returns the SR stream rate (Hz) for a given bucket. 0 if the index is out of range or param is zero.
uint8_t AP_SwarmMesh_Backend::frontend_sr_rate(uint8_t bucket) const
{
    if (bucket >= AP_SwarmMesh::NUM_BUCKETS) {
        return 0;
    }
    return MAX(0, (int8_t)_frontend.stream_rate[bucket]);
}

// Returns true if the Full message profile should be used.
// Bit 0 of stream means a Full-capable radio is fitted.
// Even with a Full radio, STM32F4 FCs are downgraded to Lite at compile time.
bool AP_SwarmMesh_Backend::frontend_uses_full() const
{
    const uint8_t hw = (uint8_t)_frontend.hardware_mask;
    if (!(hw & 0x01)) {
        // Lite radio hardware — always Lite
        return false;
    }
#if defined(STM32F4)
    // Full radio fitted but F4 CPU cannot sustain the Full message set
    return false;
#else
    return true;
#endif
}

// max combined RX dataflash log write rate accessor
uint16_t AP_SwarmMesh_Backend::frontend_log_rate_hz() const
{
    return MAX(0, (int16_t)_frontend.log_rate_hz);
}

// RX log message mask accessor
uint32_t AP_SwarmMesh_Backend::frontend_log_mask() const
{
    return (uint32_t)(int32_t)_frontend.log_mask;
}

#endif  // AP_SWARMMESH_ENABLED
