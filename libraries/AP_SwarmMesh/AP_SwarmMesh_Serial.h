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

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_SERIAL_ENABLED

#include "AP_SwarmMesh_packet.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS_config.h>

// fixed header + max payload (header CRC is embedded in the header struct, not appended)
#define SWARMMESH_MAX_PAYLOAD 255
#define SWARMMESH_MSG_BUF_MAX      (sizeof(p2p_header_t) + SWARMMESH_MAX_PAYLOAD)

class AP_SwarmMesh_Serial : public AP_SwarmMesh_Backend
{

public:
    // constructor
    using AP_SwarmMesh_Backend::AP_SwarmMesh_Backend;

    // return true if radio is basically healthy (we are receiving data)
    bool healthy() override;

    // update the peer state
    void update() override;

private:

    // TX stream buckets (order must match SR_* param indices in AP_SwarmMesh::var_info)
    enum class Bucket : uint8_t {
        POSITION = 0,   // GLOBAL_POSITION_INT, LOCAL_POSITION_NED
        EXT_STAT = 1,   // SYS_STATUS, NAV_CONTROLLER_OUTPUT, POSITION_TARGET_GLOBAL_INT, MISSION_CURRENT
        EXTRA1   = 2,   // ATTITUDE, EKF_STATUS_REPORT
        // TODO: Add more buckets
    };

    // RX state machine
    enum class ParseState : uint8_t {
        WAIT_SYNC1,         // waiting for 0xAD
        WAIT_SYNC2,         // expecting 0xBC
        HEADER,             // read the remaining header
        PAYLOAD,            // read payload_len payload
    } _state = ParseState::WAIT_SYNC1;

    // members
    uint8_t  _msgbuf[SWARMMESH_MSG_BUF_MAX];    // buffer to hold most recent message from peer
    uint16_t _msg_len;                          // number of bytes received from the current message (may be larger than size of _msgbuf)
    uint8_t  _payload_len;                      // payload_len from header (matches packet field type)
    uint8_t  _crc;                              // calculated crc which is compared against actual received crc
    uint32_t _last_rx_ms;                       // last time we receive data from peer
    uint16_t _tx_seq;                           // sequence number of sent original packets
    uint16_t _tx_fwd;                           // sequence number of forwarded packets
    uint16_t _tx_dropped;                       // TX path dropped counter
    uint16_t _crc_fail;                         // CRC mismatch counter
    uint16_t _stale;                            // stale packet counter
    uint16_t _ttl;                              // ttl packet counter
    uint16_t _dedup;                            // duplicate packet counter
    uint16_t _dropped;                          // dropped packet counter
    uint8_t  _type;                             // packet type (0 == MAVLink)
    uint32_t _last_bucket_ms[AP_SwarmMesh::NUM_BUCKETS]; // last send time per bucket
    uint32_t _last_heartbeat_ms;                // last heartbeat send time (independent of bucket timing)

    // persistent MAVLink byte-level parser state
    mavlink_message_t _mavlink_rxmsg;
    mavlink_status_t  _mavlink_rx_status;

    // process one incoming byte; returns true when a complete, valid packet has been assembled in _msgbuf
    bool parse_byte(uint8_t b);

    // called in process_packet() when type = 0 (MAVLink)
    void handle_mavlink(const mavlink_message_t &msg, AP_SwarmMesh::PeerState &ps);

    // called when parse_byte() returns true (writes peer state into frontend)
    void process_packet();

    // write backend performance counters to the onboard log
    void log_stats() override;

    // TX path
    void send_stream(Bucket bucket);

    // Generated MAVLink. Serialize header + payload into a framed packet and write to UART
    void send_mavlink(uint8_t dest_id, const mavlink_message_t *msg, uint16_t deadline_ms, uint8_t ttl);

    // Forwarded MAVLink. Serialize header + payload into a framed packet and write to UART
    void forward_mavlink(uint8_t id, uint8_t dest_id, const uint8_t *payload, uint16_t deadline_ms, uint8_t ttl, uint8_t payload_len, uint8_t flags, uint64_t origin_time, uint16_t seq);

    // Buckets
    void send_heartbeat();
#if AP_AHRS_ENABLED
    void send_global_position_int();
    void send_local_position();
    void send_attitude();
    void send_ekf_status_report();
#endif
    void send_sys_status();
    void send_nav_controller_output();
    void send_position_target_global_int();
    void send_extended_sys_state();

};

#endif  // AP_SWARMMESH_SERIAL_ENABLED
