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
    uint16_t _tx_seq;                           // sequence number of sent packets
    uint16_t _crc_fail;                         // CRC mismatch counter
    uint8_t  _type;                             // packet type (0 == MAVLink)
    
    // process one incoming byte; returns true when a complete, valid packet has been assembled in _msgbuf
    bool parse_byte(uint8_t b);

    // called when parse_byte() returns true and type == 0 (writes peer state into frontend)
    void process_mavlink();

    // TX path
    // serialize header + payload into a framed packet and write to UART
    void send_mavlink(uint8_t dest_id, const uint8_t *payload, uint16_t deadline_ms, uint8_t payload_len);    

};

#endif  // AP_SWARMMESH_SERIAL_ENABLED
