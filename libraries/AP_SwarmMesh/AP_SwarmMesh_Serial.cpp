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

#include "AP_SwarmMesh_Serial.h"

#if AP_SWARMMESH_SERIAL_ENABLED

#include "AP_SwarmMesh_packet.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>


#define SWARMMESH_SYNC1                     0xAD    // SYNC1
#define SWARMMESH_SYNC2                     0xBC    // SYNC2
#define SWARMMESH_VERSION_01                0x01    // Version 1
#define SWARMMESH_TYPE_MAVLINK              0x00    // MAVLink
#define SWARMMESH_NORMAL                    0x00    // Flag
#define SWARMMESH_NO_RTC                    0x01    // No GPS-based time sync
#define SWARMMESH_HEADER_SIZE (sizeof(p2p_header_t))


extern const AP_HAL::HAL& hal;

// return true if sensor is basically healthy (we are receiving data)
bool AP_SwarmMesh_Serial::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - _last_rx_ms) < AP_SWARMMESH_TIMEOUT_MS);
}

// update the state of the sensor
void AP_SwarmMesh_Serial::update(void)
{
    // return immediately if not serial port
    if (uart == nullptr) {
        return;
    }

    // check uart for any incoming messages
    uint32_t nbytes = MIN(uart->available(), 1024U);
    while (nbytes-- > 0) {
        const int16_t b = uart->read();
        if (b >= 0 && parse_byte((uint8_t)b)) {
            if (_type == SWARMMESH_TYPE_MAVLINK) {
                process_mavlink();
            }
        }
    }
}

// process one byte received on serial port. Message is stored in _msgbuf.
// Returns true when a complete, CRC-valid packet is ready in _msgbuf.
bool AP_SwarmMesh_Serial::parse_byte(uint8_t b)
{
    switch (_state) {

    case ParseState::WAIT_SYNC1:
        if (b == SWARMMESH_SYNC1) {
            _msgbuf[0] = b;
            _msg_len = 1;
            _crc = b;
            _state = ParseState::WAIT_SYNC2;
        }
        break;

    case ParseState::WAIT_SYNC2:
        if (b == SWARMMESH_SYNC2) {
            _msgbuf[1] = b;
            _msg_len = 2;
            _crc += b;
            _state = ParseState::HEADER;
        } else {
            _state = ParseState::WAIT_SYNC1;    // wrong byte — resync
        }
        break;

    case ParseState::HEADER:
        // accumulate CRC over all header bytes except the final CRC field itself
        if (_msg_len < SWARMMESH_HEADER_SIZE - 1) {
            _crc += b;
        }
        _msgbuf[_msg_len++] = b;
        if (_msg_len == SWARMMESH_HEADER_SIZE) {
            const p2p_header_t *hdr = (const p2p_header_t *)_msgbuf;
            _type        = hdr->type;
            _payload_len = hdr->payload_len;
            if (_crc != hdr->crc) {
                // header CRC mismatch — discard and resync
                _crc_fail++;
                _state = ParseState::WAIT_SYNC1;
            } else if (_payload_len > SWARMMESH_MAX_PAYLOAD) {
                // payload length out of range — discard
                _state = ParseState::WAIT_SYNC1;
            } else if (_payload_len == 0) {
                // no payload — packet is complete
                _state = ParseState::WAIT_SYNC1;
                return true;
            } else {
                _state = ParseState::PAYLOAD;
            }
        }
        break;

    case ParseState::PAYLOAD: {
        // bounds-check before writing (payload_len already validated in HEADER state)
        if (_msg_len < SWARMMESH_MSG_BUF_MAX) {
            _msgbuf[_msg_len] = b;
        }
        _msg_len++;
        if (_msg_len == SWARMMESH_HEADER_SIZE + _payload_len) {
            _state = ParseState::WAIT_SYNC1;
            return true;
        }
        break;
    }
    }
    return false;
}

// called when a complete MAVLink-type packet arrives — TODO: parse MAVLink frame
// and write decoded fields into _frontend.peer_state[]
void AP_SwarmMesh_Serial::process_mavlink()
{
    _last_rx_ms = AP_HAL::millis();
    // TODO: hand _msgbuf payload to a MAVLink parser and update peer state
}

void AP_SwarmMesh_Serial::send_mavlink(uint8_t dest_id, const uint8_t *payload, uint16_t deadline_ms, uint8_t payload_len)
{
    if (uart == nullptr) {
        return;
    }

    p2p_header_t hdr {};
    hdr.stx1           = SWARMMESH_SYNC1;
    hdr.stx2           = SWARMMESH_SYNC2;
    hdr.version        = SWARMMESH_VERSION_01;
    hdr.type           = SWARMMESH_TYPE_MAVLINK;
    hdr.origin_id      = frontend_sysid();   // use accessor — friendship not inherited
    hdr.dest_id        = dest_id;
    hdr.prev_id        = frontend_sysid();
    hdr.seq            = _tx_seq++;
    hdr.deadline_ms    = deadline_ms;
    hdr.payload_len    = payload_len;

    // get unix time in ms (requires GPS lock; 0 if unavailable)
    uint64_t utc_usec = 0;
#if AP_RTC_ENABLED
    AP::rtc().get_utc_usec(utc_usec);
    hdr.flags          = SWARMMESH_NORMAL;
#else    
    hdr.flags          = SWARMMESH_NO_RTC;
#endif
    hdr.origin_time_ms = utc_usec / 1000ULL;

    // compute header CRC over all bytes except the crc field itself, then set it
    uint8_t crc = 0;
    const uint8_t *hdr_bytes = (const uint8_t *)&hdr;
    for (uint8_t i = 0; i < SWARMMESH_HEADER_SIZE - 1; i++) {
        crc += hdr_bytes[i];
    }
    hdr.crc = crc;

    // write complete header then payload
    for (uint8_t i = 0; i < SWARMMESH_HEADER_SIZE; i++) {
        uart->write(hdr_bytes[i]);
    }
    for (uint8_t i = 0; i < payload_len; i++) {
        uart->write(payload[i]);
    }
}

#endif  // AP_SWARMMESH_SERIAL_ENABLED