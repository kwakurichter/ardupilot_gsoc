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
#include <AP_Logger/AP_Logger.h>


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
            process_packet();
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

// decode a fully-parsed MAVLink message, update peer state, and emit log entries
void AP_SwarmMesh_Serial::handle_mavlink(const mavlink_message_t &msg, AP_SwarmMesh::PeerState &ps)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        ps.vehicle_type = hb.type;
        ps.armed_state = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        ps.mode = (uint8_t)hb.custom_mode;
#if HAL_LOGGING_ENABLED
        const struct log_SwarmMesh_HB pkt_swarmmesh_hb{
        LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_HB_MSG),
        time_us         : AP_HAL::micros64(),
        sysid           : ps.sysid,
        vehicle_type    : ps.vehicle_type,
        mode            : ps.mode,
        armed_state     : (uint8_t)ps.armed_state
        };
        AP::logger().WriteBlock(&pkt_swarmmesh_hb, sizeof(pkt_swarmmesh_hb));
#endif
        break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t ss;
        mavlink_msg_sys_status_decode(&msg, &ss);
        ps.battery_voltage = ss.voltage_battery;
#if HAL_LOGGING_ENABLED
        const struct log_SwarmMesh_SS pkt_ss{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_SS_MSG),
            time_us     : AP_HAL::micros64(),
            sysid       : ps.sysid,
            bat_voltage : ps.battery_voltage
        };
        AP::logger().WriteBlock(&pkt_ss, sizeof(pkt_ss));
#endif
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t gp;
        mavlink_msg_global_position_int_decode(&msg, &gp);
        ps.global_pos.x = (float)gp.lat;   // degE7
        ps.global_pos.y = (float)gp.lon;   // degE7
        ps.global_pos.z = (float)gp.alt;   // mm above MSL
#if HAL_LOGGING_ENABLED
        const struct log_SwarmMesh_GP pkt_gp{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_GP_MSG),
            time_us : AP_HAL::micros64(),
            sysid   : ps.sysid,
            lat     : gp.lat,
            lon     : gp.lon,
            alt     : gp.alt
        };
        AP::logger().WriteBlock(&pkt_gp, sizeof(pkt_gp));
#endif
        break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t lp;
        mavlink_msg_local_position_ned_decode(&msg, &lp);
        ps.local_pos_NED.x = lp.x;
        ps.local_pos_NED.y = lp.y;
        ps.local_pos_NED.z = lp.z;
#if HAL_LOGGING_ENABLED
        const struct log_SwarmMesh_LP pkt_lp{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_LP_MSG),
            time_us : AP_HAL::micros64(),
            sysid   : ps.sysid,
            x       : lp.x,
            y       : lp.y,
            z       : lp.z
        };
        AP::logger().WriteBlock(&pkt_lp, sizeof(pkt_lp));
#endif
        break;
    }

    // TODO: Add more cases (ATTITUDE, EXTENDED_SYS_STATE, ...)
    }
}

// called when a complete packet arrives — routes, forwards, or
// delivers to the local parser depending on type, dest_id and TTL.
void AP_SwarmMesh_Serial::process_packet()
{
    _last_rx_ms = AP_HAL::millis();

    const p2p_header_t *hdr = (const p2p_header_t *)_msgbuf;

    if (hdr->version != SWARMMESH_VERSION_01) {
        _dropped++;
        return;
    }

    // Duplicate check (find or allocate a peer table entry for this origin)
    AP_SwarmMesh::PeerState *ps = frontend_peerstate(hdr->origin_id);
    if (ps == nullptr) {
        // peer table full — drop
        _dropped++;
        return;
    }

    if (ps->seq_seen_mask == 0) {
        // first packet ever from this peer: always accept, initialise window
        ps->seq_seen_mask = 1;
        ps->last_seq = hdr->seq;
    } else {
        //   positive delta -> packet is newer than last_seq
        //   negative delta -> packet is older (may be a late/re-flooded copy)
        const int16_t delta = (int16_t)(hdr->seq - ps->last_seq);
        if (delta == 0) {
            // exact duplicate
            _dedup++;
            return;
        } else if (delta > 0 && delta < 32) {
            // new packet within window — advance window, mark current seq
            ps->seq_seen_mask = (ps->seq_seen_mask << (uint8_t)delta) | 1U;
            ps->last_seq = hdr->seq;
        } else if (delta >= 32) {
            // large jump forward (e.g. after a gap) — reset window
            ps->seq_seen_mask = 1U;
            ps->last_seq = hdr->seq;
        } else if (delta > -32) {
            // old packet, within the trailing window — check if already seen
            const uint32_t bit = 1U << (uint8_t)(-delta);
            if (ps->seq_seen_mask & bit) {
                _dedup++;
                return;
            }
            ps->seq_seen_mask |= bit;
        } else {
            // too old, outside window (drop)
            _dropped++;
            return;
        }
    }

    // freshness check: only if sender had GPS-synchronised time
    if (!(hdr->flags & SWARMMESH_NO_RTC)) {
        uint64_t utc_usec = 0;
#if AP_RTC_ENABLED
        AP::rtc().get_utc_usec(utc_usec);
        const uint64_t deadline_us = (uint64_t)hdr->deadline_ms * 1000ULL;
        // guard subtraction against underflow before comparing
        if (utc_usec > hdr->origin_time_us && (utc_usec - hdr->origin_time_us) > deadline_us) {
            // stale, past freshness budget
            _stale++;
            return;
        }
#endif
    }

    if (hdr->ttl == 0) {
        // TTL expired (drop)
        _ttl++;
        return;
    }

    if (hdr->dest_id != frontend_sysid()) {
        // not addressed to us — forward with TTL decremented
        forward_mavlink(hdr->origin_id, hdr->dest_id,
                        &_msgbuf[SWARMMESH_HEADER_SIZE],
                        hdr->deadline_ms, hdr->ttl,
                        hdr->payload_len, hdr->flags,
                        hdr->origin_time_us, hdr->seq);
        return;
    }

    if (hdr->type == SWARMMESH_TYPE_MAVLINK) {
        // feed the raw MAVLink frame bytes through the parser
        mavlink_message_t msg;
        const uint8_t *payload = &_msgbuf[SWARMMESH_HEADER_SIZE];
        for (uint8_t i = 0; i < hdr->payload_len; i++) {
            if (mavlink_frame_char_buffer(&_mavlink_rxmsg, &_mavlink_rx_status, payload[i], &msg, &_mavlink_rx_status)) {
                handle_mavlink(msg, *ps);
            }
        }
    } else {
        _dropped++;
    }
}

void AP_SwarmMesh_Serial::send_mavlink(uint8_t dest_id, const uint8_t *payload, uint16_t deadline_ms, uint8_t ttl, uint8_t payload_len)
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
    hdr.ttl            = ttl;
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
    hdr.origin_time_us = utc_usec;

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

void AP_SwarmMesh_Serial::forward_mavlink(uint8_t id, uint8_t dest_id, const uint8_t *payload, uint16_t deadline_ms, uint8_t ttl, uint8_t payload_len, uint8_t flags, uint64_t origin_time, uint16_t seq)
{
    if (uart == nullptr) {
        return;
    }

    p2p_header_t hdr {};
    hdr.stx1           = SWARMMESH_SYNC1;
    hdr.stx2           = SWARMMESH_SYNC2;
    hdr.version        = SWARMMESH_VERSION_01;
    hdr.type           = SWARMMESH_TYPE_MAVLINK;
    hdr.flags          = flags;
    hdr.origin_id      = id;
    hdr.dest_id        = dest_id;
    hdr.prev_id        = frontend_sysid();
    hdr.ttl            = (ttl - 1);
    hdr.seq            = seq;
    hdr.deadline_ms    = deadline_ms;
    hdr.payload_len    = payload_len;
    hdr.origin_time_us = origin_time;

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

    _tx_fwd++;
}

#if HAL_LOGGING_ENABLED
// Write connection stats
void AP_SwarmMesh_Serial::log_stats()
{
    const struct log_SwarmMesh pkt_swarmmesh{
       LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_MSG),
       time_us         : AP_HAL::micros64(),
       crc_fail        : _crc_fail,
       stale           : _stale,
       ttl             : _ttl,
       dedup           : _dedup,
       drop            : _dropped,
       txseq           : _tx_seq,
       txfwd           : _tx_fwd
    };
    AP::logger().WriteBlock(&pkt_swarmmesh, sizeof(pkt_swarmmesh));
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_SWARMMESH_SERIAL_ENABLED