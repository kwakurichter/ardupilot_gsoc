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
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>

#include <AP_BattMonitor/AP_BattMonitor_config.h>
#if AP_BATTERY_ENABLED
#include <AP_BattMonitor/AP_BattMonitor.h>
#endif
#include <AP_Vehicle/AP_Vehicle.h>


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

    // TX send path
    const uint32_t now_ms = AP_HAL::millis();

    // heartbeat is always on, independent of SR_* stream config
    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000U; // 1Hz
    if (now_ms - _last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        _last_heartbeat_ms = now_ms;
        send_heartbeat();
    }

    // each bucket fires independently at its SR rate, capped by hardware limit (Lite or Full)
    const uint32_t hw_min_interval_ms = frontend_uses_full() ? (1000U / AP_SWARMMESH_FULL_HZ) : (1000U / AP_SWARMMESH_LITE_HZ);

    for (uint8_t i = 0; i < AP_SwarmMesh::NUM_BUCKETS; i++) {
        const uint8_t rate_hz = frontend_sr_rate(i);
        if (rate_hz == 0) {
            continue;
        }
        const uint32_t interval_ms = MAX(1000U / (uint32_t)rate_hz, hw_min_interval_ms);
        if (now_ms - _last_bucket_ms[i] < interval_ms) {
            continue;
        }
        _last_bucket_ms[i] = now_ms;
        send_stream(static_cast<Bucket>(i));
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

    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
        mavlink_position_target_global_int_t pt;
        mavlink_msg_position_target_global_int_decode(&msg, &pt);
        ps.target_pos.x = pt.lat_int;
        ps.target_pos.y = pt.lon_int;
        ps.target_pos.z = pt.alt;
#if HAL_LOGGING_ENABLED
        const struct log_SwarmMesh_PT pkt_pt{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_PT_MSG),
            time_us : AP_HAL::micros64(),
            sysid   : ps.sysid,
            lat     : pt.lat_int,
            lon     : pt.lon_int,
            alt     : pt.alt
        };
        AP::logger().WriteBlock(&pkt_pt, sizeof(pkt_pt));
#endif
        break;
    }

    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        mavlink_extended_sys_state_t es;
        mavlink_msg_extended_sys_state_decode(&msg, &es);
        ps.landed_state = es.landed_state;
#if HAL_LOGGING_ENABLED
        const struct log_SwarmMesh_ES pkt_es{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_ES_MSG),
            time_us        : AP_HAL::micros64(),
            sysid          : ps.sysid,
            landed_state   : es.landed_state
        };
        AP::logger().WriteBlock(&pkt_es, sizeof(pkt_es));
#endif        
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t at;
        mavlink_msg_attitude_decode(&msg, &at);
        ps.attitude.x = at.roll;
        ps.attitude.y = at.pitch;
        ps.attitude.z = at.yaw;
#if HAL_LOGGING_ENABLED
        const struct log_SwarmMesh_AT pkt_at{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_AT_MSG),
            time_us : AP_HAL::micros64(),
            sysid   : ps.sysid,
            pitch   : at.roll,
            roll    : at.pitch,
            yaw     : at.yaw
        };
        AP::logger().WriteBlock(&pkt_at, sizeof(pkt_at));
#endif
        break;
    }

    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t ek;
        mavlink_msg_ekf_status_report_decode(&msg, &ek);
        ps.pos_covariance[0] = ek.pos_horiz_variance;
        ps.pos_covariance[1] = ek.pos_vert_variance;
        ps.pos_covariance[2] = ek.velocity_variance;
#if HAL_LOGGING_ENABLED
        const struct log_SwarmMesh_EK pkt_ek{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_EK_MSG),
            time_us       : AP_HAL::micros64(),
            sysid         : ps.sysid,
            pos_horiz_var : ek.pos_horiz_variance,
            pos_vert_var  : ek.pos_vert_variance,
            vel_var       : ek.velocity_variance
        };
        AP::logger().WriteBlock(&pkt_ek, sizeof(pkt_ek));
#endif
        break;
    }

    // TODO: Add more cases (ATTITUDE, EXTENDED_SYS_STATE, ...)

    default:
        _dropped++;
        break;
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

void AP_SwarmMesh_Serial::send_mavlink(uint8_t dest_id, const mavlink_message_t *msg, uint16_t deadline_ms, uint8_t ttl)
{
    if (uart == nullptr) {
        return;
    }

    uint8_t payload[MAVLINK_MAX_PACKET_LEN];
    const uint16_t payload_len = mavlink_msg_to_send_buffer(payload, msg);

    // guard: full packet must fit in the TX ring buffer
    if (uart->txspace() < SWARMMESH_HEADER_SIZE + payload_len) {
        _tx_dropped++;
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
    uart->write((const uint8_t *)&hdr, SWARMMESH_HEADER_SIZE);
    uart->write(payload, payload_len);    
}

void AP_SwarmMesh_Serial::forward_mavlink(uint8_t id, uint8_t dest_id, const uint8_t *payload, uint16_t deadline_ms, uint8_t ttl, uint8_t payload_len, uint8_t flags, uint64_t origin_time, uint16_t seq)
{
    if (uart == nullptr) {
        return;
    }

    // guard: full packet must fit in the TX ring buffer
    if (uart->txspace() < SWARMMESH_HEADER_SIZE + payload_len) {
        _dropped++;
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
    uart->write((const uint8_t *)&hdr, SWARMMESH_HEADER_SIZE);
    uart->write(payload, payload_len);      

    _tx_fwd++;
}

void AP_SwarmMesh_Serial::send_stream(Bucket bucket)
{
    switch (bucket) {
    case Bucket::POSITION:
#if AP_AHRS_ENABLED
        send_global_position_int();
        send_local_position();
#endif
        break;
    case Bucket::EXT_STAT:
        send_sys_status();
        send_nav_controller_output();
        send_position_target_global_int();
        break;
    case Bucket::EXTRA1:
#if AP_AHRS_ENABLED
        send_attitude();
        send_ekf_status_report();
#endif
        send_extended_sys_state();
        break;
    }
    // TODO: Add mode buckets
}

void AP_SwarmMesh_Serial::send_heartbeat()
{
    // TODO: base_mode()/system_status() are per-vehicle GCS_MAVLINK overrides we have no access to (no vehicle reference)
    const bool armed = AP_HAL::get_HAL().util->get_soft_armed();
    uint8_t base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (armed) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    const MAV_STATE system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        gcs().frame_type(),
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        gcs().custom_mode(),
        system_status);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

#if AP_AHRS_ENABLED
void AP_SwarmMesh_Serial::send_global_position_int()
{
    AP_AHRS &ahrs = AP::ahrs();

    Location loc;
    UNUSED_RESULT(ahrs.get_location(loc));

    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }

    // inline relative-alt helper: distance above home in mm (up positive)
    float posD;
    ahrs.get_relative_position_D_home(posD);
    const int32_t relative_alt_mm = (int32_t)(-posD * 1000.0f);

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),
        loc.lat,                    // degE7
        loc.lng,                    // degE7
        loc.alt * 10,               // mm above MSL (Location stores cm)
        relative_alt_mm,            // mm above home
        (int16_t)(vel.x * 100),     // cm/s North
        (int16_t)(vel.y * 100),     // cm/s East
        (int16_t)(vel.z * 100),     // cm/s Down
        ahrs.yaw_sensor);           // cdeg

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

void AP_SwarmMesh_Serial::send_local_position()
{
    const AP_AHRS &ahrs = AP::ahrs();

    Vector3f pos, vel;
    if (!ahrs.get_relative_position_NED_origin_float(pos) ||
        !ahrs.get_velocity_NED(vel)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_local_position_ned_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),
        pos.x, pos.y, pos.z,
        vel.x, vel.y, vel.z);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}
#endif  // AP_AHRS_ENABLED

// TODO: Access AC_PosControl (with guard) to fill in vel/accel targets
void AP_SwarmMesh_Serial::send_position_target_global_int()
{
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle == nullptr) {
        return;
    }

    Location target;
    if (!vehicle->get_target_location(target)) {
        return;
    }
    if (!target.initialised()) {
        return;
    }
    float alt_amsl_m;
    if (!target.get_alt_m(Location::AltFrame::ABSOLUTE, alt_amsl_m)) {
        return;
    }

    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK =
        POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE |
        POSITION_TARGET_TYPEMASK_LAST_BYTE;

    mavlink_message_t msg;
    mavlink_msg_position_target_global_int_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),   // time_boot_ms
        MAV_FRAME_GLOBAL,   // targets are always global altitude
        TYPE_MASK,          // ignore everything except the x/y/z components
        target.lat,         // latitude as 1e7
        target.lng,         // longitude as 1e7
        alt_amsl_m,         // altitude AMSL in metres
        0.0f, 0.0f, 0.0f,   // vx, vy, vz
        0.0f, 0.0f, 0.0f,   // afx, afy, afz
        0.0f,               // yaw
        0.0f);              // yaw_rate

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

void AP_SwarmMesh_Serial::send_extended_sys_state()
{
    // TODO: landed_state()/vtol_state() are per-vehicle GCS_MAVLINK overrides we have no access to
    MAV_LANDED_STATE landed_state = MAV_LANDED_STATE_UNDEFINED;
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        if (!vehicle->get_likely_flying()) {
            landed_state = MAV_LANDED_STATE_ON_GROUND;
        } else if (vehicle->is_landing()) {
            landed_state = MAV_LANDED_STATE_LANDING;
        } else if (vehicle->is_taking_off()) {
            landed_state = MAV_LANDED_STATE_TAKEOFF;
        } else {
            landed_state = MAV_LANDED_STATE_IN_AIR;
        }
    }

    mavlink_message_t msg;
    mavlink_msg_extended_sys_state_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        MAV_VTOL_STATE_UNDEFINED,  // no generic VTOL-state source
        landed_state);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

#if AP_AHRS_ENABLED
void AP_SwarmMesh_Serial::send_attitude()
{
    const AP_AHRS &ahrs = AP::ahrs();
    const Vector3f omega = ahrs.get_gyro();
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),
        ahrs.get_roll_rad(),
        ahrs.get_pitch_rad(),
        ahrs.get_yaw_rad(),
        omega.x,
        omega.y,
        omega.z);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

void AP_SwarmMesh_Serial::send_ekf_status_report()
{
    nav_filter_status filter_status{};
    AP::ahrs().get_filter_status(filter_status);

    uint16_t flags = 0;
    if (filter_status.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filter_status.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filter_status.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filter_status.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filter_status.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filter_status.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filter_status.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filter_status.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filter_status.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filter_status.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filter_status.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }
    if (filter_status.flags.gps_glitching) {
        flags |= (1U << 15);
    }

    float velVar = 0, posVar = 0, hgtVar = 0, tasVar = 0;
    Vector3f magVar;
    AP::ahrs().get_variances(velVar, posVar, hgtVar, magVar, tasVar);

    mavlink_message_t msg;
    mavlink_msg_ekf_status_report_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        flags,
        velVar,
        posVar,
        hgtVar,
        fmaxf(fmaxf(magVar.x, magVar.y), magVar.z),
        0,          // terrain_alt_variance
        tasVar);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}
#endif  // AP_AHRS_ENABLED

// TODO: Fill in empty fields
void AP_SwarmMesh_Serial::send_sys_status()
{
    float voltage_mv = 0;
    float current_ca = -1; // 10mA units; -1 = unknown
    int8_t remaining_pct = -1;

#if AP_BATTERY_ENABLED
    const AP_BattMonitor &battery = AP::battery();
    if (battery.healthy()) {
        voltage_mv = battery.gcs_voltage() * 1000.0f;
        float amps;
        if (battery.current_amps(amps)) {
            current_ca = constrain_float(amps * 100.0f, -INT16_MAX, INT16_MAX);
        }
        uint8_t pct;
        if (battery.capacity_remaining_pct(pct)) {
            remaining_pct = (int8_t)pct;
        }
    }
#endif

    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        0,                  // onboard_control_sensors_present
        0,                  // onboard_control_sensors_enabled
        0,                  // onboard_control_sensors_health
        0,                  // load (permille); not tracked here
        (uint16_t)voltage_mv,
        (int16_t)current_ca,
        remaining_pct,
        0,                  // drop_rate_comm
        0,                  // errors_comm
        0, 0, 0, 0,         // errors_count1-4
        0, 0, 0);           // extended sensor fields

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

// TODO: Find way to access control targets to fill in empty
void AP_SwarmMesh_Serial::send_nav_controller_output()
{
    // TODO: SwarmMesh has no access to mode-specific control targets (no vehicle reference)
    float nav_roll_deg = 0;
    float nav_pitch_deg = 0;
// #if AP_AHRS_ENABLED
//     const AP_AHRS &ahrs = AP::ahrs();
//     nav_roll_deg = degrees(ahrs.get_roll_rad());
//     nav_pitch_deg = degrees(ahrs.get_pitch_rad());
// #endif

    float wp_bearing_deg = 0;
    float wp_distance_m = 0;
    float xtrack_error_m = 0;
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        vehicle->get_wp_bearing_deg(wp_bearing_deg);
        vehicle->get_wp_distance_m(wp_distance_m);
        vehicle->get_wp_crosstrack_error_m(xtrack_error_m);
    }

    mavlink_message_t msg;
    mavlink_msg_nav_controller_output_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        nav_roll_deg,
        nav_pitch_deg,
        (int16_t)wp_bearing_deg,
        (int16_t)wp_bearing_deg,           // target_bearing: no separate target available
        (uint16_t)MIN(wp_distance_m, (float)UINT16_MAX),
        0,                                  // alt_error
        0,                                  // aspd_error
        xtrack_error_m);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
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
       txfwd           : _tx_fwd,
       txdrop          : _tx_dropped
    };
    AP::logger().WriteBlock(&pkt_swarmmesh, sizeof(pkt_swarmmesh));
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_SWARMMESH_SERIAL_ENABLED