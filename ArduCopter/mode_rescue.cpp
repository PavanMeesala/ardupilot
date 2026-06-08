#include "Copter.h"

#if MODE_RESCUE_ENABLED

bool ModeRescue::init(bool ignore_checks)
{
    _wp_count       = 0;
    _expected_count = 0;
    _current_idx    = 0;

    return ModeGuided::init(ignore_checks);
}

// ---------------------------------------------------------------------------
// Receive RESCUE_WP from GCS
// ---------------------------------------------------------------------------
void ModeRescue::handle_rescue_wp(uint16_t seq, uint16_t total_count,
                                   int32_t lat_degE7, int32_t lon_degE7)
{
    if (seq >= RESCUE_WP_MAX) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "RESCUE_WP: seq %u exceeds max (%u), ignoring",
                        seq, RESCUE_WP_MAX);
        return;
    }

    _expected_count = total_count;

    Location wp{};
    wp.lat          = lat_degE7;
    wp.lng          = lon_degE7;
    wp.alt          = 0;
    wp.relative_alt = false;
    _waypoints[seq] = wp;

    if (static_cast<uint16_t>(seq + 1) > _wp_count) {
        _wp_count = seq + 1;
    }

    if (rescue_wps_complete()) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "RESCUE_WP: all %u waypoints received", _wp_count);

        // Forward all waypoints back to all GCS channels
        for (uint8_t i = 0; i < _wp_count; i++) {
            for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
                mavlink_msg_rescue_wp_send(
                    gcs().chan(c)->get_chan(),
                    _wp_count,
                    i,
                    _waypoints[i].lat,
                    _waypoints[i].lng
                );
            }
        }

        // if (_wp_count > 0) {
        //     _current_idx = 0;
        //     set_destination(_waypoints[_current_idx]);
        // }
    }
}

void ModeRescue::handle_user_wp_reached(uint16_t wp_index, uint8_t reached)
{
    if (reached) {
        gcs().send_text(MAV_SEVERITY_INFO, "Waypoint %u reached", wp_index);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Waypoint %u not reached", wp_index);
    }

    // Forward structured MAVLink message back to all GCS channels
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_user_wp_reached_send(
            gcs().chan(c)->get_chan(),
            wp_index,
            reached
        );
    }
}

#endif // MODE_RESCUE_ENABLED
