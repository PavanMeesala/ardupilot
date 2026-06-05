#include "Copter.h"

#if MODE_RESCUE_ENABLED

bool ModeRescue::init(bool ignore_checks)
{
    // Reset waypoint state on every mode entry
    _wp_count      = 0;
    _expected_count = 0;
    _current_idx   = 0;

    // Call parent guided mode init
    return ModeGuided::init(ignore_checks);
}

// ---------------------------------------------------------------------------
// Called by GCS_MAVLink when a RESCUE_WP packet arrives.
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
    wp.lat              = lat_degE7;
    wp.lng              = lon_degE7;
    wp.alt              = 0;
    wp.relative_alt     = false;

    _waypoints[seq] = wp;

    if (static_cast<uint16_t>(seq + 1) > _wp_count) {
        _wp_count = seq + 1;
    }

    if (rescue_wps_complete()) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "RESCUE_WP: all %u waypoints received", _wp_count);

        // Immediately send the first waypoint to guided mode navigation
        if (_wp_count > 0) {
            _current_idx = 0;
            set_destination(_waypoints[_current_idx]);
        }
    }
}
void ModeRescue::handle_user_wp_reached(uint16_t wp_index, uint8_t reached)
{
    if (reached) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Waypoint %u reached", wp_index);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Waypoint %u not reached", wp_index);
    }
}

#endif  // MODE_RESCUE_ENABLED
