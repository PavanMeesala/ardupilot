#include "Copter.h"

#if MODE_RESCUE_ENABLED

bool ModeRescue::init(bool ignore_checks)
{
    // call parent guided mode init
    return ModeGuided::init(ignore_checks);
}

#endif
