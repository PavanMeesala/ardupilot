#include "Copter.h"

#if MODE_DYNAMIC_LANDING_ENABLED

bool ModeDynamicLanding::init(bool ignore_checks)
{

    return ModeGuided::init(ignore_checks);
}

#endif // MODE_DYNAMIC_LANDING_ENABLED
