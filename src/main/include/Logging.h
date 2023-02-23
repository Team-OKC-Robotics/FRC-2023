
#pragma once

#include <plog/Log.h>
#include <plog/Initializers/RollingFileInitializer.h>

namespace Logging {
    // Define log IDs for subsystems.
    enum Loggers {
        Default = 0,
        SwerveDrive = 1,
        Arm = 2
    };
} // namespace Logging
