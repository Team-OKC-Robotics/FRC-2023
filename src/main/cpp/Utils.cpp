#include "Utils.h"


namespace TeamOKC {
    // TODO: make this a templated function.
    bool Clamp(const double &lower, const double &upper, double *value) {
        OKC_CHECK(value != nullptr);

        // Ensure value is not lower than the minimum or higher than the
        // maximum. If it is out of the bounds, set it to the boundary value it
        // violates.
        if (*value < lower) {
            *value = lower;
        } else if (*value > upper) {
            *value = upper;
        } else {
            // Nothing to do here, value is within the range.
        }

        return true;
    }

    bool WrapAngle(double *angle) {
        if (*angle < -180) {
            *angle = *angle + 360;
        } else if (*angle > 180) {
            *angle = *angle - 360;
        }
        return true;
    }

    double Radians(double degrees) {
        return degrees * M_PI / 180.0;
    }
} // namespace TeamOKC

