#include "math.h"
#include "SlewRateLimiter.h"
#include "Utils.h"

SlewRateLimiter::SlewRateLimiter(double limit) {
    limit_ = limit;
    last_output_ = 0.0;
}

double SlewRateLimiter::Calculate(double input) {
    // so change = last - current
    // we want to limit that change

    double output = last_output_;

    // if the change is higher than the limit
    if (abs(last_output_ - input) > limit_) {
        // then return what we had last plus the max allowed change
        output = last_output_ + copysign(limit_, input);
    } else {
        // otherwise we can just return the input
        output = input;
    }


    // and our last output is then set to our actual output
    last_output_ = output;
    return output;
}