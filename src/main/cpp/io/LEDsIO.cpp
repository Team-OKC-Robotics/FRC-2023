#include "io/LEDsIO.h"
#include "Parameters.h"

void LEDsIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    
    VOKC_CALL(ProcessIO());
}

void LEDsIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool LEDsIO::ProcessIO() {
    return true;
}