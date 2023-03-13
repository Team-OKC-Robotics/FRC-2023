#include "Auton.h"

std::string Auton::GetAutonName() {
    return name_;
}

bool Auton::SetName(std::string name) {
    name_ = name;

    return true;
}