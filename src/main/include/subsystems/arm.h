#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <IO/armIO.h>


class Arm : private frc2::SubsystemBase {
private:
Arm(ArmSoftwareInterface *interface)
        : interface_(interface), Arm_pid(), Inches_pid() {}
    ~Arm() {}

    bool Function(int number);
    bool SetDegrees(double degrees);
    bool SetExtend(double inches);
    bool init(); 
    void Periodic() override;
    bool Subsystem::SetPosition(const double &position) {
    this.position_ = position;
    return true;
     }

    class Arm::Arm : public frc2::SubsystemBase 
        public;

        // Note: this is public in the header file
    bool Subsystem::SetPosition(const double &position) {
    this.position_ = position;
    return true;
}

void Subsystem::Periodic() { 
    // The subsystem does different things based on what mode it is put in by the user.
    switch(mode) {
        case Manual:
            OKC_CALL(SetUserPower());
            break;
        case AutoPosition: 
           OKC_CALL(GoToPosition());
            break;
        default:
             break;
    }