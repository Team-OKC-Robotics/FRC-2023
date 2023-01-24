#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <IO/armIO.h>


class Arm : public frc2::SubsystemBase {
public:
Arm(ArmSoftwareInterface *interface)
        : interface_(interface){}
    ~Arm() {}

    bool Function(int number);
    bool SetDegrees(double degrees);
    bool SetExtend(double inches);
    bool init(); 
    void Periodic() override;
private:
    ArmSoftwareInterface *const interface_;
    std::shared_ptr<frc::PIDController> Arm_pid;
    std::shared_ptr<frc::PIDController> Inches_pid;
};