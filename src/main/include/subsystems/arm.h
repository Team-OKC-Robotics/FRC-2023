

class Arm : public frc2::SubsystemBase {
public:
Arm(ArmSoftwareInterface *interface)
        : interface_(interface), Arm_pid(1.0, 0.0, 0.0) {}
    ~Arm() {}

    bool Function(int number);
    bool SetDegrees(double degrees);
    bool SetExtend(double inches);
    bool init(); 
private:
    ArmSoftwareInterface *const interface;
};