#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/Vision.h"


/**
 *
 */
class VisionCubeAlignCommand
    : public frc2::CommandHelper<frc2::CommandBase, VisionCubeAlignCommand> {
public:
    explicit VisionCubeAlignCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<Vision> vision);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

    bool AtSetpoint();

private:
    std::shared_ptr<SwerveDrive> swerve_;
    std::shared_ptr<Vision> vision_;
};
