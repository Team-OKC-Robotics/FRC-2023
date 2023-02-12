
#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>

#include "frc/geometry/Pose2d.h"

#include "SwerveModule.h"

class SwerveModuleTest : public testing::Test {
public:
    virtual void SetUp() {
        module_ = std::make_shared<SwerveModule>();
        module_->Init(FRONT_LEFT);
    }

protected:
    std::shared_ptr<SwerveModule> module_;
};

TEST_F(SwerveModuleTest, InitModule) {
    ASSERT_TRUE(swerve_->Init());

    //TODO check configured values
}

// need to test:
// - Update()
// - drive_pid
// - steer_pid
// - angle wrapping or something idk