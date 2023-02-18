#include "subsystems/Vision.h"
#include "io/VisionIO.h"

class VisionTest : public testing::Test {
public:
    virtual void SetUp() {
        vision_ = std::make_shared<vision>(&sw_interface_);
        vision_->Init();
    }

protected:
    std::shared_ptr<VisionTest> vision_;
    
    VisionSubsystemInterface sw_interface_;
};

TEST_F(VisionTest, InitTest) {
    ASSERT_TRUE(vision_->Init());
}

TEST_F(VisionTest, ConeErrorTest) {
    ASSERT_TRUE(vision_->ConeError());
}

TEST_F(VisionTest, ConeDistanceTest) {
    ASSERT_TRUE(vision_->ConeDistance());
}

TEST_F(VisionTest, CubeDistanceTest) {
    ASSERT_TRUE(vision_->CubeDistance());
}

TEST_F(VisionTest, CubeAngleTest) {
    ASSERT_TRUE(vision_->CubeAngle());
}

TEST_F(VisionTest, ResetSubsystemTest) {
    ASSERT_TRUE(vision_->ResetSubsystem())
}