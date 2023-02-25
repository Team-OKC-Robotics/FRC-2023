

#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>
namespace fs = std::filesystem;

#include "Logging.h"

class LoggingTest : public testing::Test {
public:
    virtual void SetUp() {   
        log_.Init("default_log");
    }

protected:
    OKCLog log_;
};


TEST_F(LoggingTest, LogOutputTest) {
    for (int i = 0; i < 10; ++i)
    {
        log_.Log("/count", i);
    }
}