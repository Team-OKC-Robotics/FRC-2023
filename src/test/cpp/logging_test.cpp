

#include <frc/Filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

#include <filesystem>
namespace fs = std::filesystem;

#include "Logging.h"


class LoggingTest : public testing::Test {
public:
    virtual void SetUp() {
        plog::init<plog::OKCFormatter, Logging::Default>(plog::debug, "default_log.csv");
    }

protected:
};


TEST_F(LoggingTest, LogOutputTest) {
    for (int i = 0; i < 10; ++i)
    {
        PLOGD_(Logging::Default) << "default/index," << i;
        PLOGD_(Logging::Default) << "default/float_data," << 0.1 * static_cast<float>(i);
    }
}