#include <hal/HAL.h>

#include "gtest/gtest.h"
#include <iostream>
#include <filesystem>

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  std::cout << std::filesystem::current_path() << std::endl;
  int ret = RUN_ALL_TESTS();
  return ret;
}
