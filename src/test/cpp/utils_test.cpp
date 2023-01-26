
#include <gtest/gtest.h>
#include <iostream>

#include "Utils.h"

class UtilsTest : public testing::Test {
public:
  virtual void SetUp() {
    // Nothing to set up
  }

protected:
  bool OkcChecker(bool check) {
    OKC_CHECK(check);

    return true;
  }

  bool OkcCallChecker(bool return_val) {
    OKC_CALL(OkcChecker(return_val));
    return true;
  }
};

TEST_F(UtilsTest, OkcCheckTest) {
  std::shared_ptr<bool> b = nullptr;
  ASSERT_FALSE(OkcChecker(b != nullptr));
  ASSERT_TRUE(OkcChecker(b == nullptr));
}

TEST_F(UtilsTest, OkcCallTest) {
  ASSERT_FALSE(OkcCallChecker(false));
  ASSERT_TRUE(OkcCallChecker(true));
}