#include <gtest/gtest.h>

#include "ReadFile.h"

TEST(UtilsTest, TestReadFile) {
  std::string file = util::ReadFile("../resources/DemoFile.txt");
  EXPECT_FALSE(file.empty());

  std::string expected =
      "This is a demo file\n"
      "This is second line\n"
      "Some numbers like 1, 2, 3\n";

  EXPECT_EQ(file, expected);
}