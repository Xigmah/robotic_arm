#include "matrix4x4.h"

#include <gtest/gtest.h>

#include <cstring>

using namespace arm::math;

static constexpr size_t SIZE = 4;

TEST(Matrix4x4Tests, testConstructor) {
  Matrix4x4 mat;

  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      if (i == j) {
        EXPECT_EQ(mat.get(i, j), 1.0)
            << "Diagonal element [" << i << "][" << j << "] should be 1.0";
      } else {
        EXPECT_EQ(mat.get(i, j), 0.0)
            << "Off-diagonal element [" << i << "][" << j << "] should be 0.0";
      }
    }
  }
}

TEST(Matrix4x4Tests, GetThrowsOnOutOfBounds) {
  Matrix4x4 mat;

  // Valid access (should not throw)
  EXPECT_NO_THROW(mat.get(0, 0));
  EXPECT_NO_THROW(mat.get(3, 3));

  // Invalid access (should throw)
  EXPECT_THROW(mat.get(4, 0), std::out_of_range);
  EXPECT_THROW(mat.get(0, 4), std::out_of_range);
  EXPECT_THROW(mat.get(4, 4), std::out_of_range);
}

TEST(Matrix4x4Tests, SetThrowsOnOutOfBounds) {
  Matrix4x4 mat;

  // Valid access (should not throw)
  EXPECT_NO_THROW(mat.set(0, 0, 5.0));
  EXPECT_NO_THROW(mat.set(3, 3, 5.0));

  // Invalid access (should throw)
  EXPECT_THROW(mat.set(4, 0, 5.0), std::out_of_range);
  EXPECT_THROW(mat.set(0, 4, 5.0), std::out_of_range);
  EXPECT_THROW(mat.set(4, 4, 5.0), std::out_of_range);
}

TEST(Matrix4x4Tests, testTranslation) {
  Matrix4x4 mat = Matrix4x4::translation(10.0, 20.0, 30.0);

  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      if (i == j) {
        EXPECT_EQ(mat.get(i, j), 1.0)
            << "Diagonal element [" << i << "][" << j << "] should be 1.0";
      } else if (i == 0 && j == 3) {
        EXPECT_EQ(mat.get(i, j), 10.0);
      } else if (i == 1 && j == 3) {
        EXPECT_EQ(mat.get(i, j), 20.0);
      } else if (i == 2 && j == 3) {
        EXPECT_EQ(mat.get(i, j), 30.0);
      } else {
        EXPECT_EQ(mat.get(i, j), 0.0)
            << "Off-diagonal element [" << i << "][" << j << "] should be 0.0";
      }
    }
  }
}

TEST(Matrix4x4Tests, testRotationX) {
  Matrix4x4 mat = Matrix4x4::rotationX(10.0);

  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      if (i == 1 && j == 1) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);;
      } else if (i == 2 && j == 2) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);;
      } else if (i == 2 && j == 1) {
        EXPECT_NEAR(mat.get(i, j), std::sin(10.0), 1e-9);;
      } else if (i == 1 && j == 2) {
        EXPECT_NEAR(mat.get(i, j), -1 * std::sin(10.0), 1e-9);;
      } else if (i == j) {
        EXPECT_EQ(mat.get(i, j), 1.0)
            << "Diagonal element [" << i << "][" << j << "] should be 1.0";
      } else {
        EXPECT_EQ(mat.get(i, j), 0.0)
            << "Off-diagonal element [" << i << "][" << j << "] should be 0.0";
      }
    }
  }
}

TEST(Matrix4x4Tests, testRotationY) {
  Matrix4x4 mat = Matrix4x4::rotationY(10.0);

  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      if (i == 0 && j == 0) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);;
      } else if (i == 2 && j == 2) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);;
      } else if (i == 0 && j == 2) {
        EXPECT_NEAR(mat.get(i, j), std::sin(10.0), 1e-9);;
      } else if (i == 2 && j == 0) {
        EXPECT_NEAR(mat.get(i, j), -1 * std::sin(10.0), 1e-9);;
      } else if (i == j) {
        EXPECT_EQ(mat.get(i, j), 1.0)
            << "Diagonal element [" << i << "][" << j << "] should be 1.0";
      } else {
        EXPECT_EQ(mat.get(i, j), 0.0)
            << "Off-diagonal element [" << i << "][" << j << "] should be 0.0";
      }
    }
  }
}

TEST(Matrix4x4Tests, testRotationZ) {
  Matrix4x4 mat = Matrix4x4::rotationZ(10.0);

  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      if (i == 0 && j == 0) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);;
      } else if (i == 1 && j == 1) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);;
      } else if (i == 1 && j == 0) {
        EXPECT_NEAR(mat.get(i, j), std::sin(10.0), 1e-9);;
      } else if (i == 0 && j == 1) {
        EXPECT_NEAR(mat.get(i, j), -1 * std::sin(10.0), 1e-9);;
      } else if (i == j) {
        EXPECT_EQ(mat.get(i, j), 1.0)
            << "Diagonal element [" << i << "][" << j << "] should be 1.0";
      } else {
        EXPECT_EQ(mat.get(i, j), 0.0)
            << "Off-diagonal element [" << i << "][" << j << "] should be 0.0";
      }
    }
  }
}