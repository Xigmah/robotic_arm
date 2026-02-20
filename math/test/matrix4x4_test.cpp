#include "matrix4x4.h"

#include <gtest/gtest.h>

using namespace arm::math;

static constexpr size_t SIZE = 4;

TEST(Matrix4x4Tests, Constructor) {
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

TEST(Matrix4x4Tests, EqualsIdenticalMatrices) {
  Matrix4x4 m1 = Matrix4x4::identity();
  Matrix4x4 m2 = Matrix4x4::identity();

  EXPECT_TRUE(m1.equals(m2));
}

TEST(Matrix4x4Tests, EqualsSameObject) {
  Matrix4x4 m1 = Matrix4x4::translation(5, 10, 15);

  // Comparing to itself should always be true
  EXPECT_TRUE(m1.equals(m1));
}

TEST(Matrix4x4Tests, EqualsSameMatrices) {
  Matrix4x4 m1 = Matrix4x4::translation(5, 10, 15);
  Matrix4x4 m2 = Matrix4x4::translation(5, 10, 15);

  EXPECT_TRUE(m1.equals(m2));
}

TEST(Matrix4x4Tests, EqualsDifferentMatrices) {
  Matrix4x4 m1 = Matrix4x4::translation(5, 10, 15);
  Matrix4x4 m2 = Matrix4x4::translation(6, 10, 15);

  EXPECT_FALSE(m1.equals(m2));
}

TEST(Matrix4x4Tests, Translation) {
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

TEST(Matrix4x4Tests, TranslationVector) {
  Matrix4x4 mat = Matrix4x4::translation(10.0, 20.0, 30.0);
  Vector3D v = mat.getTranslation();

  EXPECT_EQ(v.get_x(), 10.0);
  EXPECT_EQ(v.get_y(), 20.0);
  EXPECT_EQ(v.get_z(), 30.0);
}

TEST(Matrix4x4Tests, RotationX) {
  Matrix4x4 mat = Matrix4x4::rotationX(10.0);

  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      if (i == 1 && j == 1) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);
      } else if (i == 2 && j == 2) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);
      } else if (i == 2 && j == 1) {
        EXPECT_NEAR(mat.get(i, j), std::sin(10.0), 1e-9);
      } else if (i == 1 && j == 2) {
        EXPECT_NEAR(mat.get(i, j), -1 * std::sin(10.0), 1e-9);
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

TEST(Matrix4x4Tests, RotationY) {
  Matrix4x4 mat = Matrix4x4::rotationY(10.0);

  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      if (i == 0 && j == 0) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);
      } else if (i == 2 && j == 2) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);
      } else if (i == 0 && j == 2) {
        EXPECT_NEAR(mat.get(i, j), std::sin(10.0), 1e-9);
      } else if (i == 2 && j == 0) {
        EXPECT_NEAR(mat.get(i, j), -1 * std::sin(10.0), 1e-9);
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

TEST(Matrix4x4Tests, RotationZ) {
  Matrix4x4 mat = Matrix4x4::rotationZ(10.0);

  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      if (i == 0 && j == 0) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);
      } else if (i == 1 && j == 1) {
        EXPECT_NEAR(mat.get(i, j), std::cos(10.0), 1e-9);
      } else if (i == 1 && j == 0) {
        EXPECT_NEAR(mat.get(i, j), std::sin(10.0), 1e-9);
      } else if (i == 0 && j == 1) {
        EXPECT_NEAR(mat.get(i, j), -1 * std::sin(10.0), 1e-9);
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

TEST(Matrix4x4Tests, MultiplyByIdentity) {
  Matrix4x4 trans = Matrix4x4::translation(5, 10, 15);
  Matrix4x4 identity = Matrix4x4::identity();

  Matrix4x4 result1 = trans * identity;
  Matrix4x4 result2 = identity * trans;

  // Multiplying by identity should not change the matrix
  EXPECT_EQ(result1.get(0, 3), 5.0);
  EXPECT_EQ(result1.get(1, 3), 10.0);
  EXPECT_EQ(result1.get(2, 3), 15.0);

  // Identity multiplication is commutative
  EXPECT_EQ(result2.get(0, 3), 5.0);
  EXPECT_EQ(result2.get(1, 3), 10.0);
  EXPECT_EQ(result2.get(2, 3), 15.0);
}

TEST(Matrix4x4Tests, IdentityMultiplyIdentity) {
  Matrix4x4 id1 = Matrix4x4::identity();
  Matrix4x4 id2 = Matrix4x4::identity();

  Matrix4x4 result = id1 * id2;

  // Should still be identity
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      if (i == j) {
        EXPECT_EQ(result.get(i, j), 1.0);
      } else {
        EXPECT_EQ(result.get(i, j), 0.0);
      }
    }
  }
}

TEST(Matrix4x4Tests, CombineTranslations) {
  Matrix4x4 t1 = Matrix4x4::translation(10, 20, 30);
  Matrix4x4 t2 = Matrix4x4::translation(5, 3, 7);

  Matrix4x4 result = t1 * t2;

  // Translations should add: (10+5, 20+3, 30+7)
  EXPECT_NEAR(result.get(0, 3), 15.0, 1e-6);
  EXPECT_NEAR(result.get(1, 3), 23.0, 1e-6);
  EXPECT_NEAR(result.get(2, 3), 37.0, 1e-6);

  // Should still be identity in rotation part
  EXPECT_NEAR(result.get(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(result.get(1, 1), 1.0, 1e-6);
  EXPECT_NEAR(result.get(2, 2), 1.0, 1e-6);
}

TEST(Matrix4x4Tests, CombineRotations_90DegreeZ) {
  // Two 90-degree Z rotations should equal 180 degrees
  Matrix4x4 rot90 = Matrix4x4::rotationZ(M_PI / 2);  // 90 degrees

  Matrix4x4 result = rot90 * rot90;

  // After 180° rotation around Z:
  // cos(180°) = -1, sin(180°) = 0
  EXPECT_NEAR(result.get(0, 0), -1.0, 1e-6);
  EXPECT_NEAR(result.get(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(result.get(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(result.get(1, 1), -1.0, 1e-6);
}

TEST(Matrix4x4Tests, RotationThenTranslation) {
  // Rotate 90° around Z, then translate
  Matrix4x4 rot = Matrix4x4::rotationZ(M_PI / 2);  // 90° Z rotation
  Matrix4x4 trans = Matrix4x4::translation(10, 0, 0);

  Matrix4x4 result = trans * rot;  // Apply rotation first, then translation

  // Translation happens in world space, so should be (10, 0, 0)
  EXPECT_NEAR(result.get(0, 3), 10.0, 1e-6);
  EXPECT_NEAR(result.get(1, 3), 0.0, 1e-6);
  EXPECT_NEAR(result.get(2, 3), 0.0, 1e-6);

  // Rotation part should still be 90° Z rotation
  EXPECT_NEAR(result.get(0, 0), 0.0, 1e-6);
  EXPECT_NEAR(result.get(0, 1), -1.0, 1e-6);
  EXPECT_NEAR(result.get(1, 0), 1.0, 1e-6);
  EXPECT_NEAR(result.get(1, 1), 0.0, 1e-6);
}

TEST(Matrix4x4Tests, TranslationThenRotation) {
  // Translate, then rotate 90° around Z
  Matrix4x4 trans = Matrix4x4::translation(10, 0, 0);
  Matrix4x4 rot = Matrix4x4::rotationZ(M_PI / 2);  // 90° Z rotation

  Matrix4x4 result = rot * trans;  // Apply translation first, then rotation

  // Point (10, 0, 0) rotated 90° around Z becomes (0, 10, 0)
  EXPECT_NEAR(result.get(0, 3), 0.0, 1e-6);
  EXPECT_NEAR(result.get(1, 3), 10.0, 1e-6);
  EXPECT_NEAR(result.get(2, 3), 0.0, 1e-6);
}

TEST(Matrix4x4Tests, MultipleRotations_XYZ) {
  // Combine rotations around different axes
  Matrix4x4 rotX = Matrix4x4::rotationX(M_PI / 4);  // 45° around X
  Matrix4x4 rotY = Matrix4x4::rotationY(M_PI / 4);  // 45° around Y
  Matrix4x4 rotZ = Matrix4x4::rotationZ(M_PI / 4);  // 45° around Z

  Matrix4x4 result = rotZ * rotY * rotX;

  // Just verify it doesn't crash and produces a valid matrix
  // Bottom row should still be [0, 0, 0, 1]
  EXPECT_NEAR(result.get(3, 0), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 1), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 2), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 3), 1.0, 1e-6);
}

TEST(Matrix4x4Tests, ComplexTransform_RotateTranslateRotate) {
  // Real-world example: joint transformations
  Matrix4x4 rot1 = Matrix4x4::rotationZ(M_PI / 6);      // 30° base rotation
  Matrix4x4 trans1 = Matrix4x4::translation(0, 0, 10);  // Move up 10
  Matrix4x4 rot2 = Matrix4x4::rotationY(M_PI / 3);      // 60° elbow rotation

  Matrix4x4 result = rot1 * trans1 * rot2;

  // Verify bottom row is still [0, 0, 0, 1]
  EXPECT_NEAR(result.get(3, 0), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 1), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 2), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 3), 1.0, 1e-6);

  // Z translation should be affected by first rotation but not second
  // This is a sanity check that the transformation is reasonable
  EXPECT_GT(result.get(2, 3), 9.0);  // Should be close to 10
  EXPECT_LT(result.get(2, 3), 11.0);
}

TEST(Matrix4x4Tests, AssociativityOfMultiplication) {
  // (A * B) * C should equal A * (B * C)
  Matrix4x4 A = Matrix4x4::rotationX(0.5);
  Matrix4x4 B = Matrix4x4::translation(5, 3, 2);
  Matrix4x4 C = Matrix4x4::rotationZ(0.7);

  Matrix4x4 result1 = (A * B) * C;
  Matrix4x4 result2 = A * (B * C);

  // Check all elements are equal (within tolerance)
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      EXPECT_NEAR(result1.get(i, j), result2.get(i, j), 1e-9)
          << "Element [" << i << "][" << j << "] differs";
    }
  }
}

TEST(Matrix4x4Tests, KnownMatrixMultiplication) {
  // Manually create simple matrices for known result
  std::array<std::array<double, 4>, 4> data1 = {
      {{1, 0, 0, 2}, {0, 1, 0, 3}, {0, 0, 1, 4}, {0, 0, 0, 1}}};

  std::array<std::array<double, 4>, 4> data2 = {
      {{1, 0, 0, 5}, {0, 1, 0, 6}, {0, 0, 1, 7}, {0, 0, 0, 1}}};

  Matrix4x4 m1(data1);
  Matrix4x4 m2(data2);

  Matrix4x4 result = m1 * m2;

  // Result should have translations (2+5, 3+6, 4+7) = (7, 9, 11)
  EXPECT_EQ(result.get(0, 3), 7.0);
  EXPECT_EQ(result.get(1, 3), 9.0);
  EXPECT_EQ(result.get(2, 3), 11.0);
}