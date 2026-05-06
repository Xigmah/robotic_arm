#include "../transform.h"

#include <gtest/gtest.h>

using namespace arm::math;

static constexpr size_t SIZE = 4;

TEST(TransformTests, Constructor) {
  Transform mat;

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

TEST(TransformTests, GetThrowsOnOutOfBounds) {
  Transform mat;

  // Valid access (should not throw)
  EXPECT_NO_THROW(mat.get(0, 0));
  EXPECT_NO_THROW(mat.get(3, 3));

  // Invalid access (should throw)
  EXPECT_THROW(mat.get(4, 0), std::out_of_range);
  EXPECT_THROW(mat.get(0, 4), std::out_of_range);
  EXPECT_THROW(mat.get(4, 4), std::out_of_range);
}

TEST(TransformTests, SetThrowsOnOutOfBounds) {
  Transform mat;

  // Valid access (should not throw)
  EXPECT_NO_THROW(mat.set(0, 0, 5.0));
  EXPECT_NO_THROW(mat.set(3, 3, 5.0));

  // Invalid access (should throw)
  EXPECT_THROW(mat.set(4, 0, 5.0), std::out_of_range);
  EXPECT_THROW(mat.set(0, 4, 5.0), std::out_of_range);
  EXPECT_THROW(mat.set(4, 4, 5.0), std::out_of_range);
}

TEST(TransformTests, EqualsIdenticalMatrices) {
  Transform m1 = Transform::identity();
  Transform m2 = Transform::identity();

  EXPECT_TRUE(m1.equals(m2));
}

TEST(TransformTests, EqualsSameObject) {
  Transform m1 = Transform::translation(5, 10, 15);

  // Comparing to itself should always be true
  EXPECT_TRUE(m1.equals(m1));
}

TEST(TransformTests, EqualsSameMatrices) {
  Transform m1 = Transform::translation(5, 10, 15);
  Transform m2 = Transform::translation(5, 10, 15);

  EXPECT_TRUE(m1.equals(m2));
}

TEST(TransformTests, EqualsDifferentMatrices) {
  Transform m1 = Transform::translation(5, 10, 15);
  Transform m2 = Transform::translation(6, 10, 15);

  EXPECT_FALSE(m1.equals(m2));
}

TEST(TransformTests, Translation) {
  Transform mat = Transform::translation(10.0, 20.0, 30.0);

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

TEST(TransformTests, TranslationVector) {
  Transform mat = Transform::translation(10.0, 20.0, 30.0);
  Vector3D v = mat.getTranslation();

  EXPECT_EQ(v.get_x(), 10.0);
  EXPECT_EQ(v.get_y(), 20.0);
  EXPECT_EQ(v.get_z(), 30.0);
}

TEST(TransformTests, RotationX) {
  Transform mat = Transform::rotationX(10.0);

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

TEST(TransformTests, RotationY) {
  Transform mat = Transform::rotationY(10.0);

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

TEST(TransformTests, RotationZ) {
  Transform mat = Transform::rotationZ(10.0);

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

TEST(TransformTests, MultiplyByIdentity) {
  Transform trans = Transform::translation(5, 10, 15);
  Transform identity = Transform::identity();

  Transform result1 = trans * identity;
  Transform result2 = identity * trans;

  // Multiplying by identity should not change the matrix
  EXPECT_EQ(result1.get(0, 3), 5.0);
  EXPECT_EQ(result1.get(1, 3), 10.0);
  EXPECT_EQ(result1.get(2, 3), 15.0);

  // Identity multiplication is commutative
  EXPECT_EQ(result2.get(0, 3), 5.0);
  EXPECT_EQ(result2.get(1, 3), 10.0);
  EXPECT_EQ(result2.get(2, 3), 15.0);
}

TEST(TransformTests, IdentityMultiplyIdentity) {
  Transform id1 = Transform::identity();
  Transform id2 = Transform::identity();

  Transform result = id1 * id2;

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

TEST(TransformTests, CombineTranslations) {
  Transform t1 = Transform::translation(10, 20, 30);
  Transform t2 = Transform::translation(5, 3, 7);

  Transform result = t1 * t2;

  // Translations should add: (10+5, 20+3, 30+7)
  EXPECT_NEAR(result.get(0, 3), 15.0, 1e-6);
  EXPECT_NEAR(result.get(1, 3), 23.0, 1e-6);
  EXPECT_NEAR(result.get(2, 3), 37.0, 1e-6);

  // Should still be identity in rotation part
  EXPECT_NEAR(result.get(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(result.get(1, 1), 1.0, 1e-6);
  EXPECT_NEAR(result.get(2, 2), 1.0, 1e-6);
}

TEST(TransformTests, CombineRotations_90DegreeZ) {
  // Two 90-degree Z rotations should equal 180 degrees
  Transform rot90 = Transform::rotationZ(M_PI / 2);  // 90 degrees

  Transform result = rot90 * rot90;

  // After 180° rotation around Z:
  // cos(180°) = -1, sin(180°) = 0
  EXPECT_NEAR(result.get(0, 0), -1.0, 1e-6);
  EXPECT_NEAR(result.get(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(result.get(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(result.get(1, 1), -1.0, 1e-6);
}

TEST(TransformTests, RotationThenTranslation) {
  // Rotate 90° around Z, then translate
  Transform rot = Transform::rotationZ(M_PI / 2);  // 90° Z rotation
  Transform trans = Transform::translation(10, 0, 0);

  Transform result = trans * rot;  // Apply rotation first, then translation

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

TEST(TransformTests, TranslationThenRotation) {
  // Translate, then rotate 90° around Z
  Transform trans = Transform::translation(10, 0, 0);
  Transform rot = Transform::rotationZ(M_PI / 2);  // 90° Z rotation

  Transform result = rot * trans;  // Apply translation first, then rotation

  // Point (10, 0, 0) rotated 90° around Z becomes (0, 10, 0)
  EXPECT_NEAR(result.get(0, 3), 0.0, 1e-6);
  EXPECT_NEAR(result.get(1, 3), 10.0, 1e-6);
  EXPECT_NEAR(result.get(2, 3), 0.0, 1e-6);
}

TEST(TransformTests, MultipleRotations_XYZ) {
  // Combine rotations around different axes
  Transform rotX = Transform::rotationX(M_PI / 4);  // 45° around X
  Transform rotY = Transform::rotationY(M_PI / 4);  // 45° around Y
  Transform rotZ = Transform::rotationZ(M_PI / 4);  // 45° around Z

  Transform result = rotZ * rotY * rotX;

  // Just verify it doesn't crash and produces a valid matrix
  // Bottom row should still be [0, 0, 0, 1]
  EXPECT_NEAR(result.get(3, 0), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 1), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 2), 0.0, 1e-6);
  EXPECT_NEAR(result.get(3, 3), 1.0, 1e-6);
}

TEST(TransformTests, RotateTranslateRotate) {
  // Real-world example: joint transformations
  Transform rot1 = Transform::rotationZ(M_PI / 6);      // 30° base rotation
  Transform trans1 = Transform::translation(0, 0, 10);  // Move up 10
  Transform rot2 = Transform::rotationY(M_PI / 3);      // 60° elbow rotation

  Transform result = rot1 * trans1 * rot2;

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

TEST(TransformTests, AssociativityOfMultiplication) {
  // (A * B) * C should equal A * (B * C)
  Transform A = Transform::rotationX(0.5);
  Transform B = Transform::translation(5, 3, 2);
  Transform C = Transform::rotationZ(0.7);

  Transform result1 = (A * B) * C;
  Transform result2 = A * (B * C);

  // Check all elements are equal (within tolerance)
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      EXPECT_NEAR(result1.get(i, j), result2.get(i, j), 1e-9)
          << "Element [" << i << "][" << j << "] differs";
    }
  }
}

TEST(TransformTests, KnownMatrixMultiplication) {
  // Manually create simple matrices for known result
  std::array<std::array<double, 4>, 4> data1 = {
      {{1, 0, 0, 2}, {0, 1, 0, 3}, {0, 0, 1, 4}, {0, 0, 0, 1}}};

  std::array<std::array<double, 4>, 4> data2 = {
      {{1, 0, 0, 5}, {0, 1, 0, 6}, {0, 0, 1, 7}, {0, 0, 0, 1}}};

  Transform m1(data1);
  Transform m2(data2);

  Transform result = m1 * m2;

  // Result should have translations (2+5, 3+6, 4+7) = (7, 9, 11)
  EXPECT_EQ(result.get(0, 3), 7.0);
  EXPECT_EQ(result.get(1, 3), 9.0);
  EXPECT_EQ(result.get(2, 3), 11.0);
}

TEST(TransformTests, ColumnGetter) {
  Transform t;

  // Transform constructs an Identity matrix,
  //  so first column should be (1,0,0)
  std::array<double, 3> column{t.getRotColumn(0)};
  EXPECT_EQ(column[0], 1.0);
  EXPECT_EQ(column[1], 0.0);
  EXPECT_EQ(column[2], 0.0);
}

TEST(TransformTests, ChainedTransformations) {
  Transform t =
      Transform().translate(10, 0, 0).rotateZ(M_PI / 2).translate(5, 0, 0);

  Vector3D pos = t.getTranslation();

  // After rotation, second translation goes in different direction
  EXPECT_NEAR(pos.get_x(), 10.0, 1e-6);
  EXPECT_NEAR(pos.get_y(), 5.0, 1e-6);
}

TEST(TransformTests, MultiplyTransforms) {
  Transform t1 = Transform::translation(10, 0, 0);
  Transform t2 = Transform::translation(5, 0, 0);

  Transform result = t1 * t2;
  Vector3D pos = result.getTranslation();

  EXPECT_NEAR(pos.get_x(), 15.0, 1e-6);
}

TEST(TransformTests, FactoryMethods) {
  Transform t = Transform::rotationZ(M_PI / 4);

  // Should have rotation but no translation
  Vector3D pos = t.getTranslation();
  EXPECT_EQ(pos.get_x(), 0.0);
  EXPECT_EQ(pos.get_y(), 0.0);
}
