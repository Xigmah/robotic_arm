---

## Understanding Matrix4x4 Structure

Before we code, let's visualize what we're building:
```
Matrix4x4 stored as:
     col 0   col 1   col 2   col 3
   ┌─────────────────────────────────┐
r0 │ m[0][0] m[0][1] m[0][2] m[0][3] │  ← Translation X
r1 │ m[1][0] m[1][1] m[1][2] m[1][3] │  ← Translation Y
r2 │ m[2][0] m[2][1] m[2][2] m[2][3] │  ← Translation Z
r3 │ m[3][0] m[3][1] m[3][2] m[3][3] │  ← Always [0 0 0 1]
   └─────────────────────────────────┘
      ↑
   Rotation part (3x3 top-left)
```

**Identity Matrix** (does nothing, like multiplying by 1):
```
┌            ┐
│ 1  0  0  0 │
│ 0  1  0  0 │
│ 0  0  1  0 │
│ 0  0  0  1 │
└            ┘

---

### Part 5: Matrix Multiplication (HARD - We'll Do This Together)

This is the **most important** and **trickiest** part. Matrix multiplication is NOT element-wise - there's a specific algorithm.

**The Rule:**
```
C[i][j] = sum of (A[i][k] * B[k][j]) for all k
```

In code:
```cpp
Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const {
  Matrix4x4 result;
  
  // For each row i in result
  for (size_t i = 0; i < SIZE; ++i) {
    // For each column j in result
    for (size_t j = 0; j < SIZE; ++j) {
      // Calculate dot product of row i from *this and column j from other
      double sum = 0.0;
      for (size_t k = 0; k < SIZE; ++k) {
        sum += m[i][k] * other.m[k][j];
      }
      result.set(i, j, sum);
    }
  }
  
  return result;
}
```

**Visual Example of what's happening:**
```
A * B = C

A[row 0] · B[col 0] = C[0][0]
A[row 0] · B[col 1] = C[0][1]
...

Where "·" means dot product:
[a, b, c, d] · [e, f, g, h] = a*e + b*f + c*g + d*h
```

---

### Part 6: Extract Translation (Easy)

```cpp
Vector3D Matrix4x4::getTranslation() const {
  return Vector3D(m[0][3], m[1][3], m[2][3]);
}
```

---

## Your Google Test for Matrix4x4

```cpp
#include "matrix4x4.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace arm::math;

TEST(Matrix4x4Tests, Identity) {
  Matrix4x4 identity = Matrix4x4::identity();
  
  // Check diagonal is 1
  EXPECT_EQ(identity.get(0, 0), 1.0);
  EXPECT_EQ(identity.get(1, 1), 1.0);
  EXPECT_EQ(identity.get(2, 2), 1.0);
  EXPECT_EQ(identity.get(3, 3), 1.0);
  
  // Check off-diagonal is 0
  EXPECT_EQ(identity.get(0, 1), 0.0);
  EXPECT_EQ(identity.get(1, 0), 0.0);
}

TEST(Matrix4x4Tests, Translation) {
  Matrix4x4 trans = Matrix4x4::translation(10, 20, 30);
  
  EXPECT_EQ(trans.get(0, 3), 10.0);
  EXPECT_EQ(trans.get(1, 3), 20.0);
  EXPECT_EQ(trans.get(2, 3), 30.0);
  
  Vector3D pos = trans.getTranslation();
  EXPECT_EQ(pos.get_x(), 10.0);
  EXPECT_EQ(pos.get_y(), 20.0);
  EXPECT_EQ(pos.get_z(), 30.0);
}

TEST(Matrix4x4Tests, RotationZ_90Degrees) {
  Matrix4x4 rot = Matrix4x4::rotationZ(M_PI / 2);  // 90 degrees
  
  // cos(90°) = 0, sin(90°) = 1
  EXPECT_NEAR(rot.get(0, 0), 0.0, 1e-6);
  EXPECT_NEAR(rot.get(0, 1), -1.0, 1e-6);
  EXPECT_NEAR(rot.get(1, 0), 1.0, 1e-6);
  EXPECT_NEAR(rot.get(1, 1), 0.0, 1e-6);
}

TEST(Matrix4x4Tests, MultiplyByIdentity) {
  Matrix4x4 trans = Matrix4x4::translation(5, 10, 15);
  Matrix4x4 identity = Matrix4x4::identity();
  
  Matrix4x4 result = trans * identity;
  
  // Should be unchanged
  EXPECT_EQ(result.get(0, 3), 5.0);
  EXPECT_EQ(result.get(1, 3), 10.0);
  EXPECT_EQ(result.get(2, 3), 15.0);
}

TEST(Matrix4x4Tests, CombineTranslations) {
  Matrix4x4 t1 = Matrix4x4::translation(10, 0, 0);
  Matrix4x4 t2 = Matrix4x4::translation(5, 0, 0);
  
  Matrix4x4 result = t1 * t2;
  
  // Should combine: 10 + 5 = 15
  EXPECT_NEAR(result.get(0, 3), 15.0, 1e-6);
}
```

---

## Your Action Items

1. **Fix the constructor default parameter bug** in Vector3D
2. **Implement the Matrix4x4 constructors and element access** (easy warmup)
3. **Try implementing `translation()`** - test it!
4. **Try implementing `rotationX()` and `rotationY()`** - test them!
5. **Copy my matrix multiplication** (it's tricky, don't feel bad using mine)
6. **Run your tests!**

Once you have Matrix4x4 working, we'll build the `Transform` class (which is just a nice wrapper), and then you're ready for kinematics!

How does this feel? Want to take a crack at implementing the rotation functions, or would you like me to show you those too?
