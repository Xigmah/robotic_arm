#include "../math_util.h"

#include <gtest/gtest.h>

// ─── Helpers ────────────────────────────────────────────────────────────────

// Compare two matrices element-wise within tolerance
static bool matrices_equal(const vector<vector<double>>& a,
                           const vector<vector<double>>& b,
                           double tol = def_tol) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++) {
    if (a[i].size() != b[i].size()) return false;
    for (size_t j = 0; j < a[i].size(); j++) {
      if (!compare(a[i][j], b[i][j], tol)) return false;
    }
  }
  return true;
}

// Build an n x n identity matrix
static vector<vector<double>> identity(size_t n) {
  vector<vector<double>> id(n, vector<double>(n, 0.0));
  for (size_t i = 0; i < n; i++) id[i][i] = 1.0;
  return id;
}

// ─── deg_to_rad ─────────────────────────────────────────────────────────────

TEST(MathUtilTests, DegToRadZero) { EXPECT_DOUBLE_EQ(deg_to_rad(0.0), 0.0); }
TEST(MathUtilTests, DegToRadNinety) {
  EXPECT_NEAR(deg_to_rad(90.0), M_PI / 2, def_tol);
}
TEST(MathUtilTests, DegToRadOneEighty) {
  EXPECT_NEAR(deg_to_rad(180.0), M_PI, def_tol);
}
TEST(MathUtilTests, DegToRadNegative) {
  EXPECT_NEAR(deg_to_rad(-90.0), -M_PI / 2, def_tol);
}

// ─── compare ────────────────────────────────────────────────────────────────

TEST(MathUtilTests, CompareEqualValues) { EXPECT_TRUE(compare(1.0, 1.0)); }
TEST(MathUtilTests, CompareWithinTolerance) {
  EXPECT_TRUE(compare(1.0, 1.0 + 1e-7));
}
TEST(MathUtilTests, CompareOutsideTolerance) {
  EXPECT_FALSE(compare(1.0, 1.1));
}
TEST(MathUtilTests, CompareCustomTolerance) {
  EXPECT_TRUE(compare(1.0, 1.05, 0.1));
}
TEST(MathUtilTests, CompareNegativeValues) {
  EXPECT_TRUE(compare(-3.0, -3.0 + 1e-7));
}

// ─── clamp ──────────────────────────────────────────────────────────────────

TEST(MathUtilTests, ClampWithinRange) {
  EXPECT_DOUBLE_EQ(clamp(5.0, 0.0, 10.0), 5.0);
}
TEST(MathUtilTests, ClampBelowMin) {
  EXPECT_DOUBLE_EQ(clamp(-1.0, 0.0, 10.0), 0.0);
}
TEST(MathUtilTests, ClampAboveMax) {
  EXPECT_DOUBLE_EQ(clamp(15.0, 0.0, 10.0), 10.0);
}
TEST(MathUtilTests, ClampAtMin) {
  EXPECT_DOUBLE_EQ(clamp(0.0, 0.0, 10.0), 0.0);
}
TEST(MathUtilTests, ClampAtMax) {
  EXPECT_DOUBLE_EQ(clamp(10.0, 0.0, 10.0), 10.0);
}

// ─── matrix_consistent ──────────────────────────────────────────────────────

TEST(MathUtilTests, MatrixConsistentEmptyMatrix) {
  EXPECT_TRUE(matrix_consistent({}));
}

TEST(MathUtilTests, MatrixConsistentUniformRowLengths) {
  EXPECT_TRUE(matrix_consistent({{1, 2}, {3, 4}, {5, 6}}));
}

TEST(MathUtilTests, MatrixConsistentJaggedRows) {
  EXPECT_FALSE(matrix_consistent({{1, 2}, {3}, {5, 6}}));
}

TEST(MathUtilTests, MatrixConsistentSingleRow) {
  EXPECT_TRUE(matrix_consistent({{1, 2, 3}}));
}

// ─── transpose_matrix ───────────────────────────────────────────────────────

TEST(MathUtilTests, TransposeMatrixEmptyMatrix) {
  EXPECT_FALSE(transpose_matrix({}).has_value());
}

TEST(MathUtilTests, TransposeMatrixSquare2x2) {
  vector<vector<double>> mat = {{1, 2}, {3, 4}};
  vector<vector<double>> expected = {{1, 3}, {2, 4}};
  auto result = transpose_matrix(mat);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

TEST(MathUtilTests, TransposeMatrixRectangular2x3) {
  vector<vector<double>> mat = {{1, 2, 3}, {4, 5, 6}};
  vector<vector<double>> expected = {{1, 4}, {2, 5}, {3, 6}};
  auto result = transpose_matrix(mat);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

TEST(MathUtilTests, TransposeMatrixSingleRow) {
  vector<vector<double>> mat = {{1, 2, 3}};
  vector<vector<double>> expected = {{1}, {2}, {3}};
  auto result = transpose_matrix(mat);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

TEST(MathUtilTests, TransposeMatrixDoubleTransposeIsIdentity) {
  vector<vector<double>> mat = {{1, 2, 3}, {4, 5, 6}};
  auto once = transpose_matrix(mat);
  auto twice = transpose_matrix(*once);
  ASSERT_TRUE(twice.has_value());
  EXPECT_TRUE(matrices_equal(*twice, mat));
}

// ─── add_matrices ────────────────────────────────────────────────────────────

TEST(MathUtilTests, AddMatricesEmptyA) {
  EXPECT_FALSE(add_matrices({}, {{1, 2}}).has_value());
}

TEST(MathUtilTests, AddMatricesEmptyB) {
  EXPECT_FALSE(add_matrices({{1, 2}}, {}).has_value());
}

TEST(MathUtilTests, AddMatricesDimensionMismatch) {
  vector<vector<double>> a = {{1, 2}, {3, 4}};
  vector<vector<double>> b = {{1, 2, 3}, {4, 5, 6}};
  EXPECT_FALSE(add_matrices(a, b).has_value());
}

TEST(MathUtilTests, AddMatricesSquare2x2) {
  vector<vector<double>> a = {{1, 2}, {3, 4}};
  vector<vector<double>> b = {{5, 6}, {7, 8}};
  vector<vector<double>> expected = {{6, 8}, {10, 12}};
  auto result = add_matrices(a, b);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

TEST(MathUtilTests, AddMatricesRectangular) {
  vector<vector<double>> a = {{1, 2, 3}, {4, 5, 6}};
  vector<vector<double>> b = {{7, 8, 9}, {10, 11, 12}};
  vector<vector<double>> expected = {{8, 10, 12}, {14, 16, 18}};
  auto result = add_matrices(a, b);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

TEST(MathUtilTests, AddMatricesWithZeroMatrix) {
  vector<vector<double>> a = {{1, 2}, {3, 4}};
  vector<vector<double>> zero = {{0, 0}, {0, 0}};
  auto result = add_matrices(a, zero);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, a));
}

TEST(MathUtilTests, AddMatricesWithNegatives) {
  vector<vector<double>> a = {{1, 2}, {3, 4}};
  vector<vector<double>> b = {{-1, -2}, {-3, -4}};
  vector<vector<double>> expected = {{0, 0}, {0, 0}};
  auto result = add_matrices(a, b);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

// ─── multiple_matrix_by_scalar ───────────────────────────────────────────────

TEST(MathUtilTests, MultipleMatrixByScalarEmptyMatrix) {
  vector<vector<double>> mat = {};
  multiple_matrix_by_scalar(mat, 2.0);
  EXPECT_TRUE(mat.empty());
}

TEST(MathUtilTests, MultipleMatrixByScalarByTwo) {
  vector<vector<double>> mat = {{1, 2}, {3, 4}};
  vector<vector<double>> expected = {{2, 4}, {6, 8}};
  multiple_matrix_by_scalar(mat, 2.0);
  EXPECT_TRUE(matrices_equal(mat, expected));
}

TEST(MathUtilTests, MultipleMatrixByScalarByZero) {
  vector<vector<double>> mat = {{1, 2}, {3, 4}};
  vector<vector<double>> expected = {{0, 0}, {0, 0}};
  multiple_matrix_by_scalar(mat, 0.0);
  EXPECT_TRUE(matrices_equal(mat, expected));
}

TEST(MathUtilTests, MultipleMatrixByScalarByOne) {
  vector<vector<double>> mat = {{1, 2}, {3, 4}};
  vector<vector<double>> expected = {{1, 2}, {3, 4}};
  multiple_matrix_by_scalar(mat, 1.0);
  EXPECT_TRUE(matrices_equal(mat, expected));
}

TEST(MathUtilTests, MultipleMatrixByScalarByNegative) {
  vector<vector<double>> mat = {{1, 2}, {3, 4}};
  vector<vector<double>> expected = {{-1, -2}, {-3, -4}};
  multiple_matrix_by_scalar(mat, -1.0);
  EXPECT_TRUE(matrices_equal(mat, expected));
}

// ─── multiple_matrices ──────────────────────────────────────────────────────

TEST(MathUtilTests, MultipleMatricesEmptyA) {
  EXPECT_FALSE(multiple_matrices({}, {{1, 2}}).has_value());
}

TEST(MathUtilTests, MultipleMatricesEmptyB) {
  EXPECT_FALSE(multiple_matrices({{1, 2}}, {}).has_value());
}

TEST(MathUtilTests, MultipleMatricesDimensionMismatch) {
  // A is 2x3, B is 2x2 — A cols != B rows
  vector<vector<double>> a = {{1, 2, 3}, {4, 5, 6}};
  vector<vector<double>> b = {{1, 2}, {3, 4}};
  EXPECT_FALSE(multiple_matrices(a, b).has_value());
}

TEST(MathUtilTests, MultipleMatricesIdentityMultiply) {
  vector<vector<double>> mat = {{1, 2}, {3, 4}};
  auto result = multiple_matrices(mat, identity(2));
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, mat));
}

TEST(MathUtilTests, MultipleMatricesSquare2x2) {
  vector<vector<double>> a = {{1, 2}, {3, 4}};
  vector<vector<double>> b = {{5, 6}, {7, 8}};
  vector<vector<double>> expected = {{19, 22}, {43, 50}};
  auto result = multiple_matrices(a, b);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

TEST(MathUtilTests, MultipleMatricesRectangular2x3by3x2) {
  vector<vector<double>> a = {{1, 2, 3}, {4, 5, 6}};
  vector<vector<double>> b = {{7, 8}, {9, 10}, {11, 12}};
  vector<vector<double>> expected = {{58, 64}, {139, 154}};
  auto result = multiple_matrices(a, b);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

// ─── inverse_matrix ─────────────────────────────────────────────────────────

TEST(MathUtilTests, InverseMatrixEmptyMatrix) {
  EXPECT_FALSE(inverse_matrix({}).has_value());
}

TEST(MathUtilTests, InverseMatrixNonSquare) {
  EXPECT_FALSE(inverse_matrix({{1, 2, 3}, {4, 5, 6}}).has_value());
}

TEST(MathUtilTests, InverseMatrixSingularMatrix) {
  // Rows are linearly dependent — no inverse
  vector<vector<double>> mat = {{1, 2}, {2, 4}};
  EXPECT_FALSE(inverse_matrix(mat).has_value());
}

TEST(MathUtilTests, InverseMatrixIdentity2x2) {
  auto result = inverse_matrix(identity(2));
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, identity(2)));
}

TEST(MathUtilTests, InverseMatrixKnown2x2) {
  vector<vector<double>> mat = {{4, 7}, {2, 6}};
  vector<vector<double>> expected = {{0.6, -0.7}, {-0.2, 0.4}};
  auto result = inverse_matrix(mat);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(matrices_equal(*result, expected));
}

TEST(MathUtilTests, InverseMatrixKnown3x3) {
  vector<vector<double>> mat = {{1, 2, 3}, {0, 1, 4}, {5, 6, 0}};
  auto result = inverse_matrix(mat);
  ASSERT_TRUE(result.has_value());
  // A * A^-1 should equal identity
  auto product = multiple_matrices(mat, *result);
  ASSERT_TRUE(product.has_value());
  EXPECT_TRUE(matrices_equal(*product, identity(3)));
}

TEST(MathUtilTests, InverseMatrixATimesInverseIsIdentity) {
  // Property test: works for any invertible matrix
  vector<vector<double>> mat = {{2, 1, 0}, {1, 3, 1}, {0, 1, 2}};
  auto inv = inverse_matrix(mat);
  ASSERT_TRUE(inv.has_value());
  auto product = multiple_matrices(mat, *inv);
  ASSERT_TRUE(product.has_value());
  EXPECT_TRUE(matrices_equal(*product, identity(3)));
}
