/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-16 16:35:51
 * @modify date 2026-02-16 16:35:51
 * @desc Header file for basic math utils
 */
#pragma once

#include <stdio.h>

#include <cmath>
#include <optional>
#include <vector>

using std::optional;
using std::vector;

inline static constexpr double def_tol = 1e-6;

inline double deg_to_rad(double deg) { return deg * (M_PI / 180); }

inline bool compare(double v1, double v2, double tol = def_tol) {
  return fabs(v1 - v2) <= tol;
}

inline double clamp(double value, double min, double max) {
  return fmax(min, fmin(value, max));
}

// Generate identity matrix
inline vector<vector<double>> generate_identity(size_t m_size) {
  vector<vector<double>> identity(m_size, vector<double>(m_size, 0.0));
  for (size_t row{0}; row < m_size; row++) {
    // Where row = col, set element to 1.0
    identity[row][row] = 1.0;
  }
  return identity;
}

// Matrix printer, if needed for debugging
inline void print_matrix(const vector<vector<double>>& matrix) {
  printf("\n");
  for (size_t i{0}; i < matrix.size(); i++) {
    printf("| ");
    for (size_t j{0}; j < matrix[0].size(); j++) {
      printf("%0.2f ", matrix[i][j]);
    }
    printf("|\n");
  }
  printf("\n");
}

// Checking that matrix has consistent columns across rows
inline bool matrix_consistent(const vector<vector<double>>& matrix) {
  // Check if matrix is empty
  if (matrix.empty()) {
    printf("Matrix is empty\n");
    return true;
  }

  // Setting first column size
  size_t row_size = matrix[0].size();
  for (const auto& row : matrix) {
    if (row.size() != row_size) {
      printf("Matrix columns in each row are not consistent\n");
      return false;
    }
  }
  return true;
}

// Transpose Matrix
inline optional<vector<vector<double>>> transpose_matrix(
    const vector<vector<double>>& mat) {
  // Check for empty matrices
  if (mat.empty()) {
    printf("Matrix is empty\n");
    return std::nullopt;
  }

  // Check if Matrix is consistent
  if (!matrix_consistent(mat)) {
    return std::nullopt;
  }

  // Resultant matrix, col x rows
  vector<vector<double>> result(mat[0].size(), vector<double>(mat.size()));

  for (size_t row{0}; row < mat.size(); row++) {
    for (size_t col{0}; col < mat[0].size(); col++) {
      result[col][row] = mat[row][col];
    }
  }
  return result;
}

/* Matrix Addition
 * Matrices must have the same dimensions
 */
inline optional<vector<vector<double>>> add_matrices(
    const vector<vector<double>>& a_mat, const vector<vector<double>>& b_mat) {
  // Check for empty matrices
  if (a_mat.empty() || b_mat.empty()) {
    printf("One of the matrices are empty\n");
    return std::nullopt;
  }

  // Check if A and B are the same dimension
  if (a_mat.size() != b_mat.size() || a_mat[0].size() != b_mat[0].size()) {
    printf("Matrices must have the same dimensions\n");
    return std::nullopt;
  }

  // Check if both matrices are consistent
  if (!matrix_consistent(a_mat) || !matrix_consistent(b_mat)) {
    return std::nullopt;
  }

  vector<vector<double>> resultant(a_mat.size(),
                                   vector<double>(a_mat[0].size(), 0.0));
  for (size_t row{0}; row < a_mat.size(); row++) {
    for (size_t col{0}; col < a_mat[row].size(); col++) {
      resultant[row][col] = a_mat[row][col] + b_mat[row][col];
    }
  }

  return resultant;
}

// Multiply Matrix by Scalar
inline void multiple_matrix_by_scalar(vector<vector<double>>& mat,
                                      const double& scalar) {
  // Check for empty matrices
  if (mat.empty()) {
    printf("Matrix is empty\n");
    return;
  }

  // Check if Matrix is consistent
  if (!matrix_consistent(mat)) {
    return;
  }

  for (auto& row : mat) {
    for (auto& element : row) {
      element *= scalar;
    }
  }
}

/* Matrix multiplication
 * If A is an m x n matrix,
 * and B is a n x p matrix,
 * then:
 *           n-1
 * C[i][j] = ∑ A[i][k] * B[k][j]
 *           k=0
 * Where i = 1 -> m and j = 1 -> p
 * and A has the same number of columns as B has rows
 */
inline optional<vector<vector<double>>> multiple_matrices(
    const vector<vector<double>>& a_mat, const vector<vector<double>>& b_mat) {
  // Check for empty matrices
  if (a_mat.empty() || b_mat.empty()) {
    printf("One of the matrices are empty\n");
    return std::nullopt;
  }

  // Check if A columns and B rows are equivalent
  if (a_mat[0].size() != b_mat.size()) {
    printf("A columns and B rows are not equivalent\n");
    return std::nullopt;
  }

  // Check if both matrices are consistent
  if (!matrix_consistent(a_mat) || !matrix_consistent(b_mat)) {
    return std::nullopt;
  }

  // Resultant matrix, m x p
  vector<vector<double>> result(a_mat.size(), vector<double>(b_mat[0].size()));

  // Loop through A rows
  for (size_t row{0}; row < a_mat.size(); row++) {
    // Loop through B columns
    for (size_t col{0}; col < b_mat[0].size(); col++) {
      // Prepare summation before loop
      double sum{0};

      // Loop through A columns/B rows
      for (size_t sum_enum{0}; sum_enum < a_mat[row].size(); sum_enum++) {
        sum += a_mat[row][sum_enum] * b_mat[sum_enum][col];
      }

      // Apply summation
      result[row][col] = sum;
    }
  }
  return result;
}

inline optional<vector<vector<double>>> inverse_matrix(
    const vector<vector<double>>& mat) {
  // Check for empty matrix
  if (mat.empty()) {
    printf("Matrix is empty\n");
    return std::nullopt;
  }

  // Check matrix is square
  if (mat.size() != mat[0].size()) {
    printf("Matrix is not square\n");
    return std::nullopt;
  }

  // Check consistency
  if (!matrix_consistent(mat)) {
    return std::nullopt;
  }

  size_t n = mat.size();

  /* Build augmented matrix [A | I]
   * for use in Gauss-Jordan elimination
   */
  vector<vector<double>> augmented(n, vector<double>(2 * n, 0.0));

  for (size_t row{0}; row < n; row++) {
    for (size_t col{0}; col < n; col++) {
      augmented[row][col] = mat[row][col];  // Left side: copy of A
    }
    augmented[row][row + n] = 1.0;  // Right side: identity
  }

  // Gauss-Jordan elimination
  // TODO: Rewrite to clarify and explain better
  for (size_t col{0}; col < n; col++) {
    // Find pivot (largest value in column to reduce floating point error)
    size_t pivot = col;
    for (size_t row{col + 1}; row < n; row++) {
      if (fabs(augmented[row][col]) > fabs(augmented[pivot][col])) {
        pivot = row;
      }
    }

    // Swap pivot row to current row
    swap(augmented[col], augmented[pivot]);

    // Check if matrix is singular (non-invertible)
    if (fabs(augmented[col][col]) < 1e-10) {
      return std::nullopt;
    }

    // Scale pivot row so the diagonal becomes 1
    double scale = augmented[col][col];
    for (size_t j{0}; j < 2 * n; j++) {
      augmented[col][j] /= scale;
    }

    // Eliminate all other rows in this column
    for (size_t row{0}; row < n; row++) {
      if (row == col) continue;
      double factor = augmented[row][col];
      for (size_t j{0}; j < 2 * n; j++) {
        augmented[row][j] -= factor * augmented[col][j];
      }
    }
  }

  // Extract right half as the inverse
  vector<vector<double>> result(n, vector<double>(n));
  for (size_t i{0}; i < n; i++) {
    for (size_t j{0}; j < n; j++) {
      result[i][j] = augmented[i][j + n];
    }
  }

  return result;
}
