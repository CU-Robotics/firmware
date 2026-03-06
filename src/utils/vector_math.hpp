#include <math.h>

#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H
namespace Utils {

/// @brief Computes the magnitude of a vector given length n
/// @param a Vector to compute the magnitude of
/// @param n Length of Vector a
/// @return returns the magnitude of a
float magnitude(float* a, int n);

/// @brief Computes the dot product of 2 vectors with a given length (nx1)
/// @param a Vector A
/// @param b Vector B
/// @param n Length of A and B (must be the same length)
/// @return returns Dot product solution (scalar)
float vectorProduct(float* a, float* b, int n);

/// @brief Computes the cross product of 2 given vectors of length 3
/// @param v_A Vector A (3x1)
/// @param v_B Vector B (3x1)
/// @param output Cross product output vector (3x1)
void crossProduct(float v_A[ ], float v_B[ ], float output[ ]);

/// @brief Rotates input_vector around the given unit_vector by theta radians
/// @param unit_vector Vector to rotate around
/// @param input_vector Vector to be rotated
/// @param theta Angle to rotate (Rad)
/// @param output New rotated vector
void rotateVector3D(float unit_vector[ ], float input_vector[ ], float theta, float output[ ]);

/// @brief This functions finds the determinant of a 3x3 Matrix
///@param mat matrix to find determinant of
///@return calculated determinant
float determinantOfMatrix(float mat[3][3]);

/// @brief This function finds the solution of a 3x3 system of linear equations using cramer's rule.
/// @param coeff 3x3 coeff matrix for the system with 3x1 solution matrix added to the end
/// @param output 3x1 Array for the solutions
void solveSystem(float coeff[3][4], float output[3]);

} // namespace Utils
#endif // VECTOR_MATH_H