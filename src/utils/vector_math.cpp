#include "vector_math.hpp"

/// @brief Computes the magnitude of a vector given length n
/// @param a Vector to compute the magnitude of
/// @param n Length of Vector a
/// @return returns the magnitude of a
float magnitude(float* a, int n) {
	float square_sum = 0;
	for (int i = 0; i < n; i++) {
		square_sum += pow(a[i], 2);
	}
	square_sum = sqrt(square_sum);
	return square_sum;
}

/// @brief Computes the dot product of 2 vectors with a given length (nx1)
/// @param a Vector A
/// @param b Vector B
/// @param n Length of A and B (must be the same length)
/// @return returns Dot product solution (scalar)
float vectorProduct(float* a, float* b, int n) {
	float product = 0;
	for (int i = 0; i < n; i++) {
		product += a[i] * b[i];
	}
	return product;
}

/// @brief Computes the cross product of 2 given vectors of length 3
/// @param v_A Vector A (3x1)
/// @param v_B Vector B (3x1)
/// @param output Cross product output vector (3x1)
void crossProduct(float v_A[ ], float v_B[ ], float output[ ]) {
	output[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
	output[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
	output[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
}

/// @brief Rotates input_vector around the given unit_vector by theta radians
/// @param unit_vector Vector to rotate around
/// @param input_vector Vector to be rotated
/// @param theta Angle to rotate (Rad)
/// @param output New rotated vector
void rotateVector3D(float unit_vector[ ], float input_vector[ ], float theta, float output[ ]) {
	float unit_cross_input[3];
	crossProduct(unit_vector, input_vector, unit_cross_input);
	output[0] = (input_vector[0] * cos(theta)) + (unit_cross_input[0] * sin(theta)) + (unit_vector[0] * vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
	output[1] = (input_vector[1] * cos(theta)) + (unit_cross_input[1] * sin(theta)) + (unit_vector[1] * vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
	output[2] = (input_vector[2] * cos(theta)) + (unit_cross_input[2] * sin(theta)) + (unit_vector[2] * vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
}

/// @brief This functions finds the determinant of a 3x3 Matrix
///@param mat matrix to find determinant of
///@return calculated determinant
float determinantOfMatrix(float mat[3][3]) {
	float ans;
	ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
	return ans;
}

/// @brief This function finds the solution of a 3x3 system of linear equations using cramer's rule.
/// @param coeff 3x3 coeff matrix for the system with 3x1 solution matrix added to the end
/// @param output 3x1 Array for the solutions
void solveSystem(float coeff[3][4], float output[3]) {
	// Matrix d using coeff as given in cramer's rule
	float d[3][3] = {
		{coeff[0][0], coeff[0][1], coeff[0][2]},
		{coeff[1][0], coeff[1][1], coeff[1][2]},
		{coeff[2][0], coeff[2][1], coeff[2][2]}
	};
	// Matrix d1 using coeff as given in cramer's rule
	float d1[3][3] = {
		{coeff[0][3], coeff[0][1], coeff[0][2]},
		{coeff[1][3], coeff[1][1], coeff[1][2]},
		{coeff[2][3], coeff[2][1], coeff[2][2]},
	};
	// Matrix d2 using coeff as given in cramer's rule
	float d2[3][3] = {
		{coeff[0][0], coeff[0][3], coeff[0][2]},
		{coeff[1][0], coeff[1][3], coeff[1][2]},
		{coeff[2][0], coeff[2][3], coeff[2][2]},
	};
	// Matrix d3 using coeff as given in cramer's rule
	float d3[3][3] = {
		{coeff[0][0], coeff[0][1], coeff[0][3]},
		{coeff[1][0], coeff[1][1], coeff[1][3]},
		{coeff[2][0], coeff[2][1], coeff[2][3]},
	};

	// Calculating Determinant of Matrices d, d1, d2, d3
	float D = determinantOfMatrix(d);
	float D1 = determinantOfMatrix(d1);
	float D2 = determinantOfMatrix(d2);
	float D3 = determinantOfMatrix(d3);

	// Case 1
	if (D != 0) {
		// Coeff have a unique solution
		output[0] = D1 / D;
		output[1] = D2 / D;
		output[2] = D3 / D;
	}
	// Case 2
	else {
		if (D1 == 0 && D2 == 0 && D3 == 0) {
			output[0] = 0;
			output[1] = 0;
			output[2] = 0;
		} else if (D1 != 0 || D2 != 0 || D3 != 0) {
			output[0] = 0;
			output[1] = 0;
			output[2] = 0;
		}
	}
}