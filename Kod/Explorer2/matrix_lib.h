/* 2016-06-06
 * Johan Isaksson
 * Simple matrix library
 *
 *
 *
*/

#ifndef MATRIX_H
#define MATRIX_H

//typedef double value;
typedef float value;

struct matrix
{
	int rows;
	int columns;
	value* data;
};

matrix matrix_create(int r, int c, value* d);

void matrix_zeroes(matrix A);

void matrix_identity(matrix A);

void matrix_multiply(matrix R, matrix A, matrix B);

void matrix_scale(matrix R, matrix A, value b );

void matrix_add(matrix R, matrix A, matrix B);

void matrix_subtract(matrix R, matrix A, matrix B);

void matrix_transpose(matrix R, matrix A);

void matrix_inverse(matrix R, matrix A);

/*
matrix operator*(matrix A, matrix B);
matrix operator+(matrix A, matrix B);
matrix operator-(matrix A, matrix B);
matrix operator/(matrix A, matrix B);

matrix operator*(matrix A, double b);
matrix operator*(double b, matrix A);
*/


#endif /* MATRIX_H */

