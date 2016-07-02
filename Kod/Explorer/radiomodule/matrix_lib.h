
/* 2016-06-06
 * Johan Isaksson
 * Simple matrix library
 *
 *
 *
*/

template <int r, int c>
struct matrix
{
	int rows = r;
	int columns = c;
	double data[r][c];
};


template <int r1, int c1, int r2, int c2>
matrix<r1, c2> matrix_multiply(matrix<r1, c1> A, matrix<r2, c2> B);

template <int r, int c>
matrix<r, c> matrix_add(matrix<r, c> A, matrix<r, c> B);

template <int r, int c>
matrix<r, c> matrix_scale(matrix<r, c> A, double b);

template <int r, int c>
matrix<r, c> matrix_subtract(matrix<r, c> A, matrix<r, c> B);

template <int r, int c>
matrix<c, r> matrix_transpose(matrix<r, c> A);


template <int r, int c>
matrix<r, c> matrix_inverse(matrix<r, c> A);

template<int r, int c>
matrix<r, c> identity_matrix();


template<int r, int c>
matrix<r, c> matrix_create(double arr[r*c]);


