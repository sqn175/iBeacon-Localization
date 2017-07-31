/*
 *The Matrix library is licensed under a permissive 3 - clause BSD license.Contributions must be made under the same license.

Copyright(c) 2015, Matrix Development Team.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met :

*Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of matrix nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Modified by: Shi Qin
// modify the Matrix library to use the core functions

#pragma once
#include <cstring>

namespace BIP
{

template<typename Type, size_t M, size_t N>
class Matrix
{
public:
	Type _data[M][N];

	// Constructors
	Matrix() : _data() {}
	Matrix(const Type data_[][N]) : _data()
	{
		memcpy(_data, data_, sizeof(_data));
	}
	Matrix(const Type *data_) : _data()
	{
		memcpy(_data, data_, sizeof(_data));
	}
	Matrix(const Matrix &other) : _data()
	{
		memcpy(_data, other._data, sizeof(_data));
	}

	// Accessors/Assignment
	inline Type operator()(size_t i, size_t j) const
	{
		return _data[i][j];
	}

	inline Type &operator()(size_t i, size_t j)
	{
		return _data[i][j];
	}
	Matrix<Type, M, N> & operator=(const Matrix<Type, M, N> &other)
	{
		if (this != &other) {
			memcpy(_data, other._data, sizeof(_data));
		}
		return (*this);
	}

	// operations
	template<size_t P>
	Matrix<Type, M, P> operator*(const Matrix<Type, N, P> &other) const
	{
		const Matrix<Type, M, N> &self = *this;
		Matrix<Type, M, P> res;
		res.setZero();

		for (size_t i = 0; i < M; i++) {
			for (size_t k = 0; k < P; k++) {
				for (size_t j = 0; j < N; j++) {
					res(i, k) += self(i, j) * other(j, k);
				}
			}
		}

		return res;
	}

	Matrix<Type, M, N> operator+(const Matrix<Type, M, N> &other) const
	{
		Matrix<Type, M, N> res;
		const Matrix<Type, M, N> &self = *this;

		for (size_t i = 0; i < M; i++) {
			for (size_t j = 0; j < N; j++) {
				res(i, j) = self(i, j) + other(i, j);
			}
		}

		return res;
	}

	Matrix<Type, M, N> operator-(const Matrix<Type, M, N> &other) const
	{
		Matrix<Type, M, N> res;
		const Matrix<Type, M, N> &self = *this;

		for (size_t i = 0; i < M; i++) {
			for (size_t j = 0; j < N; j++) {
				res(i, j) = self(i, j) - other(i, j);
			}
		}

		return res;
	}

	Matrix<Type, N, M> transpose() const
	{
		Matrix<Type, N, M> res;
		const Matrix<Type, M, N> &self = *this;

		for (size_t i = 0; i < M; i++) {
			for (size_t j = 0; j < N; j++) {
				res(j, i) = self(i, j);
			}
		}

		return res;
	}

	void setZero()
	{
		memset(_data, 0, sizeof(_data));
	}
	void setIdentity()
	{
		setZero();
		Matrix<Type, M, N> &self = *this;

		for (size_t i = 0; i < M && i < N; i++) {
			self(i, i) = 1;
		}
	}

	inline void identity()
	{
		setIdentity();
	}

	inline void swapRows(size_t a, size_t b)
	{
		if (a == b) {
			return;
		}

		Matrix<Type, M, N> &self = *this;

		for (size_t j = 0; j < N; j++) {
			Type tmp = self(a, j);
			self(a, j) = self(b, j);
			self(b, j) = tmp;
		}
	}

	inline void swapCols(size_t a, size_t b)
	{
		if (a == b) {
			return;
		}

		Matrix<Type, M, N> &self = *this;

		for (size_t i = 0; i < M; i++) {
			Type tmp = self(i, a);
			self(i, a) = self(i, b);
			self(i, b) = tmp;
		}
	}
};

template<typename Type, size_t M>
class SquareMatrix : public Matrix<Type, M, M>
{
public:
	SquareMatrix() :
		Matrix<Type, M, M>()
	{
	}

	SquareMatrix(const Type *data_) :
		Matrix<Type, M, M>(data_)
	{
	}

	SquareMatrix(const Matrix<Type, M, M> &other) :
		Matrix<Type, M, M>(other)
	{
	}

	// inverse alias
	inline SquareMatrix<Type, M> I() const
	{
		SquareMatrix<Type, M> i;
		if (inv(*this, i)) {
			return i;
		}
		else {
			i.setZero();
			return i;
		}
	}

};


template<typename Type>
bool is_finite(Type x) {
#if defined (__PX4_NUTTX)
	return PX4_ISFINITE(x);
#elif defined (__PX4_QURT)
	return __builtin_isfinite(x);
#else
	return std::isfinite(x);
#endif
}


/**
* inverse based on LU factorization with partial pivotting
*/
template<typename Type, size_t M>
bool inv(const SquareMatrix<Type, M> & A, SquareMatrix<Type, M> & inv)
{
	SquareMatrix<Type, M> L;
	L.setIdentity();
	SquareMatrix<Type, M> U = A;
	SquareMatrix<Type, M> P;
	P.setIdentity();

	//printf("A:\n"); A.print();

	// for all diagonal elements
	for (size_t n = 0; n < M; n++) {

		// if diagonal is zero, swap with row below
		if (fabs(static_cast<float>(U(n, n))) < 1e-8f) {
			//printf("trying pivot for row %d\n",n);
			for (size_t i = n + 1; i < M; i++) {

				//printf("\ttrying row %d\n",i);
				if (fabs(static_cast<float>(U(i, n))) > 1e-8f) {
					//printf("swapped %d\n",i);
					U.swapRows(i, n);
					P.swapRows(i, n);
					L.swapRows(i, n);
					L.swapCols(i, n);
					break;
				}
			}
		}

#ifdef MATRIX_ASSERT
		//printf("A:\n"); A.print();
		//printf("U:\n"); U.print();
		//printf("P:\n"); P.print();
		//fflush(stdout);
		//ASSERT(fabs(U(n, n)) > 1e-8f);
#endif

		// failsafe, return zero matrix
		if (fabs(static_cast<float>(U(n, n))) < 1e-8f) {
			return false;
		}

		// for all rows below diagonal
		for (size_t i = (n + 1); i < M; i++) {
			L(i, n) = U(i, n) / U(n, n);

			// add i-th row and n-th row
			// multiplied by: -a(i,n)/a(n,n)
			for (size_t k = n; k < M; k++) {
				U(i, k) -= L(i, n) * U(n, k);
			}
		}
	}

	//printf("L:\n"); L.print();
	//printf("U:\n"); U.print();

	// solve LY=P*I for Y by forward subst
	//SquareMatrix<Type, M> Y = P;

	// for all columns of Y
	for (size_t c = 0; c < M; c++) {
		// for all rows of L
		for (size_t i = 0; i < M; i++) {
			// for all columns of L
			for (size_t j = 0; j < i; j++) {
				// for all existing y
				// subtract the component they
				// contribute to the solution
				P(i, c) -= L(i, j) * P(j, c);
			}

			// divide by the factor
			// on current
			// term to be solved
			// Y(i,c) /= L(i,i);
			// but L(i,i) = 1.0
		}
	}

	//printf("Y:\n"); Y.print();

	// solve Ux=y for x by back subst
	//SquareMatrix<Type, M> X = Y;

	// for all columns of X
	for (size_t c = 0; c < M; c++) {
		// for all rows of U
		for (size_t k = 0; k < M; k++) {
			// have to go in reverse order
			size_t i = M - 1 - k;

			// for all columns of U
			for (size_t j = i + 1; j < M; j++) {
				// for all existing x
				// subtract the component they
				// contribute to the solution
				P(i, c) -= U(i, j) * P(j, c);
			}

			// divide by the factor
			// on current
			// term to be solved
			//
			// we know that U(i, i) != 0 from above
			P(i, c) /= U(i, i);
		}
	}

	//check sanity of results
	for (size_t i = 0; i < M; i++) {
		for (size_t j = 0; j < M; j++) {
			if (!is_finite(P(i, j))) {
				return false;
			}
		}
	}
	//printf("X:\n"); X.print();
	inv = P;
	return true;
}

/**
* inverse based on LU factorization with partial pivotting
*/
template<typename Type, size_t M>
SquareMatrix<Type, M> inv(const SquareMatrix<Type, M> & A)
{
	SquareMatrix<Type, M> i;
	if (inv(A, i)) {
		return i;
	}
	else {
		i.setZero();
		return i;
	}
}

} // namespace BIP