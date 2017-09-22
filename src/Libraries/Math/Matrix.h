/*
 * Matrix.h
 *
 *  Created on: 31 Mar 2015
 *      Author: David
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include <cstddef>		// for size_t

// Base class for matrices, allows us to write functions that work with any size matrix
template<class T> class MathMatrix
{
public:
	virtual size_t rows() const = 0;
	virtual size_t cols() const = 0;
	virtual T& operator() (size_t r, size_t c) = 0;
	virtual const T& operator() (size_t r, size_t c) const = 0;
	virtual ~MathMatrix() { }		// to keep Eclipse code analysis happy
};

// Fixed size matrix class
template<class T, size_t ROWS, size_t COLS> class FixedMatrix : public MathMatrix<T>
{
public:
	size_t rows() const override { return ROWS; }
	size_t cols() const override { return COLS; }

	// Indexing operator, non-const version
	T& operator() (size_t r, size_t c) override
	pre(r < ROWS; c < COLS)
	{
		return data[r * COLS + c];
	}

	// Indexing operator, const version
	const T& operator() (size_t r, size_t c) const override
	pre(r < ROWS; c < COLS)
	{
		return data[r * COLS + c];
	}

	void SwapRows(size_t i, size_t j, size_t numCols = COLS)
	pre(i < ROWS; j < ROWS)
	;

	void GaussJordan(T *solution, size_t numRows)
	pre(numRows <= ROWS; numRows + 1 <= COLS)
	;

	// Return a pointer to a specified row, non-const version
	T* GetRow(size_t r)
	pre(r < ROWS)
	{
		return data + (r * COLS);
	}

	// Return a pointer to a specified row, const version
	const T* GetRow(size_t r) const
	pre(r < ROWS)
	{
		return data + (r * COLS);
	}

private:
	T data[ROWS * COLS];
};

// Swap 2 rows of a matrix
template<class T, size_t ROWS, size_t COLS> inline void FixedMatrix<T, ROWS, COLS>::SwapRows(size_t i, size_t j, size_t numCols)
{
	if (i != j)
	{
		for (size_t k = i; k < numCols; ++k)
		{
			T temp = (*this)(i, k);
			(*this)(i, k) = (*this)(j, k);
			(*this)(j, k) = temp;
		}
	}
}

// Perform Gauss-Jordan elimination on a N x (N+1) matrix.
// Returns a pointer to the solution vector.
template<class T, size_t ROWS, size_t COLS> void FixedMatrix<T, ROWS, COLS>::GaussJordan(T *solution, size_t numRows)
{
	for (size_t i = 0; i < numRows; ++i)
	{
		// Swap the rows around for stable Gauss-Jordan elimination
		float vmax = fabsf((*this)(i, i));
		for (size_t j = i + 1; j < numRows; ++j)
		{
			const float rmax = fabsf((*this)(j, i));
			if (rmax > vmax)
			{
				SwapRows(i, j);
				vmax = rmax;
			}
		}

		// Use row i to eliminate the ith element from previous and subsequent rows
		T v = (*this)(i, i);
		for (size_t j = 0; j < i; ++j)
		{
			T factor = (*this)(j, i)/v;
			(*this)(j, i) = 0.0;
			for (size_t k = i + 1; k <= numRows; ++k)
			{
				(*this)(j, k) -= (*this)(i, k) * factor;
			}
		}

		for (size_t j = i + 1; j < numRows; ++j)
		{
			T factor = (*this)(j, i)/v;
			(*this)(j, i) = 0.0;
			for (size_t k = i + 1; k <= numRows; ++k)
			{
				(*this)(j, k) -= (*this)(i, k) * factor;
			}
		}
	}

	for (size_t i = 0; i < numRows; ++i)
	{
		solution[i] = (*this)(i, numRows) / (*this)(i, i);
	}
}

#endif /* MATRIX_H_ */
