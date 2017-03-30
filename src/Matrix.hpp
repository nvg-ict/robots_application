/*
 * MatrixArray.hpp
 *
 *  Created on: Mar 11, 2017
 *      Author: nico
 */

#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include <array>
#include <iostream>
#include <iomanip>

template<unsigned long ROW, unsigned long COL, typename T = int>
class Matrix
{
public:
	/**
	 * @brief Default constructor to create Matrix
	 */
	Matrix();
	/**
	 * @brief Constructor to create a matrix
	 * @param list 2D initializer list of values that need to be insterted into the matrix
	 */
	Matrix(std::initializer_list<std::initializer_list<T>> list);
	/**
	 * @brief Constructor to create a matrix
	 * @param list 2D initializer list of values that need to be insterted into the matrix
	 */
	Matrix(std::array<std::array<T, ROW>, COL> m);
	/**
	 * @brief Copy constructor
	 * @param m A Matrix
	 */
	Matrix(const Matrix& m);
	/**
	 * Destructor
	 */
	virtual ~Matrix();
	/**
	 * @brief Equal operator. Compares own matrix with input Matrix m
	 * @param m A matrix
	 * @return true if matrices match
	 */
	bool operator==(const Matrix<ROW, COL, T>& m);
	/**
	 * @brief Operator +. Will add a matrix with another and return a new matrix
	 * @param rhs A Matrix
	 * @return The new matrix with both matrices added up
	 */
	Matrix<ROW, COL, T> operator+(const Matrix<ROW, COL, T>& rhs);
	/**
	 * @brief Operator -. Will substract a matrix with another and return a new matrix
	 * @param rhs A Matrix
	 * @return The new matrix with both matrices substracted
	 */
	Matrix<ROW, COL, T> operator-(const Matrix<ROW, COL, T>& rhs);
	/**
	 * @brief Operator =. Will assign one matrix to the other
	 * @param rhs A Matrix
	 * @return A new Matrix based on the old one.
	 */
	Matrix<ROW, COL, T> operator=(const Matrix& rhs);
	/**
	 * @brief Operator *. Will multiply the Matrix with a scalar
	 * @param scalar A scalar of type T2
	 * @return A new matrix multiplied by the scalar
	 */
	template<typename T2>
	Matrix<ROW, COL, T> operator*(const T2 scalar);
	/**
	 * @brief Operator /. Will divide the Matrix with a scalar
	 * @param scalar A scalar of type T2
	 * @return A new matrix divided by the scalar
	 */
	template<typename T2>
	Matrix<ROW, COL, T> operator/(const T2 scalar);
	/**
	 * @brief Will transpose a matrix in a new matrix
	 * @return The new matrix
	 */
	Matrix<COL, ROW, T> trans();
	/**
	 *
	 * @param rhs
	 * @return
	 */
	template<unsigned long COL2>
	Matrix<ROW, COL2, T> operator*(const Matrix<COL, COL2, T>& rhs);
	/**
	 *
	 * @param m
	 * @param mI
	 * @return
	 */
	Matrix<ROW, COL, double> inverse();

	/**
	 *
	 * @param m
	 * @param mI
	 * @return
	 */
	Matrix<ROW, COL, double> identity();

	/**
	 *
	 * @param os
	 * @param m
	 * @return
	 */
	friend std::ostream& operator<<(std::ostream& os, const Matrix& m)
	{
		for (unsigned long i = 0; i < ROW; ++i)
		{
			for (unsigned long j = 0; j < COL; ++j)
			{
				//Manipulate close to zero values
				if(m.matrix.at(i).at(j) != 0 && m.matrix.at(i).at(j) >= -0.0001 && m.matrix.at(i).at(j) <= 0.0001)
				{
					os << 0 << " ";
				}
				else
				{
				os << std::setprecision(5) << m.matrix.at(i).at(j) << " ";
				}
			}
			os << std::endl;
		}
		return os;
	}

	const unsigned long col() const;
	const unsigned long row() const;

	std::array<T, COL>& operator[](unsigned long index);
private:
	/**
	 *
	 * @param m
	 * @param row
	 */
	unsigned short getCorrectRow(Matrix<ROW, COL, T>& m, unsigned long row);
	/**
	 *
	 * @param m
	 * @param mI
	 * @param row
	 */
	void pivotToOne(Matrix<ROW, COL, T>& m, Matrix<ROW, COL, T>& mI,
			unsigned long row);
	/**
	 *
	 * @param m
	 * @param mI
	 * @param row
	 */
	void calculateColsToZero(Matrix<ROW, COL, T>& m, Matrix<ROW, COL, T>& mI,
			unsigned long row);

	std::array<std::array<T, COL>, ROW> matrix;
	//friend any other matrix class so it can access the private parts of the matrix class
	template<unsigned long, unsigned long, typename >
	friend class Matrix;
};

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<ROW, COL, T>::Matrix() :
		matrix()
{
}

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<ROW, COL, T>::Matrix(
		std::initializer_list<std::initializer_list<T>> list) :
		matrix()
{
	if (list.begin()->size() != COL || list.size() != ROW)
	{
		throw std::runtime_error(
				"Matrix is not valid. Check rows and collumns.");
	}
	int i = 0, j = 0;
	for (const auto& l : list)
	{
		for (const auto& v : l)
		{
			matrix[i][j] = v;
			++j;
		}
		j = 0;
		++i;
	}
}

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<ROW, COL, T>::Matrix(std::array<std::array<T, ROW>, COL> m) :
		matrix(m)
{
}

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<ROW, COL, T>::Matrix(const Matrix& m) :
		matrix(m.matrix)
{
}

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<ROW, COL, T>::~Matrix()
{

}

template<unsigned long ROW, unsigned long COL, typename T>
inline bool Matrix<ROW, COL, T>::operator==(const Matrix<ROW, COL, T>& m)
{

	for (unsigned long i = 0; i < ROW; ++i)
	{
		for (unsigned long j = 0; j < COL; ++j)
		{
			if (matrix.at(i).at(j) != m.matrix.at(i).at(j))
				return false;
		}
	}
	return true;
}

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<ROW, COL, T> Matrix<ROW, COL, T>::operator+(
		const Matrix<ROW, COL, T>& rhs)
{
	Matrix<ROW, COL, T> m;
	for (unsigned long i = 0; i < ROW; ++i)
	{
		for (unsigned long j = 0; j < COL; ++j)
		{
			m.matrix.at(i).at(j) = matrix.at(i).at(j) + rhs.matrix.at(i).at(j);
		}
	}
	return m;
}

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<ROW, COL, T> Matrix<ROW, COL, T>::operator-(
		const Matrix<ROW, COL, T>& rhs)
{
	Matrix<ROW, COL, T> m;
	for (unsigned long i = 0; i < ROW; ++i)
	{

		for (unsigned long j = 0; j < COL; ++j)
		{
			m.matrix.at(i).at(j) = matrix.at(i).at(j) - rhs.matrix.at(i).at(j);
		}
	}
	return m;
}

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<ROW, COL, T> Matrix<ROW, COL, T>::operator=(const Matrix& rhs)
{
	if (&rhs != this)
	{
		matrix = rhs.matrix;
	}

	return *this;
}

template<unsigned long ROW, unsigned long COL, typename T>
template<typename T2>
inline Matrix<ROW, COL, T> Matrix<ROW, COL, T>::operator*(const T2 scalar)
{
	Matrix<ROW, COL, T> m;
	for (unsigned long i = 0; i < ROW; ++i)
	{
		for (unsigned long j = 0; j < COL; ++j)
		{
			m.matrix.at(i).at(j) = matrix.at(i).at(j) * scalar;
		}
	}
	return m;
}

template<unsigned long ROW, unsigned long COL, typename T>
template<typename T2>
inline Matrix<ROW, COL, T> Matrix<ROW, COL, T>::operator/(const T2 scalar)
{
	Matrix<ROW, COL, T> m;
	for (unsigned long i = 0; i < ROW; ++i)
	{
		for (unsigned long j = 0; j < COL; ++j)
		{
			m.matrix.at(i).at(j) = matrix.at(i).at(j) / scalar;
		}
	}
	return m;
}

template<unsigned long ROW, unsigned long COL, typename T>
inline Matrix<COL, ROW, T> Matrix<ROW, COL, T>::trans()
{
	Matrix<COL, ROW, T> m;
	for (unsigned long i = 0; i < COL; ++i)
	{
		for (unsigned long j = 0; j < ROW; ++j)
		{
			m.matrix.at(i).at(j) = matrix.at(j).at(i);
		}
	}
	return m;
}

template<unsigned long ROW, unsigned long COL, typename T>
template<unsigned long COL2>
inline Matrix<ROW, COL2, T> Matrix<ROW, COL, T>::operator*(
		const Matrix<COL, COL2, T>& rhs)
{
	Matrix<ROW, COL2, T> m;

	for (unsigned long i = 0; i < ROW; ++i)
	{
		for (unsigned long j = 0; j < COL2; ++j)
		{
			T value = 0;
			for (unsigned long k = 0; k < COL; ++k)
			{
				value += matrix.at(i).at(k) * rhs.matrix.at(k).at(j);
			}
			m.matrix.at(i).at(j) = value;
		}
	}
	return m;
}

template<unsigned long ROW, unsigned long COL, typename T>
inline const unsigned long Matrix<ROW, COL, T>::col() const
{
	return COL;
}

template<unsigned long ROW, unsigned long COL, typename T>
inline const unsigned long Matrix<ROW, COL, T>::row() const
{
	return ROW;
}

template<unsigned long ROW, unsigned long COL, typename T>
inline std::array<T, COL>& Matrix<ROW, COL, T>::operator[](unsigned long index)
{
	return matrix.at(index);
}

template<unsigned long ROW, unsigned long COL, typename T>
Matrix<ROW, COL, double> Matrix<ROW, COL, T>::inverse()
{
	Matrix<ROW, COL, T> m(*this);
	//Create the Identity matrix
	Matrix<ROW, COL, T> mI(identity());
	for (unsigned short row = 0; row < m.row(); ++row)
	{
		//1 Vind de correcte bovenste rij
		unsigned long index = getCorrectRow(m, row);

		//2 swappen
		m[index].swap(m[row]);
		mI[index].swap(mI[row]);

		//3 maak bovenste rij 1 * * vorm
		pivotToOne(m, mI, row);

		//4 Maak vd overige rijen eerste element 0
		calculateColsToZero(m, mI, row);
	}

	return mI;
}

template<unsigned long ROW, unsigned long COL, typename T>
Matrix<ROW, COL, double> Matrix<ROW, COL, T>::identity()
{
	//Matrix<ROW,COL,double> m;
	Matrix<ROW, COL, T> mI;
	for (unsigned short i = 0; i < mI.row(); ++i)
	{
		mI[i].fill(0);
		mI[i][i] = 1;
	}
	return mI;
}

template<unsigned long ROW, unsigned long COL, typename T>
unsigned short Matrix<ROW, COL, T>::getCorrectRow(Matrix<ROW, COL, T>& m,
		unsigned long row)
{
	double highestValue = 0;
	unsigned short indexOfRow;
	for (unsigned short i = row; i < m.row(); ++i)
	{
		if (m[i][row] == 1 || m[i][row] == -1)
		{
			indexOfRow = i;
			break;
		}
		if (highestValue < fabs(m[i][row]))
		{
			highestValue = fabs(m[i][row]);
			indexOfRow = i;
		}
	}
	return indexOfRow;
}

template<unsigned long ROW, unsigned long COL, typename T>
void Matrix<ROW, COL, T>::pivotToOne(Matrix<ROW, COL, T>& m,
		Matrix<ROW, COL, T>& mI, unsigned long row)
{
	double a = m[row][row];
	for (unsigned short i = 0; i < m.col(); ++i)
	{
		m[row][i] = m[row][i] / a;
		mI[row][i] = mI[row][i] / a;
	}
}

template<unsigned long ROW, unsigned long COL, typename T>
void Matrix<ROW, COL, T>::calculateColsToZero(Matrix<ROW, COL, T>& m,
		Matrix<ROW, COL, T>& mI, unsigned long row)
{
	for (unsigned short i = 0; i < m.row(); ++i)
	{
		double a = m[i][row];
		for (unsigned short j = 0; j < m.col(); ++j)
		{
			if (i != row)
			{
				std::array<T, COL> temp = m[row];
				std::array<T, COL> tempI = mI[row];
				m[i][j] = m[i][j] - temp[j] * a;
				mI[i][j] = mI[i][j] - tempI[j] * a;
			}
		}
	}

	std::cout<<mI<<std::endl;

//	for (unsigned short i = 0; i < m.row(); ++i)
//	{
//		for (unsigned short j = 0; j < m.col(); ++j)
//		{
//			std::cout << i << "," << j << ";" << mI[i][j] << std::endl;
//			if (mI[i][j] >= -0.00001 && mI[i][j] <= 0.00001 && mI[i][j] != 0)
//			{
//				std::cout << "Adjust 0 position" << std::endl;
//				mI[i][j] = 0;
//			}
//		}
//	}
}

#endif /* MATRIX_HPP_ */
