

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <algorithm>
#include <array>
#include <vector>
#include <memory>
#include <stdio.h>
#include <stdarg.h>

#include "app.h"

using std::make_shared;
using std::shared_ptr;
using std::array;
using std::vector;

#ifdef USE_CMSIS_DSP

#include "arm_math.h"

class Matrix {
	
	friend class TestMatrix;
	friend Matrix operator*(const double &a, const Matrix &b);
	
private:
	int m_row;
	int m_column;
	vector<float32_t> m_values;
	arm_matrix_instance_f32 m_matrix;

public:
	Matrix( int row, int column );
	Matrix( int row, int column, const vector<double> &v );
	Matrix( const arm_matrix_instance_f32 &matrix );
	Matrix( const Matrix &other );
	Matrix operator+(const Matrix &other);
	Matrix operator-(const Matrix &other);
	Matrix operator*(const Matrix &other);
	Matrix operator*(const double &other);
	Matrix transpose();

	int 		rowSize() const;
	int 		columnSize() const;
	double 	at(int i, int j) const;
	void   	set(double value, int i, int j);
};

#else

class Matrix {
	
private:
	int m_row;
	int m_column;
	vector<double> m_values;

public:
	Matrix( int row, int column );
	Matrix( int row, int column, const vector<double> &v );
	Matrix( const Matrix &other );
	Matrix operator+(const Matrix &other);
	Matrix operator-(const Matrix &other);
	Matrix operator*(const Matrix &other);
	Matrix operator*(const double &other);
	Matrix transpose();

	int 		rowSize() const;
	int 		columnSize() const;
	double 	at(int i, int j) const;
	void   	set(double value, int i, int j);
};

#endif

Matrix operator*(const double &a, const Matrix &b);


#endif /* _MATRIX_H_ */