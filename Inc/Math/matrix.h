

#ifndef _LINALG_H_
#define _LINALG_H_

#include <algorithm>
#include <array>
#include <vector>
#include <stdio.h>
#include <stdarg.h>

using std::make_shared;
using std::shared_ptr;
using std::vector;


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

Matrix operator*(const double &a, const Matrix &b);


#endif /* _LINALG_H_ */