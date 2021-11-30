

#include "matrix.h"

Matrix::Matrix( int row, int column ) : m_row{row}, m_column{column} {
	m_values.resize( row * column );
}

Matrix::Matrix( int row, int column, const vector<double> &values ) : m_row{row}, m_column{column}, m_values{values} {
}

Matrix::Matrix( const Matrix &other ) {
	this->m_row 		= other.m_row;
	this->m_column 	= other.m_column;
	this->m_values 	= other.m_values;
}

int Matrix::rowSize() const {
	return m_row;
}

int Matrix::columnSize() const {
	return m_column;
}

double Matrix::at(int i, int j) const {
		int idx = (i * this->m_column) + j;
		return m_values.at(idx);
}

void Matrix::set(double value, int i, int j) {
	int idx = (i * this->m_column) + j;
	m_values.at(idx) = value;
}

Matrix Matrix::operator+(const Matrix &other) {
	
	Matrix result{ this->m_row, this->m_column };
	
	for (int i = 0; i < this->m_row; i++) {
		for (int j = 0; j < this->m_column; j++) {
			int idx = (i * this->m_column) + j;
			result.m_values.at(idx) = this->m_values.at(idx) + other.m_values.at(idx);
		}
	}	
	return result;
}

Matrix Matrix::operator-(const Matrix &other) {
	
	Matrix result{ this->m_row, this->m_column };
	
	for (int i = 0; i < this->m_row; i++) {
		for (int j = 0; j < this->m_column; j++) {
			int idx = (i * this->m_column) + j;
			result.m_values.at(idx) = this->m_values.at(idx) - other.m_values.at(idx);
		}
	}
	return result;
}

Matrix Matrix::operator*(const Matrix &other) {
	
	Matrix result{ this->m_row, other.m_column };
	
	for (int i = 0; i < this->m_row; i++) {
			for (int j = 0; j < other.m_column; j++) {
				
					result.set(0, i, j);
					for (int k = 0; k < this->m_column; k++) {
						result.set( result.at(i, j) + ( this->at(i, k) * other.at(k, j) ), i, j );
					}
			}
	}	
	return result;
}

Matrix Matrix::operator*(const double &other) {
	
	Matrix result{ this->m_row, this->m_column };
	for (int i = 0; i < this->m_row; i++) {
		for (int j = 0; j < this->m_column; j++) {
			
				result.set( this->at(i, j) * other, i, j );
		}
	}
	return result;
}

Matrix operator*(const double &a, const Matrix &b) {

	Matrix result{ b.rowSize(), b.columnSize() };
	for (int i = 0; i < b.rowSize(); i++) {
		for (int j = 0; j < b.columnSize(); j++) {
			
				result.set( b.at(i, j) * a, i, j );
		}
	}
	return result;	
}

Matrix Matrix::transpose() {
	
	Matrix result{ this->m_column, this->m_row };

	for (int i = 0; i < this->m_row; i++) {
		for (int j = 0; j < this->m_column; j++) {
			result.set( this->at(i, j), j, i );
		}
	}
	return result;
}

void tran( vector<double> &A, int row, int column) {

	vector<double> B( row * column );

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			int idx_a = (i * column) + j;
			int idx_b = (j * row) + i;
			B.at(idx_b) = A.at(idx_a);
		}
	}

	std::copy(std::begin(B), std::end(B), std::begin(A));
}


