
#include "matrix.h"

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


#ifdef USE_CMSIS_DSP

Matrix::Matrix( int row, int column ) : m_row{row}, m_column{column} {
	m_values.resize( row * column );
}

Matrix::Matrix( int row, int column, const vector<double> &values ) : m_row{row}, m_column{column} {
	m_values.resize( m_row * m_column );
	std::copy(values.begin(), values.end(), m_values.begin());
	arm_mat_init_f32(&m_matrix, m_row, m_column, m_values.data());
}

Matrix::Matrix( const arm_matrix_instance_f32 &matrix ) {
	
	m_row = matrix.numRows;
	m_column = matrix.numCols;
	m_values.resize( m_row * m_column );
	std::copy(matrix.pData, matrix.pData + (m_row * m_column), m_values.begin());
	arm_mat_init_f32(&m_matrix, m_row, m_column, m_values.data());
}

Matrix::Matrix( const Matrix &other ) {
	
	m_row = other.m_row;
	m_column = other.m_column;
	m_values.resize( m_row * m_column );
	std::copy(other.m_values.begin(), other.m_values.end(), m_values.begin());
	arm_mat_init_f32(&m_matrix, m_row, m_column, m_values.data());
}

Matrix Matrix::operator+(const Matrix &other) {
	
	Matrix result{ this->m_row, this->m_column };
	
	vector<float32_t> result_vec( result.m_row * result.m_column, 0.0 );
	// arm_matrix_instance_f32 result_matrix;
	arm_mat_init_f32(&result.m_matrix, result.m_row, result.m_column, result_vec.data());
	arm_mat_add_f32(&this->m_matrix, &other.m_matrix, &result.m_matrix);
	std::copy(result.m_matrix.pData, result.m_matrix.pData + (result.m_row * result.m_column), result.m_values.begin());
	
	return result;
}

Matrix Matrix::operator-(const Matrix &other) {
	
	Matrix result{ this->m_row, this->m_column };
	
	vector<float32_t> result_vec( result.m_row * result.m_column, 0.0 );
	// arm_matrix_instance_f32 result_matrix;
	arm_mat_init_f32(&result.m_matrix, result.m_row, result.m_column, result_vec.data());
	arm_mat_sub_f32(&this->m_matrix, &other.m_matrix, &result.m_matrix);
	std::copy(result.m_matrix.pData, result.m_matrix.pData + (result.m_row * result.m_column), result.m_values.begin());
	
	return result;
}

Matrix Matrix::operator*(const Matrix &other) {
	
	Matrix result{ this->m_row, other.m_column };
	
	vector<float32_t> result_vec( result.m_row * result.m_column, 0.0 );
	// arm_matrix_instance_f32 result_matrix;
	arm_mat_init_f32(&result.m_matrix, result.m_row, result.m_column, result_vec.data());
	arm_mat_mult_f32(&this->m_matrix, &other.m_matrix, &result.m_matrix);
	std::copy(result.m_matrix.pData, result.m_matrix.pData + (result.m_row * result.m_column), result.m_values.begin());
	
	return result;
}

Matrix Matrix::operator*(const double &other) {
	
	Matrix result{ this->m_row, this->m_column };

	vector<float32_t> result_vec( result.m_row * result.m_column, 0.0 );
	// arm_matrix_instance_f32 result_matrix;
	arm_mat_init_f32(&result.m_matrix, result.m_row, result.m_column, result_vec.data());
	arm_mat_scale_f32(&this->m_matrix, other, &result.m_matrix);
	std::copy(result.m_matrix.pData, result.m_matrix.pData + (result.m_row * result.m_column), result.m_values.begin());
	
	return result;
}

Matrix operator*(const double &a, const Matrix &b) {

	Matrix result{ b.rowSize(), b.columnSize() };
	
	vector<float32_t> result_vec( result.m_row * result.m_column, 0.0 );
	// arm_matrix_instance_f32 result_matrix;
	arm_mat_init_f32(&result.m_matrix, result.m_row, result.m_column, result_vec.data());
	arm_mat_scale_f32(&b.m_matrix, a, &result.m_matrix);
	std::copy(result.m_matrix.pData, result.m_matrix.pData + (result.m_row * result.m_column), result.m_values.begin());
	
	return result;	
}

Matrix Matrix::transpose() {
	
	Matrix result{ this->m_column, this->m_row };

	vector<float32_t> result_vec( result.m_row * result.m_column, 0.0 );
	arm_mat_init_f32(&result.m_matrix, result.m_row, result.m_column, result_vec.data());
	arm_mat_trans_f32(&this->m_matrix, &result.m_matrix);
	std::copy(result.m_matrix.pData, result.m_matrix.pData + (result.m_row * result.m_column), result.m_values.begin());
	
	return result;
}

#else

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

#endif
