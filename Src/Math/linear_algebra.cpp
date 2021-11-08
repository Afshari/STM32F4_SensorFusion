

#include "linalg.h"

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

void add( const vector<double> &A, const vector<double> &B, vector<double> &C, int row, int column ) {
	
	C.resize(row * column);
	
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {

			int idx = (i * column) + j;
			C.at(idx) = A.at(idx) + B.at(idx);

		}
	}
}


void mul( const vector<double> &A, const vector<double> &B, vector<double> &C, 
					int row_a, int column_a, int column_b ) {
		
		C.resize( row_a * column_b );
  
    for (int i = 0; i < row_a; i++) {
			
        for (int j = 0; j < column_b; j++) {
					
						int idx = (i * column_b) + j;
            C.at(idx) = 0;
            for (int k = 0; k < column_a; k++) {
								int idx_a = (i * column_a) + k;
								int idx_b = (k * column_b) + j;
                C.at(idx) += A.at(idx_a) * B.at(idx_b);
            }
        }
    }
}
					
void sub( const vector<double> &A, const vector<double> &B, vector<double> &C, int row, int column ) {
	
	C.resize(row * column);
	
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			
				int idx = (i * column) + j;
				C.at(idx) = A.at(idx) - B.at(idx);
		}
	}
}

void scale( vector<double> &A, double scalar, int row, int column) {

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			
				int idx = (i * column) + j;
				A.at(idx) = A.at(idx) * scalar;
		}
	}
}