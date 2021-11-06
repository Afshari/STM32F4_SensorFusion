

#include "linalg.h"

void sub( const vector<double> &A, const vector<double> &B, vector<double> &C, int row, int column ) {
	
	C.resize(row * column);
	
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			
				int idx = (i * column) + j;
				C.at(idx) = A.at(idx) - B.at(idx);
		}
	}
}