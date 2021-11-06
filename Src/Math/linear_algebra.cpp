

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