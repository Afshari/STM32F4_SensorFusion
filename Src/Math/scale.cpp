

#include "linalg.h"

void scale( vector<double> &A, double scalar, int row, int column) {

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			
				int idx = (i * column) + j;
				A.at(idx) = A.at(idx) * scalar;
		}
	}
}