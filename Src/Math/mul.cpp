
#include "linalg.h"


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