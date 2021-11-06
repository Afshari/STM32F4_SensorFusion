

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


void add( const vector<double> &A, const vector<double> &B, vector<double> &C, int row, int column );
void sub( const vector<double> &A, const vector<double> &B, vector<double> &C, int row, int column );
void mul( const vector<double> &A, const vector<double> &B, vector<double> &C, int row_a, int column_a, int column_b );
void scale( vector<double> &A, double scalar, int row, int column);

#endif /* _LINALG_H_ */