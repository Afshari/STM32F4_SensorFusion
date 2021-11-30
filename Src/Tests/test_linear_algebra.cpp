
#include "Tests/test_linear_algebra.h"

TestLinearAlgebra::TestLinearAlgebra() : TestBase() {

}


void TestLinearAlgebra::init() {

	testAdd();
	testSub();
	testScale_1();
	testScale_2();
	testMul_1();
	testMul_2();
	testMul_3();
	testMul_4();
	testTran_1();
	testTran_2();
	testTran_3();

	printReport("Linear Algebra");
}


void TestLinearAlgebra::testAdd() {
											
	Matrix ref_C{ 3, 3, { 5,  10, 6,
												8,  10, 13,
												13, 4,  5 } };
	Matrix A{ 3, 3, { 1, 2, 3,
										4, 5, 6,
										7, 2, 2 } };
	Matrix B{ 3, 3, { 4, 8, 3,
										4, 5, 7,
										6, 2, 3 } };
	Matrix C = A + B;

	checkEqual( C, ref_C, 0.1, " " );										
}


void TestLinearAlgebra::testSub() {
	
	Matrix ref_C{ 3, 3, {   -3,  -6, 	 0,
													 0,  	0,  -1,
													 1, 	0,  -1 } };
	Matrix A{ 3, 3, { 1, 2, 3,
										4, 5, 6,
										7, 2, 2 } };
	Matrix B{ 3, 3, { 4, 8, 3,
										4, 5, 7,
										6, 2, 3 } };
	Matrix C = A - B;
											
	checkEqual( C, ref_C, 0.1, " ");
}


void TestLinearAlgebra::testScale_1() {

	Matrix ref_C{ 3, 3, {   2.5,  	5, 	 		7.5,
													10,  		12.5,  	15,
													17.5, 	5,  		5 } };
	Matrix A{ 3, 3, { 1, 2, 3,
										4, 5, 6,
										7, 2, 2 } };
	Matrix C = A * 2.5;
	
	checkEqual( C, ref_C, 0.01, " " );
}

void TestLinearAlgebra::testScale_2() {
		
	Matrix ref_C{ 3, 3, {   2.5,  	5, 	 		7.5,
													10,  		12.5,  	15,
													17.5, 	5,  		5 } };
	Matrix A{ 3, 3, { 1, 2, 3,
										4, 5, 6,
										7, 2, 2 } };
	Matrix C = 2.5 * A;
	
	checkEqual( C, ref_C, 0.01, " " );
}


void TestLinearAlgebra::testMul_1() {
	
	Matrix ref_C{ 3, 3, { 30, 24, 26,
												72, 69, 65,
												48, 70, 41 } };
	Matrix A{ 3, 3, { 1, 2, 3,
										4, 5, 6,
										7, 2, 2 } };
	Matrix B{ 3, 3, { 4, 8, 3,
										4, 5, 7,
										6, 2, 3 } };
	Matrix C = A * B;
	
	checkEqual( C, ref_C, 0.01, " " );
}

void TestLinearAlgebra::testMul_2() {
		
	Matrix ref_C{ 3, 1, { 30,
												72,
												48 } };
	Matrix A{ 3, 3, { 1, 2, 3,
										4, 5, 6,
										7, 2, 2 } };
	Matrix B{ 3, 1, { 4,
										4,
										6, } };
	Matrix C = A * B;
	
	checkEqual( C, ref_C, 0.01, " " );
}


void TestLinearAlgebra::testMul_3() {
	
	Matrix ref_C{ 3, 3, { 4,  8,  12,
												4,  8,  12,
												6,  12, 18 } };
	Matrix A{ 1, 3, { 1, 2, 3 } };
	Matrix B{ 3, 1, { 4,
										4,
										6, } };
	Matrix C = B * A;
	
	checkEqual( C, ref_C, 0.01, " " );
}

void TestLinearAlgebra::testMul_4() {
	
	Matrix ref_C{ 1, 1, { 30 } };
	Matrix A{ 1, 3, { 1, 2, 3 } };
	Matrix B{ 3, 1, { 4,
										4,
										6, } };
	Matrix C = A * B;
	
	checkEqual( C, ref_C, 0.01, " " );
}

void TestLinearAlgebra::testTran_1() {
		
	Matrix ref_Transpose{ 3, 3, { 1, 4, 7,
																2, 5, 8,
																3, 6, 2 } };
	Matrix A{3, 3, {  1, 2, 3,
										4, 5, 6,
										7, 8, 2 } };
	checkEqual( A.transpose(), ref_Transpose, 0.01, " " );
}


void TestLinearAlgebra::testTran_2() {
		
	Matrix ref_Transpose{ 2, 3, { 1, 4, 7,
																2, 5, 6 } };
	Matrix A{ 3, 2, { 1, 2,
										4, 5,
										7, 6, } };
	checkEqual( A.transpose(), ref_Transpose, 0.01, " " );
}

void TestLinearAlgebra::testTran_3() {
		
	Matrix ref_Transpose{ 1, 3, { 1, 4, 7 } };
	Matrix A{ 3, 1, { 1,
										4,
										7, } };
	checkEqual( A.transpose(), ref_Transpose, 0.01, " " );
}






