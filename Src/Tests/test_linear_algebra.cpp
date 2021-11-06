
#include "Tests/test_linear_algebra.h"

TestLinearAlgebra::TestLinearAlgebra() : TestBase() {

}


void TestLinearAlgebra::init() {

	testAdd();
	testSub();
	testScale();
	testMul_1();
	testMul_2();
	testMul_3();
	testMul_4();

	printReport("Linear Algebra");
}


void TestLinearAlgebra::testAdd() {

	vector<double> ref_C( { 5,  10, 6,
													8,  10, 13,
													13, 4,  5 } );
	
	vector<double> A( { 1, 2, 3,
											4, 5, 6,
											7, 2, 2 } );
	
	vector<double> B( { 4, 8, 3,
											4, 5, 7,
											6, 2, 3 } );		

	vector<double> C;

	add( A, B, C, 3, 3 );
											
	checkEqual( ref_C, C, 0.1, " ");
											
}


void TestLinearAlgebra::testSub() {

	vector<double> ref_C( {   -3,  -6, 	 0,
														 0,  	0,  -1,
														 1, 	0,  -1 } );
	
	vector<double> A( { 1, 2, 3,
											4, 5, 6,
											7, 2, 2 } );
	
	vector<double> B( { 4, 8, 3,
											4, 5, 7,
											6, 2, 3 } );		

	vector<double> C;

	sub( A, B, C, 3, 3 );
											
	checkEqual( ref_C, C, 0.1, " ");
}


void TestLinearAlgebra::testScale() {
	
	vector<double> ref_A( {   2.5,  	5, 	 		7.5,
														10,  		12.5,  	15,
														17.5, 	5,  		5 } );
	
	vector<double> A( { 1, 2, 3,
											4, 5, 6,
											7, 2, 2 } );	

	scale( A, 2.5, 3, 3 );
											
	checkEqual( ref_A, A, 0.1, " ");
}


void TestLinearAlgebra::testMul_1() {

	vector<double> ref_C( { 30, 24, 26,
													72, 69, 65,
													48, 70, 41 } );
	
	vector<double> A( { 1, 2, 3,
											4, 5, 6,
											7, 2, 2 } );
	
	vector<double> B( { 4, 8, 3,
											4, 5, 7,
											6, 2, 3 } );		

	vector<double> C;

	mul( A, B, C, 3, 3, 3 );
											
	checkEqual( ref_C, C, 0.1, " ");
}

void TestLinearAlgebra::testMul_2() {

	vector<double> ref_C( { 30,
													72,
													48 } );
	
	vector<double> A( { 1, 2, 3,
											4, 5, 6,
											7, 2, 2 } );
	
	vector<double> B( { 4,
											4,
											6, } );		

	vector<double> C;

	mul( A, B, C, 3, 3, 1 );
											
	checkEqual( ref_C, C, 0.1, " ");
}


void TestLinearAlgebra::testMul_3() {

	vector<double> ref_C( { 4,  8,  12,
													4,  8,  12,
													6,  12, 18 } );
	
	vector<double> A( { 1, 2, 3 } );
	
	vector<double> B( { 4,
											4,
											6, } );			

	vector<double> C;

	mul( B, A, C, 3, 1, 3 );
											
	checkEqual( ref_C, C, 0.1, " ");
}

void TestLinearAlgebra::testMul_4() {

	vector<double> ref_C( { 30 } );
	
	vector<double> A( { 1, 2, 3 } );
	
	vector<double> B( { 4,
											4,
											6, } );		

	vector<double> C;

	mul( A, B, C, 1, 3, 1 );
											
	checkEqual( ref_C, C, 0.1, " ");
}


