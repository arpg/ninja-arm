#pragma once
#include "main.h"
#include "fixed.h"
namespace Andromeda
{
  class Matrix
  {
	public:
    static void add( fixed *A, fixed *B, fixed *C, int m, int n );
    static void subtract(fixed *A, fixed *B, fixed *C, int m, int n );
    static void multiply( float *A, float *B, float *C, int m, int n, int p );
    static void multiply( fixed *A, fixed *B, fixed *C, int m, int n, int p );
    static void transpose( fixed *A, fixed *B, int m, int n );
    static void transpose( float *A, float *B, int m, int n );
    static void initialize( fixed *A, int n, int m );
    static void initialize( float *A, int n, int m );
	static void identity( fixed *I, int size, fixed diagonalValue = 1.0 );
    static void identity( float *I, int size, float diagonalValue = 1.0 );
    static void multiplyVector( fixed *A, fixed *b, fixed *v, int m, int n );
    static void multiplyVector( float *A, float *b, float *c, int m, int n );
    static void luDecomposition( fixed *A, fixed *L, fixed*U, int n );
    static void getColumn( fixed *A, fixed *v, int columnNo, int m, int n );
    static void setColumn( fixed *v, fixed *A, int c, int m, int n );
    static void solveUpperTriangular( fixed *A, fixed *b, fixed *x, int n );
    static void solveLowerTriangular( fixed *A, fixed *b, fixed *x, int n );
    static void inverse( fixed *A, fixed *B, int n );
  };
}