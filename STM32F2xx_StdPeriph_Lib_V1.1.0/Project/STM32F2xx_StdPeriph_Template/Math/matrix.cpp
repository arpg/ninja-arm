#pragma once
#include "main.h"
#include "matrix.h"
using namespace Andromeda;


/*----------------------------------------------------------------------------*/
/* This will perform the inverse on matrix A return in matrix B
/* inv(A(n,n)) = B(n,n)
/*----------------------------------------------------------------------------*/
void Matrix::inverse( fixed *A, fixed *B, int n )
{
    fixed  identCol [MAXSIZE] ;
    fixed  ident    [MAXSIZE*MAXSIZE] ;
    fixed  L       [MAXSIZE*MAXSIZE] ;
    fixed  U        [MAXSIZE*MAXSIZE] ;
    fixed  invUcol  [MAXSIZE] ;
    fixed  invLcol  [MAXSIZE] ;
    fixed  invU     [MAXSIZE*MAXSIZE] ;
    fixed  invL     [MAXSIZE*MAXSIZE] ;
    fixed  detA ;
    int     i ;

    //special case if A is a 1 x 1
    if( n == 1 )
    {
        B[0] = fixed(1.0) / A[0] ;
        return ;
    }

    //special case if A is 2 x 2
    if( n == 2 )
    {
        detA = A[0]*A[3] - A[1]*A[2] ;
        B[0] =  A[3] / detA ;
        B[1] = -A[1] / detA ;
        B[2] = -A[2] / detA ;
        B[3] =  A[0] / detA ;
        return ;
    }

    //general case
    identity( ident, n ) ; 
    luDecomposition( A, L, U, n ) ;

    for (i=0; i<n; ++i)
    {
        /* Separates the ith column */
        getColumn( ident, identCol, i, n, n ) ;

        solveLowerTriangular( U, identCol, invUcol, n ) ;
        solveLowerTriangular( L, identCol, invLcol, n ) ;

        /* Place invUcol in ith column of invU */
        setColumn( invUcol, invU, i, n, n ) ;

        /* Place invLcol in ith column of invL */
        setColumn( invLcol, invL, i, n, n ) ;
    }

    /* inv(A) = inv(U)*inv(L) */
    multiply( invU, invL, B, n, n, n ) ;
}

//-----------------------------------------------------------------------------
// add
//
// Adds two matrices together (A and B) and puts the result in matrix C which must
// be initialized to n x m.
// C(n,m) = A(n,m) + B(n,m)
//-----------------------------------------------------------------------------
void Matrix::add( fixed *A, fixed *B, fixed *C, int m, int n )
{
    int  i, j, k;

    k = 0 ;
    for( i=0 ; i<n ; ++i )
    {
        for( j=0 ; j<m ; ++j )
        {
            C[k] = A[k] + B[k];
            k += 1;
        }
    }
}

//-----------------------------------------------------------------------------
// add
//
// Subtracts two matrices from eachother (A and B) and puts the result in matrix C which must
// be initialized to n x m.
// C(n,m) = A(n,m) - B(n,m)
//-----------------------------------------------------------------------------
void Matrix::subtract(fixed *A, fixed *B, fixed *C, int m, int n )
{
    int  i, j, k;

    k = 0 ;
    for( i=0 ; i<n ; ++i )
    {
        for( j=0 ; j<m ; ++j )
        {
            C[k] = A[k] - B[k] ;
            k += 1 ;
        }
    }
}

//-----------------------------------------------------------------------------
// multiply (float)
// 
// Multiplies two matrices together and puts the result in matrix C as per the 
// following equation. Matrix C must be initialized to m x p.
// C(m,p) = A(m,n) * B(n,p)
//-----------------------------------------------------------------------------
void Matrix::multiply( float *A, float *B, float *C, int m, int n, int p )
{
    fixed   s;
    int     i, j, k ;

    for( i=0 ; i<m ; ++i )
    {
        for( j=0 ; j<p ; ++j )
        {
            s = 0.0 ;

            for( k=0 ; k<n ; ++k ) 
                s += A[i*n+k] * B[k*p+j] ;

            C[i*p+j] = s;
        }
    }
}

//-----------------------------------------------------------------------------
// multiply (fixed)
// 
// Multiplies two matrices together and puts the result in matrix C as per the 
// following equation. Matrix C must be initialized to m x p.
// C(m,p) = A(m,n) * B(n,p)
//-----------------------------------------------------------------------------
void Matrix::multiply( fixed *A, fixed *B, fixed *C, int m, int n, int p )
{
    fixed   s ;
    int     i, j, k ;

    for( i=0 ; i<m ; ++i )
    {
        for( j=0 ; j<p ; ++j )
        {
            s = 0.0 ;

            for( k=0 ; k<n ; ++k ) 
                s += A[i*n+k] * B[k*p+j] ;

            C[i*p+j] = s;
        }
    }
}

//-----------------------------------------------------------------------------
// transpose
//
// Will transpose matrix A into matrix B
// B(n,m) = A(m,n)'
//-----------------------------------------------------------------------------
void Matrix::transpose( fixed *A, fixed *B, int m, int n )
{
    int  i, j, k, l ;
    k = 0 ; // initialize B index

    /* For each row of B */
    for( i=0 ; i<n ; ++i )
    {
        l = i ; // initialize A index to upper element of colum i
        /* For each colum of B */
        for ( j=0 ; j<m ; ++j )
        {
            B[k] = A[l];
            k += 1 ; // increment B index
            l += n ; // increment A row #
        }
    }
}

//-----------------------------------------------------------------------------
// transpose
//
// Will transpose matrix A into matrix B
// B(n,m) = A(m,n)'
//-----------------------------------------------------------------------------
void Matrix::transpose( float *A, float *B, int m, int n )
{
    int  i, j, k, l ;

    k = 0 ; // initialize B index

    /* For each row of B */
    for( i=0 ; i<n ; ++i )
    {
        l = i ; // initialize A index to upper element of colum i

        /* For each colum of B */
        for ( j=0 ; j<m ; ++j )
        {
            B[k] = A[l];

            k += 1 ; // increment B index
            l += n ; // increment A row #
        }
    }
}


//-----------------------------------------------------------------------------
// identity
//
// This will initialize the matrix as an identity matrix. It must already be 
// initialized as a square matrix of size x size. The diagonalValue can be used
// to set it to a I(size) x diagonalValue.
//-----------------------------------------------------------------------------
void Matrix::identity( fixed *I, int size, fixed diagonalValue /*= 1.0*/ )
{
    int  i ;
    initialize( I, size, size ) ;
    for( i=0 ; i<size ; ++i ) 
        I[i*size+i] = diagonalValue;
}

//-----------------------------------------------------------------------------
// identity
//
// This will initialize the matrix as an identity matrix. It must already be 
// initialized as a square matrix of size x size. The diagonalValue can be used
// to set it to a I(size) x diagonalValue.
//-----------------------------------------------------------------------------
void Matrix::identity( float *I, int size, float diagonalValue /*= 1.0*/ )
{
    int  i ;
    initialize( I, size, size ) ;
    for( i=0 ; i<size ; ++i ) 
        I[i*size+i] = diagonalValue;
}




//-----------------------------------------------------------------------------
// multiplyVector
//
// This will multiply a matrix by a vector and return the result in vector v
// c(m,1) = A(m,n) * b(n,1) 
//-----------------------------------------------------------------------------
void Matrix::multiplyVector( fixed *A, fixed *b, fixed *v, int m, int n )
{
    fixed  s ;
    int     i, j ;

    for( j=0; j<m; ++j )
    {
        s = 0.0 ;
        for( i=0 ; i<n ; ++i ) 
            s += A[j*n+i] * b[i] ;

        v[j] = s ;
    }
}

//-----------------------------------------------------------------------------
// multiplyVector
//
// This will multiply a matrix by a vector and return the result in vector v
// c(m,1) = A(m,n) * b(n,1) 
//-----------------------------------------------------------------------------
void Matrix::multiplyVector( float *A, float *b, float *c, int m, int n )
{
    fixed  s ;
    int     i, j ;


    for( j=0; j<m; ++j )
    {
        s = 0.0 ;

        for( i=0 ; i<n ; ++i ) 
            s += A[j*n+i] * b[i] ;

        c[j] = s ;
    }
}

/*----------------------------------------------------------------------------*/
/* This will perform LU decomp on matrix A return matrix L and matrix U
/* LU(A(n,n)) => L(n,n) and U(n,n)
/*----------------------------------------------------------------------------*/
void Matrix::luDecomposition( fixed *A, fixed *L, fixed*U, int n )
{
    fixed  Acopy [MAXSIZE*MAXSIZE] ;
    int     i, j, k ;

    //create copy of A matrix
    for( i=0 ; i<n ; ++i ) 
    {
        for(j=0; j<n; ++j) 
            Acopy[i*n+j] = A[i*n+j];
    }

    //decompose copy of A
    for( k=0 ; k<n-1 ; ++k )
    {
        for( i=k+1 ; i<n ; ++i )
        {
            Acopy[i*n+k] = Acopy[i*n+k] / Acopy[k*n+k];

            for( j=k+1 ; j<n ; ++j )
                Acopy[i*n+j] -= Acopy[i*n+k] * Acopy[k*n+j];
        }
    }

    //get the L matrix
    identity( L, n ) ;
    for( j=0 ; j<n-1 ; ++j )
    {
        for ( i=j+1 ; i<n ; ++i ) 
            L[i*n+j] = Acopy[i*n+j];
    }

    //get the U matrix
    initialize( U, n, n ) ;
    for( i=0; i<n; ++i )
    {
        for ( j=i; j<n; ++j ) 
            U[i*n+j] = Acopy[i*n+j];
    }
}

//-----------------------------------------------------------------------------
// getColumn
//
// This function retreives a column from the matrix. The matrix dimensions must
// be m x n. The result is placed into vector v
//-----------------------------------------------------------------------------
void Matrix::getColumn( fixed *A, fixed *v, int columnNo, int m, int n )
{
    int  i ;

    for( i=0; i<m; ++i ) 
        v[i] = A[i*n+columnNo];
}

//-----------------------------------------------------------------------------
// setColumn
//
// This function sets a column in the matrix to the given vector v. The matrix 
// dimensions must be m x n.
//-----------------------------------------------------------------------------
void Matrix::setColumn( fixed *v, fixed *A, int c, int m, int n )
{
    int  i ;

    for( i=0; i<m; ++i ) 
        A[i*n+c] = v[i] ;
}

//-----------------------------------------------------------------------------
// solveUpperTriangular
//
// This will solve A*x = b, where matrix A is upper triangular
// A(n,n)*x(n,1) = b(n,1)
//-----------------------------------------------------------------------------
void Matrix::solveUpperTriangular( fixed *A, fixed *b, fixed *x, int n )
{
    int  i, j, p ;


    p = n + 1 ;

    for( i=1; i<=n; ++i )
    {
        x[p-i-1] = b[p-i-1] ;

        for( j=(p+1-i); j<=n; ++j ) x[p-i-1] -= A[(p-i-1)*n+(j-1)]*x[j-1] ;

        x[p-i-1] = x[p-i-1] / A[(p-i-1)*n+(p-i-1)] ;
    }
}

//-----------------------------------------------------------------------------
// solveUpperTriangular
//
// This will solve A*x = b, where matrix A is lower triangular
// A(n,n)*x(n,1) = b(n,1)
//-----------------------------------------------------------------------------
void Matrix::solveLowerTriangular( fixed *A, fixed *b, fixed *x, int n )
{
    int  i, j ;


    for( i=1; i<=n; ++i )
    {
        x[i-1] = b[i-1] ;

        for(j=1; j<=i-1; ++j ) x[i-1] = x[i-1] - A[(i-1)*n+(j-1)]*x[j-1] ;

        x[i-1] = x[i-1]/A[(i-1)*n+(i-1)] ;
    }
}



//-----------------------------------------------------------------------------
// initialize
//
// This will fill the matrix with zeros. It must already be initialized to n x m.
//-----------------------------------------------------------------------------
void Matrix::initialize( fixed *A, int n, int m )
{
    int  i, j ;
    for(i=0 ; i<n ; ++i ) 
    {
        for( j=0 ; j<m ; ++j ) 
            A[i*m+j] = 0.0;
    }
}

//-----------------------------------------------------------------------------
// initialize
//
// This will fill the matrix with zeros. It must already be initialized to n x m.
//-----------------------------------------------------------------------------
void Matrix::initialize( float *A, int n, int m )
{
    int  i, j ;
    for(i=0 ; i<n ; ++i ) 
    {
        for( j=0 ; j<m ; ++j ) 
            A[i*m+j] = 0.0;
    }
}
