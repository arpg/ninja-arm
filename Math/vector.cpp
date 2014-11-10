#pragma once
#include "vector.h"
using namespace Andromeda;

//-----------------------------------------------------------------------------
// normalize
//
// normalizes the vector v by calculating its length and dividing the elements by
// the length. The vector must have a length of 3
//-----------------------------------------------------------------------------
void Vector::normalize(fixed *v)
{
	fixed length = Vector::length(v);
	if( length != 0 )
	{
		v[0] /= length;
		v[1] /= length;
		v[2] /= length;
	}
}
void Vector::normalize(float *v)
{
	float length = Vector::length(v);
	if( length != 0 )
	{
		v[0] /= length;
		v[1] /= length;
		v[2] /= length;
	}
}

//-----------------------------------------------------------------------------
// dotProduct2
//
// Optimised vector dot product for 2D vectors
//-----------------------------------------------------------------------------
fixed Vector::dotProduct2(fixed *v1, fixed *v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1];
}
float Vector::dotProduct2(float *v1, float *v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1];
}

//-----------------------------------------------------------------------------
// length2
//
// Optimised vector length function for 2D vectors
//-----------------------------------------------------------------------------
fixed Vector::length2(fixed *v)
{
	return sqrt(v[0]*v[0] + v[1]*v[1]);
}
float Vector::length2(float *v)
{
	return sqrt(v[0]*v[0] + v[1]*v[1]);
}

//-----------------------------------------------------------------------------
// add2
//
// Optimised vector add function for 2D vectors
//-----------------------------------------------------------------------------
void Vector::add2(fixed *v1, fixed *v2, fixed *result)
{
	result[0] = v1[0] + v2[0];
	result[1] = v1[1] + v2[1];
}
void Vector::add2(float *v1, float *v2, float *result)
{
	result[0] = v1[0] + v2[0];
	result[1] = v1[1] + v2[1];
}

//-----------------------------------------------------------------------------
// add2
//
// Optimised vector add function for 2D vectors
//-----------------------------------------------------------------------------
void Vector::subtract2(fixed *v1, fixed *v2, fixed *result)
{
	result[0] = v1[0] - v2[0];
	result[1] = v1[1] - v2[1];
}
void Vector::subtract2(float *v1, float *v2, float *result)
{
	result[0] = v1[0] - v2[0];
	result[1] = v1[1] - v2[1];
}

//-----------------------------------------------------------------------------
// Scales a vector by a scalar
// @v1 - the input vector to scale
// @scale - the scalar value to scale by
// @result - the resulting vector is placed here
//-----------------------------------------------------------------------------
void Vector::scale2(fixed *v1, fixed scale, fixed *result)
{
	result[0] = v1[0]*scale;
    result[1] = v1[1]*scale;
}
void Vector::scale2(float *v1, float scale, float *result)
{
	result[0] = v1[0]*scale;
    result[1] = v1[1]*scale;
}

//-----------------------------------------------------------------------------
// Calculates the cross product of 2 2D vectors and returns it
// @v1 - The first input vector
// @v2 - The second input vector 
//-----------------------------------------------------------------------------
fixed Vector::crossProduct2(fixed  *v1, fixed  *v2)
{
	return (v1[0]*v2[1])-(v1[1]*v2[0]);
}
float Vector::crossProduct2(float *v1, float *v2)
{
	return (v1[0]*v2[1])-(v1[1]*v2[0]);
}

//-----------------------------------------------------------------------------
// crossProduct
//
// Computes the cross product of v1 and v2 and puts the result in the result
// variable. Both v1 and v2 must have length of 3
//-----------------------------------------------------------------------------
void Vector::crossProduct(fixed  *v1, fixed  *v2, fixed  *result )
{
	result[0] = v1[1]*v2[2] - v1[2]*v2[1];
	result[1] = v1[2]*v2[0] - v1[0]*v2[2];
	result[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

void Vector::crossProduct(float  *v1, float  *v2, float  *result )
{
	result[0] = v1[1]*v2[2] - v1[2]*v2[1];
	result[1] = v1[2]*v2[0] - v1[0]*v2[2];
	result[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

//-----------------------------------------------------------------------------
// add
//
// This will add two vectors of length n together and put the result in vector c.
// c(n,1) + a(n,1) + b(n,1)
//-----------------------------------------------------------------------------
void Vector::add( fixed *a, fixed *b, fixed *c, int n )
{
	int  i ;
	for( i=0 ; i<n ; ++i ) 
		c[i] = a[i] + b[i] ;
}

void Vector::subtract( fixed *a, fixed *b, fixed *c, int n )
{
	int  i ;
	for( i=0 ; i<n ; ++i ) 
		c[i] = a[i] - b[i] ;
}

//-----------------------------------------------------------------------------
// length
//
// calculates the length of the vector and returns it.
//-----------------------------------------------------------------------------
double Vector::length(float *v)
{
	return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

fixed Vector::length(fixed *v)
{
	return fixed::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}