#pragma once
#include "fixed.h"
namespace Andromeda
{
  class Vector
  {
	public:
    static double length(float *v);
    static fixed length(fixed *v);
    static void normalize(fixed *v);
    static void normalize(float *v);
    static void crossProduct(fixed  *v1, fixed  *v2, fixed  *result );
    static void crossProduct(float  *v1, float  *v2, float  *result );
    static void add( fixed *a, fixed *b, fixed *c, int n );
    static void subtract( fixed *a, fixed *b, fixed *c, int n );
	
	static fixed dotProduct2(fixed *v1, fixed *v2);
    static float dotProduct2(float *v1, float *v2);
	static fixed length2(fixed *v);
    static float length2(float *v);
	static void add2(fixed *v1, fixed *v2, fixed *result);
    static void add2(float *v1, float *v2, float *result);
	static void subtract2(fixed *v1, fixed *v2, fixed *result);
    static void subtract2(float *v1, float *v2, float *result);
    static void scale2(fixed *v1, fixed scale, fixed *result);
    static void scale2(float *v1, float scale, float *result);
    static fixed crossProduct2(fixed  *v1, fixed  *v2);
    static float crossProduct2(float  *v1, float  *v2);
  };

}