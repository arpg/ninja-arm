#pragma once
#include "main.h"
#include "fixed.h"
#define DEG_TO_RAD 0.0174532925
namespace Andromeda
{
  
  class Math
  {
	public:
    static void quat2euler( fixed *Q, fixed *roll, fixed *pitch, fixed *yaw);
    static void quat2euler( float *Q, float *roll, float *pitch, float *yaw);
    static void euler2quat( float phi, float theta, float psi, float *Q );
  };
}