#pragma once
#include "mathroutines.h"
using namespace Andromeda;

//-----------------------------------------------------------------------------
// quat2euler
//
// This function converts an input quaternion vector into pitch, roll and yaw
// values. The quaternion vector must have length of 4
//-----------------------------------------------------------------------------
void Math::quat2euler( fixed *Q, fixed *roll, fixed *pitch, fixed *yaw)
{
	*roll   = fixed::atan2x((Q[2]*Q[3] + Q[0]*Q[1]) * 2,
		(Q[1]*Q[1] + Q[2]*Q[2])*-2 + 1) ;

	*pitch = -fixed::asin((Q[0]*Q[2] - Q[1]*Q[3])*2);

	*yaw   = -fixed::atan2x((Q[1]*Q[2] + Q[0]*Q[3])*2,
		(Q[2]*Q[2] + Q[3]*Q[3])*-2 + 1 ) ;

}
void Math::quat2euler( float *Q, float *roll, float *pitch, float *yaw)
{
	*roll   = fixed::atan2x((Q[2]*Q[3] + Q[0]*Q[1]) * 2,
		(Q[1]*Q[1] + Q[2]*Q[2])*-2 + 1) ;

	*pitch = -fixed::asin((Q[0]*Q[2] - Q[1]*Q[3])*2);

	*yaw   = fixed::atan2x((Q[1]*Q[2] + Q[0]*Q[3])*2,
		(Q[2]*Q[2] + Q[3]*Q[3])*-2 + 1 ) ;
}
//-----------------------------------------------------------------------------
// quat2euler
//
// This function converts the given pitch, roll and yaw values into a quaternion.
// the given Q pointer must point to an array with length of 4
//-----------------------------------------------------------------------------
void Math::euler2quat( float phi, float theta, float psi, float *Q )
{
	float  phi_2, theta_2, psi_2 ;
	float  sphi, cphi, stheta, ctheta, spsi, cpsi ;
	float  cphi_ctheta , sphi_stheta , cphi_stheta , sphi_ctheta ;


	phi_2 = phi / 2.0 ; theta_2 = theta / 2.0 ; psi_2 = psi / 2.0 ;

	sphi   = sin( phi_2 ) ;    cphi   = cos( phi_2 ) ;
	stheta = sin( theta_2 ) ;  ctheta = cos( theta_2 ) ;
	spsi   = sin( psi_2 ) ;    cpsi   = cos( psi_2 ) ;

	cphi_ctheta = cphi * ctheta ; sphi_stheta = sphi * stheta ;
	cphi_stheta = cphi * stheta ; sphi_ctheta = sphi * ctheta ;

	Q[0] =  cphi_ctheta * cpsi + sphi_stheta * spsi ;
	Q[1] = -cphi_stheta * spsi + sphi_ctheta * cpsi ;
	Q[2] =  cphi_stheta * cpsi + sphi_ctheta * spsi ;
	Q[3] =  cphi_ctheta * spsi - sphi_stheta * cpsi ;
}
