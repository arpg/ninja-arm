
#include "imuKalman.h"

using namespace Andromeda;


//-----------------------------------------------------------------------------
// ImuKalman
//
// Constructor function. This function needs the magnetic inclination and declination
// values in degrees for the given location. This can be obtained from:
// http://www.ngdc.noaa.gov/geomagmodels/IGRFWMM.jsp
//-----------------------------------------------------------------------------
ImuKalman::ImuKalman()
{
	_prevMag = 0;
	float reference[3];

	_magReference[0] = MAGREF_X;// reference[0];
	_magReference[1] = MAGREF_Y;//reference[1];
	_magReference[2] = MAGREF_Z;//reference[2];

	Vector::normalize(_magReference);
	_isInitialised = false;
}

//-----------------------------------------------------------------------------
// initialize
//
// initializes the state quaternions given the initial accelerometer and magnetometer
// readings. Uses the cross products from these vectors to calculate the 3 vectors
// of the DCM and then converts them to quaternions to put them in the state matrix.
//-----------------------------------------------------------------------------
void ImuKalman::initialize(float xAccel, float yAccel, float zAccel, float xMagn, float yMagn, float zMagn)
{
	float accels[3] = {xAccel, yAccel, zAccel};
	float mags[3] = {xMagn, yMagn, zMagn};
			
	//inititalize the euler angles array
	_eulerAngles[0] = 0;
	_eulerAngles[1] = 0;
	_eulerAngles[2] = 0;
			
	//zero the state quaternion
	for( int i = 0; i < STATE_COUNT ; i++ )
		_state[i] = 0;
	
	_state[4] = DEFAULT_P_BIAS;
	_state[5] = DEFAULT_Q_BIAS;
	_state[6] = DEFAULT_R_BIAS;



	//zero the jacobian matrix of the state equation
	for( int i = 0 ; i < STATE_COUNT_P2 ; i++ )
		_A[i] = 0;


	//and set the values for the bias states to 1
	_A[32] = 1;
	_A[40] = 1;
	_A[48] = 1;

	//zero the jacobian matrix of the measurement equation
	for( int i = 0 ; i < STATEXMEASUREMENT_COUNT ; i++ )
		_H[i] = 0;

	//initialize the covariance matrix
	for( int i = 0 ; i < STATE_COUNT_P2 ; i++ )
		_P[i] = 0;

	int i = 0;
	for( i=0 ; i<STATE_COUNT_P2 ; i++ ) _P[i]   = 0.0 ;
	for( i=0 ; i< 4 ; i++ ) _P[i*(STATE_COUNT+1)] = INI_QUAT_VAR ;
	for( i=4 ; i< 7 ; i++ ) _P[i*(STATE_COUNT+1)] = INI_BIAS_VAR ;


	//initialize the process noise covariance matrix
	for( int i = 0 ; i < STATE_COUNT_P2 ; i++ )
		_Q[i] = 0;

	for( i=0 ; i<STATE_COUNT_P2 ; i++ ) _Q[i]   = 0.0 ;
	for( i=0 ; i< 4 ; i++ ) _Q[i*(STATE_COUNT+1)] = PRO_QUAT_VAR ;
	for( i=4 ; i< 7 ; i++ ) _Q[i*(STATE_COUNT+1)] = PRO_BIAS_VAR ;

	//initialize the measurement noise covariance matrix
	for( int i = 0 ; i < MEASUREMENT_COUNT_P2 ; i++ )
	{
		_R_Accel[i] = 0;
		_R_Mag[i] = 0;
	}
	
	for( int i = 0 ; i < GPS_MEASUREMENT_COUNT_P2 ; i++ )
		_R_Gps[i] = 0;
	
	for( i = 0 ; i< GPS_MEASUREMENT_COUNT ; i++ ) 
		_R_Gps[i *(GPS_MEASUREMENT_COUNT+1)] = GPS_NORM_VAR;
	
	for( i = 0 ; i< MEASUREMENT_COUNT ; i++ ) 
		_R_Accel[i *(MEASUREMENT_COUNT+1)] = ACC_NORM_VAR;
	
	for( i = 0 ; i< MEASUREMENT_COUNT ; i++ ) 
		_R_Mag[i *(MEASUREMENT_COUNT+1)] = MAG_NORM_VAR;

	Vector::normalize(mags);
	Vector::normalize(accels);

	////now claculate the body zW vector
	float zWb[3] = {-accels[0],-accels[1],-accels[2]};

	float crossProduct[3];
	Vector::crossProduct(zWb, mags, crossProduct);
	
	float length = Vector::length(crossProduct);
	float yWb[3] = {crossProduct[0]/length,crossProduct[1]/length,crossProduct[2]/length};
	//float yWb[3] = {0,1,0};

	//finally calculate the body xW vector
	float xWb[3];
	Vector::crossProduct(yWb,zWb,xWb);

	float q[4];
	q[0] = sqrt(1+xWb[0]+yWb[1]+zWb[2])/2;
	q[1] = (zWb[1]-yWb[2])/(4*q[0]);
	q[2] = (xWb[2]-zWb[0])/(4*q[0]);
	q[3] = (yWb[0]-xWb[1])/(4*q[0]);

	//normalize quaternion
	float l = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );

	q[0] /= l;
	q[1] /= l;
	q[2] /= l;
	q[3] /= l;

	//and then initialize the GN states
	_state[0] = q[0];
	_state[1] = q[1];
	_state[2] = q[2];
	_state[3] = q[3];
	
	float roll = 0;
	float pitch = 0;
	float yaw = 0;

	Math::quat2euler(_state,&_eulerAngles[0], &_eulerAngles[1],&_eulerAngles[2]);
	_roll =  _eulerAngles[0];
	_pitch = _eulerAngles[1];
	_yaw = _eulerAngles[2];

	//set the initialised flag
    _isInitialised = true;
}

//-----------------------------------------------------------------------------
// EKF measurement update. Has the option of using Accelerometers and GPS
// heading coordinates
//-----------------------------------------------------------------------------
void ImuKalman::measurementUpdate(fixed xAccel, fixed yAccel, fixed zAccel,fixed xGpsComponent, fixed yGpsComponent, bool useAccel /*= true*/, bool useGps /*= true*/)
{
  //first iterate gauss newton to get its state
  //iterateGaussNewton(xAccel, yAccel, zAccel,xMag, yMag,  zMag);

  //now using this we can perform our measurement
  //normalize quaternion
   normalizeState();

  float accels[3] = {xAccel, yAccel, zAccel};
	
  Vector::normalize(accels);

  //create y matrix
  fixed y[3] = {accels[0], accels[1], accels[2]};  //the reason these are negative is that the jacobian has a - sign in front of it 
  fixed dq[4];

  float y0bE[6];
  fixed ym[3] = {_magReference[0] , _magReference[1], _magReference[2]};

  fixed q0 = _state[0];
  fixed q1 = _state[1];
  fixed q2 = _state[2];
  fixed q3 = _state[3];


  //and now we can calculate the error 
  fixed e[6];

  
  
  if( useAccel )
  {
	  //calculate the predicted accelerometer and magnetometer readings
	  y0bE[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*y[0] +               (q1*q2 - q3*q0)*2*y[1] +               (q1*q3 + q2*q0)*2*y[2];
	  y0bE[1] =               (q1*q2 + q3*q0)*2*y[0] + (q0*q0 + q2*q2 - q1*q1 - q3*q3)*y[1] +               (q2*q3 - q1*q0)*2*y[2];
	  y0bE[2] =               (q1*q3 - q2*q0)*2*y[0] +               (q2*q3 + q1*q0)*2*y[1] + (q0*q0 + q3*q3 - q2*q2 - q1*q1)*y[2];
	  
	  e[0] = -y0bE[0];
	  e[1] = -y0bE[1];
	  e[2] = -y0bE[2]-1;
		  
	   //1st column - qy0m/q0q2 * y
	  _H[0] = (q0*y[0] + -q3*y[1] + q2*y[2])*2;
	  _H[7] = (q3*y[0] + q0*y[1] + -q1*y[2])*2;
	  _H[14] =  (-q2*y[0] + q1*y[1] + q0*y[2])*2;
	
	
	  //2nd column - qy0m/q0q2 * y
	  _H[1] = (q1*y[0] + q2*y[1] + q3*y[2])*2;
	  _H[8] = (q2*y[0] + -q1*y[1] + -q0*y[2])*2;
	  _H[15] = (q3*y[0] + q0*y[1] + -q1*y[2])*2;
	
	
	  //3rd column - qy0m/q0q3 * y
	  _H[2] = (-q2*y[0] + q1*y[1] + q0*y[2])*2;
	  _H[9] = (q1*y[0] + q2*y[1] + q3*y[2])*2;
	  _H[16] =(-q0*y[0] + q3*y[1] + -q2*y[2])*2;
	
	
	  //4th column - qy0m/q0q0 * y
	  _H[3] = (-q3*y[0] + -q0*y[1] + q1*y[2])*2;
	  _H[10] = (q0*y[0] + -q3*y[1] + q2*y[2])*2;
	  _H[17] = (q1*y[0] + q2*y[1] + q3*y[2])*2;
	  
	  //and now correct using the EKF
		kalmanCorrect(_P,_state,_H,_R_Accel,e,_K,STATE_COUNT,MEASUREMENT_COUNT);
	  
	  //normalize quaternion
		normalizeState();
  }
  
  if( useGps )
  {
	    
	  y0bE[3] = (q1*q2 + q0*q3)*-2; //this must be flipped to match the yGpsComponent polarity
	  y0bE[4] = (q2*q2 + q3*q3)*-2 + 1;
		
		float x = xGpsComponent;
		float y = yGpsComponent;
		
	  e[3] = yGpsComponent  - y0bE[3];
	  e[4] = xGpsComponent  - y0bE[4];
		
	  if( e[3] > 0.02 )
			e[3] = 0.02;
	  else if( e[3] < 0.02 )
			e[3]  = -0.02;
		
	 if( e[4] > 0.02 )
			e[4] = 0.02;
	  else if( e[4] < 0.02 )
			e[4]  = -0.02;


    _H[0] =	q3*-2;
	_H[7] = 0;
  
    _H[1] = q2*-2;
	_H[8] = 0;
  
    _H[2] = q1 * -2;
	_H[9] = q2 * -4;
  
    _H[3] = q0 * -2;
	_H[10] = q3 * -4;
  
  //and now correct using the EKF
	kalmanCorrect(_P,_state,_H,_R_Gps,e+3,_K,STATE_COUNT,GPS_MEASUREMENT_COUNT);
  
  //normalize quaternion
	normalizeState();
  
  }
}

//-----------------------------------------------------------------------------
// measurementUpdate
//
// EKF measurement update. Must be called when new measurements are available.
// This function is called externally and supplied with accelerometer and magnetometer
// data. The units do not matter as they are normalized, however they must be the
// same between accelerometers and magnetometers
//-----------------------------------------------------------------------------
void ImuKalman::measurementUpdate(fixed xAccel, fixed yAccel, fixed zAccel,fixed xMag, fixed yMag, fixed zMag, bool useAccel /* = true*/, bool useMag /*= true*/)
{
  //first iterate gauss newton to get its state
  //iterateGaussNewton(xAccel, yAccel, zAccel,xMag, yMag,  zMag);

  //now using this we can perform our measurement
  //normalize quaternion
   normalizeState();

  float accels[3] = {xAccel, yAccel, zAccel};
  float mags[3] = {xMag, yMag, zMag};
	
	
  Vector::normalize(mags);
  Vector::normalize(accels);

  //create y matrix
  fixed y[6] = {accels[0], accels[1], accels[2], mags[0], mags[1], mags[2]};  //the reason these are negative is that the jacobian has a - sign in front of it 
  fixed dq[4];

  fixed y0bE[6];
  fixed ym[3] = {_magReference[0] , _magReference[1], _magReference[2]};

  fixed q0 = _state[0];
  fixed q1 = _state[1];
  fixed q2 = _state[2];
  fixed q3 = _state[3];


  //and now we can calculate the error 
  fixed e[6];

  
  
  if( useAccel )
  {
	  //calculate the predicted accelerometer and magnetometer readings
	  y0bE[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*y[0] +               (q1*q2 - q3*q0)*2*y[1] +               (q1*q3 + q2*q0)*2*y[2];
	  y0bE[1] =               (q1*q2 + q3*q0)*2*y[0] + (q0*q0 + q2*q2 - q1*q1 - q3*q3)*y[1] +               (q2*q3 - q1*q0)*2*y[2];
	  y0bE[2] =               (q1*q3 - q2*q0)*2*y[0] +               (q2*q3 + q1*q0)*2*y[1] + (q0*q0 + q3*q3 - q2*q2 - q1*q1)*y[2];
	  
	  e[0] = -y0bE[0];
	  e[1] = -y0bE[1];
	  e[2] = -y0bE[2]-1;
		  
	  _H[0] = (q0*y[0] + -q3*y[1] + q2*y[2])*2;
	  _H[7] = (q3*y[0] + q0*y[1] + -q1*y[2])*2;
	  _H[14] =  (-q2*y[0] + q1*y[1] + q0*y[2])*2;
	
	
	  //2nq0 column - q0m/q0q2 * y
	  _H[1] = (q1*y[0] + q2*y[1] + q3*y[2])*2;
	  _H[8] = (q2*y[0] + -q1*y[1] + -q0*y[2])*2;
	  _H[15] = (q3*y[0] + q0*y[1] + -q1*y[2])*2;
	
	
	  //3rq0 column - q0m/q0q3 * y
	  _H[2] = (-q2*y[0] + q1*y[1] + q0*y[2])*2;
	  _H[9] = (q1*y[0] + q2*y[1] + q3*y[2])*2;
	  _H[16] =(-q0*y[0] + q3*y[1] + -q2*y[2])*2;
	
	
	  //4th column - q0m/q0q0 * y
	  _H[3] = (-q3*y[0] + -q0*y[1] + q1*y[2])*2;
	  _H[10] = (q0*y[0] + -q3*y[1] + q2*y[2])*2;
	  _H[17] = (q1*y[0] + q2*y[1] + q3*y[2])*2;
	  
	  //and now correct using the EKF
		kalmanCorrect(_P,_state,_H,_R_Accel,e,_K,STATE_COUNT,MEASUREMENT_COUNT);
	  
	  //normalize quaternion
		normalizeState();
  }
  
  if( useMag )
  {
	    
  y0bE[3] = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*y[3] +               (q1*q2 - q3*q0)*2*y[4] +               (q1*q3 + q2*q0)*2*y[5];
  y0bE[4] =               (q1*q2 + q3*q0)*2*y[3] + (q0*q0 + q2*q2 - q1*q1 - q3*q3)*y[4] +               (q2*q3 - q1*q0)*2*y[5];
  y0bE[5] =               (q1*q3 - q2*q0)*2*y[3] +               (q2*q3 + q1*q0)*2*y[4] + (q0*q0 + q3*q3 - q2*q2 - q1*q1)*y[5];
		
		
  e[3] = _magReference[0]  - y0bE[3];
  e[4] = _magReference[1]  - y0bE[4];
  e[5] = _magReference[2]  - y0bE[5];


    _H[0] =                                     (q0*y[3] + -q3*y[4] + q2*y[5])*2;
  _H[7] =                                     (q3*y[3] + q0*y[4] + -q1*y[5])*2;
  _H[14] =                                     (-q2*y[3] + q1*y[4] + q0*y[5])*2;
  
    _H[1] =                                     (q1*y[3] + q2*y[4] + q3*y[5])*2;
  _H[8] =                                     (q2*y[3] + -q1*y[4] + -q0*y[5])*2;
  _H[15] =                                     (q3*y[3] + q0*y[4] + -q1*y[5])*2;
  
    _H[2] =                                     (-q2*y[3] + q1*y[4] + q0*y[5])*2;
  _H[9] =                                     (q1*y[3] + q2*y[4] + q3*y[5])*2;
  _H[16] =                                     (-q0*y[3] + q3*y[4] + -q2*y[5])*2;
  
    _H[3] =                                     (-q3*y[3] + -q0*y[4] + q1*y[5])*2;
  _H[10] =                                     (q0*y[3] + -q3*y[4] + q2*y[5])*2;
  _H[17] =                                     (q1*y[3] + q2*y[4] + q3*y[5])*2;
  
  //and now correct using the EKF
	kalmanCorrect(_P,_state,_H,_R_Mag,e+3,_K,STATE_COUNT,MEASUREMENT_COUNT);
  
  //normalize quaternion
	normalizeState();
  
  }

}

//-----------------------------------------------------------------------------
// timeUpdate
//
// EKF time update. Must be called on every iteration. The gyro values must be
// given in radians per second and dT must be given in seconds. 
//-----------------------------------------------------------------------------
void ImuKalman::timeUpdate(fixed xGyro_raw, fixed yGyro_raw, fixed zGyro_raw, fixed dT)
{
	//time must not be 0
	if( dT == 0 )
		return;
	
	//if time is larger than the maximum timestep, then exit
	if( dT > MAX_TIME_STEP )
		return;
	
	//temporary variable which will be used in calculations
	fixed dT_2 = dT/2;      

	//correct gyros for bias by subtracting the calculated bias
	fixed xGyro = xGyro_raw - _state[xGyro_Bias];
	fixed yGyro = yGyro_raw - _state[yGyro_Bias];
	fixed zGyro = zGyro_raw - _state[zGyro_Bias];

	//get the quaternion values used in the calculation of the jacobian
	fixed  Q1dT_2 = _state[Q1] *dT_2;
	fixed  Q2dT_2 = _state[Q2] *dT_2;
	fixed  Q3dT_2 = _state[Q3] *dT_2;
	fixed  Q4dT_2 = _state[Q4] *dT_2;

	//get the gyroscope values used in the calculation of the jacobian
	fixed  xGyrodT_2 = xGyro * dT_2;
	fixed  yGyrodT_2 = yGyro * dT_2;
	fixed  zGyrodT_2 = zGyro * dT_2;

	
	//populate the jacobian. first row: partial derivatives of the equation for Q1
	_A[0] = 1.0;
	_A[1] = -xGyrodT_2;
	_A[2] = -yGyrodT_2;
	_A[3] = -zGyrodT_2;
	_A[4] = Q2dT_2;
	_A[5] = Q3dT_2;
	_A[6] = Q4dT_2;

	//2nd row: partial derivatives of the equation for Q2
	_A[7] = xGyrodT_2;
	_A[8] = 1.0;
	_A[9] = zGyrodT_2;
	_A[10] = -yGyrodT_2;
	_A[11] = -Q1dT_2;
	_A[12] = Q4dT_2;
	_A[13] = -Q3dT_2;

	//3rd row: partial derivatives of the equation for Q3
	_A[14] = yGyrodT_2;
	_A[15] = -zGyrodT_2;
	_A[16] = 1.0;
	_A[17] = xGyrodT_2;
	_A[18] = -Q4dT_2;
	_A[19] = -Q1dT_2;
	_A[20] = Q2dT_2;

	//4th row: partial derivatives of the equation for Q4
	_A[21] = zGyrodT_2;
	_A[22] = yGyrodT_2;
	_A[23] = -xGyrodT_2;
	_A[24] = 1.0;
	_A[25] = Q3dT_2;
	_A[26] = -Q2dT_2;
	_A[27] = -Q1dT_2;

	//Rows 5-7 have been pre-initialized in the constructor and do not need to be
	//re initialized. These are to do with the biases and do not change

	//do the Kalman prediction update
	kalmanPredict(_state,_P,_A,_Q);

	//normalize quaternion
	normalizeState();


}

bool ImuKalman::normalizeState()
{
	fixed magnitude = 0;
	for( int i=0 ; i<4 ; i++)
		magnitude += _state[i]*_state[i];
	magnitude = sqrt((float)magnitude);
	if( magnitude != 0 )
	{
	  for( int i=0 ; i<4 ; i++)
		_state[i] /= magnitude;
	}else
	  return false;
}

//-----------------------------------------------------------------------------
// kalmanPredict
//
// Performs the EKF predict stage calculation given the state vector, covariance 
// matrix (P), state transition matrix (A) and state error variance matrix (Q).
// Note: only states 1-4 which are the quaternions are updated. This is because
// states 5-7 which are the biases are assumed to be constant in the time-update
// phase
//-----------------------------------------------------------------------------
void ImuKalman::kalmanPredict(fixed *state, fixed *P, fixed *A , fixed *Q)
{

  //store the old quaternions
  
  fixed Q_old[4];
  for( int i=0 ; i<4 ; i++)
	  Q_old[i] = state[i] ;

  //now propagate the new states
  state[Q1] =  A[0]*Q_old[Q1]  + A[1]*Q_old[Q2]  + A[2]*Q_old[Q3]  + A[3]*Q_old[Q4];
  state[Q2] =  A[7]*Q_old[Q1]  + A[8]*Q_old[Q2]  + A[9]*Q_old[Q3]  + A[10]*Q_old[Q4];
  state[Q3] =  A[14]*Q_old[Q1] + A[15]*Q_old[Q2] + A[16]*Q_old[Q3] + A[17]*Q_old[Q4];
  state[Q4] =  A[21]*Q_old[Q1] + A[22]*Q_old[Q2] + A[23]*Q_old[Q3] + A[24]*Q_old[Q4];
  

  //now propagate the covariance
  Matrix::transpose( A, _temp1, 7, 7 ) ;			// A'
  Matrix::multiply( A, P, _temp2, 7, 7, 7 ) ;		// A.P
  Matrix::multiply( _temp2, _temp1, _temp3, 7, 7, 7 ) ;		// A.P.A'
  Matrix::add( _temp3, Q, P, 7 , 7 ) ;			// A.P.A' + Q
}

//-----------------------------------------------------------------------------
// kalmanCorrect
// 
// Performs the correction stage of the EKF given all the KF matrices as follows:
//	P(n,n)		Covariance matrix
//	state(n,1)      State Vector
//	C(m,n)		Measurement matrix; m=# of measurements, n=# of states
//	R(m,m)		Measurement weight matrix 
//	E(m,1)		Error vector = Xmeasurement(m,1) - Xestimate(m,1)
//	K(n,m)		Kalman Gain matrix
//-----------------------------------------------------------------------------
void ImuKalman::kalmanCorrect( fixed *P, fixed *state, fixed *C, fixed *R, fixed *E, fixed *K, int n, int m )
{
   
	// F = C*P*C' + R 
	Matrix::multiply( C, P, _temp1, m, n, n ) ;
	Matrix::transpose( C, _temp2, m, n ) ;
	Matrix::multiply( _temp1, _temp2, _temp3, m, n, m ) ;
	Matrix::add( _temp3, R, _temp3, m, m ) ;

	// K = P*C'*inv(F) 
	Matrix::multiply( P, _temp2, _temp1, n, n, m ) ;
	Matrix::inverse( _temp3, _temp2, m ) ;
	Matrix::multiply( _temp1, _temp2, K, n, m, m ) ;

	// x = x + K*(ys - yp)
	Matrix::multiplyVector( K, E, _tempV, n, m ) ;
	Vector::add( state, _tempV, state, n ) ;

	// P = P - K*C*P 
	Matrix::multiply( K, C, _temp1, n, m, n ) ;
	Matrix::multiply( _temp1, P, _temp2, n, n, n ) ;
	Matrix::subtract( P, _temp2, P, n, n ) ;
}



