#pragma once
#include "main.h"
#include "mathroutines.h"
#include "matrix.h"
#include "vector.h"

//quaternion state indices
#define Q1 0
#define Q2 1
#define Q3 2
#define Q4 3
//gyro offset state indices
#define xGyro_Bias 4 
#define yGyro_Bias 5
#define zGyro_Bias 6

#define GN_STATE_COUNT 4 
#define GN_JAC_COUNT 24

#define STATE_COUNT 7
#define STATE_COUNT_P2 49
#define MEASUREMENT_COUNT 3
#define MEASUREMENT_COUNT_P2 9
#define GPS_MEASUREMENT_COUNT_P2 4
#define GPS_MEASUREMENT_COUNT 2
#define STATEXMEASUREMENT_COUNT 21

#define INI_QUAT_VAR  0.1         // Initial quaternion variance
#define INI_BIAS_VAR  0.2         // Initial offsets variance 
#define PRO_BIAS_VAR  0.001  // Process gyro offsets noise variance 
#define PRO_QUAT_VAR  0.00001   // Process quaternion noise variance 
#define ACC_NORM_VAR  100          // Normalized accelerometers variance 
#define MAG_NORM_VAR  50          // Normalized accelerometers variance 
#define GPS_NORM_VAR 1
//#define PRO_QUAT_VAR  0.00001   // Process quaternion noise variance 
//#define ACC_NORM_VAR  500          // Normalized accelerometers variance 

#define DEFAULT_P_BIAS -0.0311499
#define DEFAULT_Q_BIAS -0.00967026
#define DEFAULT_R_BIAS -0.00459671

#define MAGREF_X 2.81	//north component in 10uTesla
#define MAGREF_Y -0.5454	//west component in 10uTesla
#define MAGREF_Z 4.4831 //up component in 10uTesla

#define MAX_TIME_STEP 0.5

using namespace Andromeda;

namespace Andromeda
{
    class ImuKalman
    {      
    public:
        fixed _pitch;
        fixed _roll;
        fixed _yaw;
        fixed _prevMag;
		bool _isInitialised;

    public:
        fixed _state[STATE_COUNT];
        fixed _A[STATE_COUNT_P2];
        fixed _P[STATE_COUNT_P2];
        fixed _Q[STATE_COUNT_P2];
        fixed _H[STATEXMEASUREMENT_COUNT];
        fixed _R_Mag[MEASUREMENT_COUNT_P2];
		fixed _R_Gps[GPS_MEASUREMENT_COUNT_P2];	
		fixed _R_Accel[MEASUREMENT_COUNT_P2];
        fixed _K[STATEXMEASUREMENT_COUNT];

        fixed _temp1[STATE_COUNT_P2];
        fixed _temp2[STATE_COUNT_P2];
        fixed _temp3[STATE_COUNT_P2];
        fixed _eulerAngles[3];
        fixed  _tempV [MAXSIZE] ;

		fixed _magReference[3];	


		//-----------------------------------------------------------------------------
		// ImuKalman
		//
		// Constructor function. This function needs the magnetic inclination and declination
		// values in degrees for the given location. This can be obtained from:
		// http://www.ngdc.noaa.gov/geomagmodels/IGRFWMM.jsp
		//-----------------------------------------------------------------------------
		ImuKalman();
		bool isInitialised() { return _isInitialised; }
		void initialize(float xAccel, float yAccel, float zAccel, float xMagn, float yMagn, float zMagn);
		void iterateGaussNewton(fixed xAccel, fixed yAccel, fixed zAccel,fixed xMag, fixed yMag, fixed zMag);
		void measurementUpdate(fixed xAccel, fixed yAccel, fixed zAccel,fixed xMag, fixed yMag, fixed zMag, bool useAccel = true, bool useMag = true);
		void measurementUpdate(fixed xAccel, fixed yAccel, fixed zAccel,fixed xGpsComponent, fixed yGpsComponent, bool useAccel = true, bool useGps = true);
		void timeUpdate(fixed xGyro_raw, fixed yGyro_raw, fixed zGyro_raw, fixed dT);
		bool normalizeState();
		void kalmanPredict(fixed *state, fixed *P, fixed *A , fixed *Q);
		void kalmanCorrect( fixed *P, fixed *state, fixed *C, fixed *R, fixed *E, fixed *K, int n, int m );
		
    };
}
