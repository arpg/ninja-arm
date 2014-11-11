#pragma once

#include <math.h>

//#define	RESOLUTION			1000000L
#define	RESOLUTION			1048576L

#define	RESOLUTION_FLOAT	1048576.0
#define	RESOLUTION_FLOATf	1048576.0f
#define	FLOAT_RESOLUTION	0.0000005
#define	FLOAT_RESOLUTIONf	0.0000005f
#define _XPI      3294198 // 3.1415926535897932384626433832795
#define XPI      fixed(RAW,_XPI)

#define _X2PI     6588397 // 6.283185307179586476925286766559
#define X2PI     fixed(RAW,_X2PI)

#define XPI_NEGATIVE fixed(RAW, _XPI_NEGATIVE)
#define _XPI_NEGATIVE -3294198

#define _XPIO2    1647099 // 1.5707963267948966192313216916398
#define XPIO2    fixed(RAW,_XPIO2)
#define _XPIO3    1098066 // 1.0471975511965977461542144610932
#define XPIO3    fixed(RAW,_XPIO3)
#define _XPIO4    823549.5 // 0.78539816339744830961566084581988
#define XPIO4    fixed(RAW, _XPIO4)
#define _XPIO4X3    2470648 // 2.3561944901923449288469825374596
#define XPIO4X3    fixed(RAW, _XPIO4X3)
#define _XLN_E    2850325 // 2.71828182845904523536
#define XLN_E    fixed(RAW,_XLN_E)
#define _XLN_10   2414435 // 2.30258509299404568402
#define XLN_10   fixed(RAW,_XLN_10)
#define _XRAD_TO_DEG 60078979
#define XRAD_TO_DEG fixed(RAW,_XRAD_TO_DEG)
#define _XDEG_TO_RAD 18301 //0.01745329251994329576923690768489
#define XDEG_TO_RAD fixed(RAW,_XDEG_TO_RAD)
#define sqrt_error   fixed(RAW, 1048) // 0.001

//        const static long BP= 20;  // how many low bits are right of Binary Point
//        const static long BP2= BP*2;  // how many low bits are right of Binary Point
//        const static long BPhalf= BP/2;  // how many low bits are right of Binary Point
//
//        static double STEP() { return 1.0 / (1<<BP); }  // smallest step we can represent




namespace Andromeda
{
	enum fixedRaw { RAW };
			
	
#define BP 20
#define BP2 40
#define BPhalf 10
#define STEP() 0.00000095367431640625

    class fixed 
	{
    public:
		long	g; // the guts
        inline fixed(fixedRaw, int guts) : g(guts) {}
        //fixed(bool bInternal, long nVal): g(nVal) {}  
        fixed() : g(0) {}
        fixed(const fixed& a) : g( a.g ) {}
        fixed(float a) : g( int(a / (float)STEP()) ) {}
        fixed(double a) : g( int(a / (double)STEP()) ) {}
        fixed(int a) : g( a << BP ) {}
        fixed(long a) : g( a << BP ) {}
        fixed& operator =(const fixed& a) { g= a.g; return *this; }
        fixed& operator =(float a) { g= fixed(a).g; return *this; }
        fixed& operator =(double a) { g= fixed(a).g; return *this; }
        fixed& operator =(int a) { g= fixed(a).g; return *this; }
        fixed& operator =(long a) { g= fixed(a).g; return *this; }
        long GetLong(void) { return g; }

        operator float() { return g * (float)STEP(); }
        operator double() { return g * (double)STEP(); }
        operator int() { return g>>BP; }
        operator short int() { return g>>BP; }
        operator long() { return g>>BP; }



        fixed operator +() const { return fixed(RAW,g); }
        fixed operator -() const { return fixed(RAW,-g); }

        fixed operator +(const fixed& a) const { return fixed(RAW, g + a.g); }
        fixed operator -(const fixed& a) const { return fixed(RAW, g - a.g); }
#if 1
        // more acurate, using long long
        inline fixed operator *(const fixed& a) const { return fixed(RAW,  (int)( ((long long)g * (long long)a.g ) >> BP)); }
#else
        // faster, but with only half as many bits right of binary point
        inline fixed operator *(const fixed& a) const { return fixed(RAW, (g>>BPhalf) * (a.g>>BPhalf) ); }
#endif
        inline fixed operator /(const fixed& a) const { return fixed(RAW, int( (((long long)g << BP2) / (long long)(a.g)) >> BP) ); }

        inline fixed operator % (const fixed& a) const{ return fixed(RAW,g % a.g); }
        inline fixed operator % (const int a) const{ return fixed(RAW,g % fixed(a).g); }

        inline fixed operator +(float a) const { return fixed(RAW, g + fixed(a).g); }
        inline fixed operator -(float a) const { return fixed(RAW, g - fixed(a).g); }
        inline fixed operator *(float a) const { return fixed(RAW, (g>>BPhalf) * (fixed(a).g>>BPhalf) ); }
        inline fixed operator /(float a) const { return fixed(RAW, int( (((long long)g << BP2) / (long long)(fixed(a).g)) >> BP) ); }

        inline fixed operator +(double a) const { return fixed(RAW, g + fixed(a).g); }
        inline fixed operator -(double a) const { return fixed(RAW, g - fixed(a).g); }
        inline fixed operator *(double a) const { return fixed(RAW, (g>>BPhalf) * (fixed(a).g>>BPhalf) ); }
        inline fixed operator /(double a) const { return fixed(RAW, int( (((long long)g << BP2) / (long long)(fixed(a).g)) >> BP) ); }

        inline fixed operator +(int a) const { return fixed(RAW, g + fixed(a).g); }
        inline fixed operator -(int a) const { return fixed(RAW, g - fixed(a).g); }
        inline fixed operator *(int a) const { return fixed(RAW, (g>>BPhalf) * (fixed(a).g>>BPhalf) ); }
        inline fixed operator /(int a) const { return fixed(RAW, int( (((long long)g << BP2) / (long long)(fixed(a).g)) >> BP) ); }


        inline fixed& operator +=(fixed a) { return *this = *this + a; return *this; }
        inline fixed& operator -=(fixed a) { return *this = *this - a; return *this; }
        inline fixed& operator *=(fixed a) { return *this = *this * a; return *this; }
        inline fixed& operator /=(fixed a) { return *this = *this / a; return *this; }

        inline fixed& operator +=(int a) { return *this = *this + (fixed)a; return *this; }
        inline fixed& operator -=(int a) { return *this = *this - (fixed)a; return *this; }
        inline fixed& operator *=(int a) { return *this = *this * (fixed)a; return *this; }
        inline fixed& operator /=(int a) { return *this = *this / (fixed)a; return *this; }

        inline fixed& operator +=(long a) { return *this = *this + (fixed)a; return *this; }
        inline fixed& operator -=(long a) { return *this = *this - (fixed)a; return *this; }
        inline fixed& operator *=(long a) { return *this = *this * (fixed)a; return *this; }
        inline fixed& operator /=(long a) { return *this = *this / (fixed)a; return *this; }

        inline fixed& operator +=(float a) { return *this = *this + a; return *this; }
        inline fixed& operator -=(float a) { return *this = *this - a; return *this; }
        inline fixed& operator *=(float a) { return *this = *this * a; return *this; }
        inline fixed& operator /=(float a) { return *this = *this / a; return *this; }

        inline fixed& operator +=(double a) { return *this = *this + a; return *this; }
        inline fixed& operator -=(double a) { return *this = *this - a; return *this; }
        inline fixed& operator *=(double a) { return *this = *this * a; return *this; }
        inline fixed& operator /=(double a) { return *this = *this / a; return *this; }

        inline bool operator ==(const fixed& a) const { return g == a.g; }
        inline bool operator !=(const fixed& a) const { return g != a.g; }
        inline bool operator <=(const fixed& a) const { return g <= a.g; }
        inline bool operator >=(const fixed& a) const { return g >= a.g; }
        inline bool operator  <(const fixed& a) const { return g  < a.g; }
        inline bool operator  >(const fixed& a) const { return g  > a.g; }

        inline bool operator ==(float a) const { return g == fixed(a).g; }
        inline bool operator !=(float a) const { return g != fixed(a).g; }
        inline bool operator <=(float a) const { return g <= fixed(a).g; }
        inline bool operator >=(float a) const { return g >= fixed(a).g; }
        inline bool operator  <(float a) const { return g  < fixed(a).g; }
        inline bool operator  >(float a) const { return g  > fixed(a).g; }

        inline bool operator ==(double a) const { return g == fixed(a).g; }
        inline bool operator !=(double a) const { return g != fixed(a).g; }
        inline bool operator <=(double a) const { return g <= fixed(a).g; }
        inline bool operator >=(double a) const { return g >= fixed(a).g; }
        inline bool operator  <(double a) const { return g  < fixed(a).g; }
        inline bool operator  >(double a) const { return g  > fixed(a).g; }

        inline bool operator ==(int a) const { return g == fixed(a).g; }
        inline bool operator !=(int a) const { return g != fixed(a).g; }
        inline bool operator <=(int a) const { return g <= fixed(a).g; }
        inline bool operator >=(int a) const { return g >= fixed(a).g; }
        inline bool operator  <(int a) const { return g  < fixed(a).g; }
        inline bool operator  >(int a) const { return g  > fixed(a).g; }



        static fixed cos(fixed val)
        {
            return sin(val + XPIO4);
        }

        static fixed sin(fixed val)
        {
            long x = val.GetLong();
            x >> 7;
            // S(x) = x * ( (3<<p) - (x*x>>r) ) >> s
            // n : Q-pos for quarter circle             13
            // A : Q-pos for output                     12
            // p : Q-pos for parentheses intermediate   15
            // r = 2n-p                                 11
            // s = A-1-p-n                              17

            static const int qN = 13, qA= 12, qP= 15, qR= 2*qN-qP, qS= qN+qP+1-qA;

            x= x<<(30-qN);          // shift to full s32 range (Q13->Q30)

            if( (x^(x<<1)) < 0)     // test for quadrant 1 or 2
                x= (1<<31) - x;

            x= x>>(30-qN);

            long res = x * ( (3<<qP) - (x*x>>qR) ) >> qS;

            res << 8;
            return fixed(RAW,res);
        }





        //this function is referenced here
        //http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
        static fixed atan2x( fixed y, fixed x )
        {
            fixed coeff_1 = XPIO4;  //this is really XPIO4
            fixed coeff_2 = XPIO4X3;

            fixed abs_y = absx(y)+fixed(RAW,1);      // kludge to prevent 0/0 condition
            fixed r = 0;
            fixed angle = 0;

            if (x>=0)
            {
				//divide by zero protection
				fixed temp = (x + abs_y);
				if(temp == 0 )
					return 0;
                r = (x - abs_y) / temp;
                angle = coeff_1 - coeff_1 * r;
            }
            else
            {
				//divide by zero protection
				fixed temp = (abs_y - x);
				if(temp == 0 )
					return 0;
                r = (x + abs_y) / temp;
                angle = coeff_2 - coeff_1 * r;
            }
            if (y < 0)
                return(fixed(-1)*angle);     // negate if in quad III or IV
            else
                return(angle);
        }

        // implementation of asin
        static fixed asin(fixed arg)
        {
            fixed temp;
            int sign;

            sign = 0;
            if(arg < 0)
            {
                arg = -arg;
                sign++;
            }
            if(arg > fixed(1))
                return 0;
            temp = fixed::sqrt(fixed(1) - arg*arg);
            if(arg > fixed(0.7))
                temp = XPIO2 - fixed::atan2x(temp,arg);
            else
                temp =  fixed::atan2x(arg,temp);
            if(sign > 0)
                temp = -temp;
            return temp;
        }

        // implementation of acos
        static fixed acos(fixed arg)
        {
            if(arg > 1 || arg < -1)
                return 0;
            return XPIO2 - asin(arg);
        }


        static fixed absx( fixed p_Base )
        {
            if( p_Base < 0 ) 
                return p_Base * -1;

            return p_Base;
        }

        static fixed iExp2(fixed p_Base)
        {
            fixed w;
            fixed y;
            int num;

            for( w=1, y=1, num=1 ; y != y+w ; ++num )
                y += ( w *= p_Base / num );

            return y;
        }

        static fixed iLog2( fixed p_Base )
        {   
            fixed w = 0;
            fixed y = 0;
            fixed z = 0;
            int num = 1;
            int dec = 0;

            if( p_Base == 1 )
                return 0;

            for( dec=0 ; absx( p_Base ) >= 2 ; ++dec )
                p_Base /= XLN_E;

            p_Base -= 1;
            z = p_Base;
            y = p_Base;
            w = 1;

            while( y != y + w )
                y += ( w = ( z = fixed(0) - ( z * p_Base ) ) / ( num += 1 ) );

            return y + dec;
        }

        static fixed ilog10( fixed p_Base )
        {
            return iLog2( p_Base ) / XLN_10;
        }

        //this routine from http://www.devmaster.net/articles/fixed-point-optimizations/
        static fixed sqrt(fixed f)
        {
            long value = f.GetLong();
            if (value != 0) {
                long g = 0;
                short bshft = 15;
                long b = 1<<bshft;
                do {
                    long temp = (g+g+b)<<bshft;
                    if (value >= temp) {
                        g += b;
                        value -= temp;
                    }
                    b>>=1;
                } while (bshft--);

                return fixed(RAW,g*1000);
            }
            else
                return 0;
        }


        static fixed ipow( fixed p_Base, fixed p_Power )
        {
            if( p_Base < 0 && p_Power%2 != 0 )
                return  iExp2( (p_Power * iLog2( p_Base * -1 )) ) * -1;
            else
                return iExp2( (p_Power * iLog2(absx( p_Base ))) );
        }
    };

    inline fixed operator +(float a, const fixed& b) { return fixed(a)+b; }
    inline fixed operator -(float a, const fixed& b) { return fixed(a)-b; }
    inline fixed operator *(float a, const fixed& b) { return fixed(a)*b; }
    inline fixed operator /(float a, const fixed& b) { return fixed(a)/b; }

    inline bool operator ==(float a, const fixed& b) { return fixed(a) == b; }
    inline bool operator !=(float a, const fixed& b) { return fixed(a) != b; }
    inline bool operator <=(float a, const fixed& b) { return fixed(a) <= b; }
    inline bool operator >=(float a, const fixed& b) { return fixed(a) >= b; }
    inline bool operator  <(float a, const fixed& b) { return fixed(a)  < b; }
    inline bool operator  >(float a, const fixed& b) { return fixed(a)  > b; }



    inline fixed operator +(double a, const fixed& b) { return fixed(a)+b; }
    inline fixed operator -(double a, const fixed& b) { return fixed(a)-b; }
    inline fixed operator *(double a, const fixed& b) { return fixed(a)*b; }
    inline fixed operator /(double a, const fixed& b) { return fixed(a)/b; }

    inline bool operator ==(double a, const fixed& b) { return fixed(a) == b; }
    inline bool operator !=(double a, const fixed& b) { return fixed(a) != b; }
    inline bool operator <=(double a, const fixed& b) { return fixed(a) <= b; }
    inline bool operator >=(double a, const fixed& b) { return fixed(a) >= b; }
    inline bool operator  <(double a, const fixed& b) { return fixed(a)  < b; }
    inline bool operator  >(double a, const fixed& b) { return fixed(a)  > b; }


    inline int& operator +=(int& a, const fixed& b) { a = (fixed)a + b; return a; }
    inline int& operator -=(int& a, const fixed& b) { a = (fixed)a - b; return a; }
    inline int& operator *=(int& a, const fixed& b) { a = (fixed)a * b; return a; }
    inline int& operator /=(int& a, const fixed& b) { a = (fixed)a / b; return a; }

    inline long& operator +=(long& a, const fixed& b) { a = (fixed)a + b; return a; }
    inline long& operator -=(long& a, const fixed& b) { a = (fixed)a - b; return a; }
    inline long& operator *=(long& a, const fixed& b) { a = (fixed)a * b; return a; }
    inline long&  operator /=(long& a, const fixed& b) { a = (fixed)a / b; return a; }

    inline float& operator +=(float& a, const fixed& b) { a = a + b; return a; }
    inline float& operator -=(float& a, const fixed& b) { a = a - b; return a; }
    inline float& operator *=(float& a, const fixed& b) { a = a * b; return a; }
    inline float& operator /=(float& a, const fixed& b) { a = a / b; return a; }

    inline double& operator +=(double& a, const fixed& b) { a = a + b; return a; }
    inline double& operator -=(double& a, const fixed& b) { a = a - b; return a; }
    inline double& operator *=(double& a, const fixed& b) { a = a * b; return a; }
    inline double& operator /=(double& a, const fixed& b) { a = a / b; return a; }
				
	/*
	#define STEP() 1

	class fixed 
	{

    private:

        float	g; // the guts
    public:
        inline fixed(fixedRaw, int guts) : g(guts/RESOLUTION_FLOAT) {}
        //fixed(bool bInternal, long nVal): g(nVal) {}  
        fixed() : g(0) {}
        fixed(const fixed& a) : g( a.g ) {}
        fixed(float a) : g( a) {}
        fixed(double a) : g( a) {}
        fixed(int a) : g( a ) {}
        fixed(long a) : g( a ) {}
        fixed& operator =(const fixed& a) { g= a.g; return *this; }
        fixed& operator =(float a) { g= fixed(a).g; return *this; }
        fixed& operator =(double a) { g= fixed(a).g; return *this; }
        fixed& operator =(int a) { g= fixed(a).g; return *this; }
        fixed& operator =(long a) { g= fixed(a).g; return *this; }
        long GetLong(void) { return g*RESOLUTION_FLOAT; }

        operator float() { return g; }
        operator double() { return (double)g; }
        operator int() { return (int)g; }
        operator short int() { return (short)g; }
        operator long() { return (long)g; }



        fixed operator +() const { return fixed(g); }
        fixed operator -() const { return fixed(-g); }

        fixed operator +(const fixed& a) const { return fixed(g + a.g); }
        fixed operator -(const fixed& a) const { return fixed(g - a.g); }
#if 1
        // more acurate, using long long
        inline fixed operator *(const fixed& a) const { return fixed(g*a.g); }
#else
        // faster, but with only half as many bits right of binary point
        inline fixed operator *(const fixed& a) const { return fixed(RAW, (g>>BPhalf) * (a.g>>BPhalf) ); }
#endif
        inline fixed operator /(const fixed& a) const { return fixed(g/a.g); }

        inline fixed operator % (const fixed& a) const{ return fixed(RAW, fixed(g).GetLong() % fixed(a).GetLong()); }
        inline fixed operator % (const int a) const{ return fixed(RAW, fixed(g).GetLong() % fixed(a).GetLong()); }

        inline fixed operator +(float a) const { return fixed(g + fixed(a).g); }
        inline fixed operator -(float a) const { return fixed(g - fixed(a).g); }
        inline fixed operator *(float a) const { return fixed(g*a); }
        inline fixed operator /(float a) const { return fixed(g/a); }

        inline fixed operator +(double a) const { return fixed(g + fixed(a).g); }
        inline fixed operator -(double a) const { return fixed(g - fixed(a).g); }
        inline fixed operator *(double a) const { return fixed(g*a ); }
        inline fixed operator /(double a) const { return fixed(g/a); }

        inline fixed operator +(int a) const { return fixed(g+a); }
        inline fixed operator -(int a) const { return fixed(g-a); }
        inline fixed operator *(int a) const { return fixed(g*a); }
        inline fixed operator /(int a) const { return fixed(g/a ); }


        inline fixed& operator +=(fixed a) { return *this = *this + a; return *this; }
        inline fixed& operator -=(fixed a) { return *this = *this - a; return *this; }
        inline fixed& operator *=(fixed a) { return *this = *this * a; return *this; }
        inline fixed& operator /=(fixed a) { return *this = *this / a; return *this; }

        inline fixed& operator +=(int a) { return *this = *this + (fixed)a; return *this; }
        inline fixed& operator -=(int a) { return *this = *this - (fixed)a; return *this; }
        inline fixed& operator *=(int a) { return *this = *this * (fixed)a; return *this; }
        inline fixed& operator /=(int a) { return *this = *this / (fixed)a; return *this; }

        inline fixed& operator +=(long a) { return *this = *this + (fixed)a; return *this; }
        inline fixed& operator -=(long a) { return *this = *this - (fixed)a; return *this; }
        inline fixed& operator *=(long a) { return *this = *this * (fixed)a; return *this; }
        inline fixed& operator /=(long a) { return *this = *this / (fixed)a; return *this; }

        inline fixed& operator +=(float a) { return *this = *this + a; return *this; }
        inline fixed& operator -=(float a) { return *this = *this - a; return *this; }
        inline fixed& operator *=(float a) { return *this = *this * a; return *this; }
        inline fixed& operator /=(float a) { return *this = *this / a; return *this; }

        inline fixed& operator +=(double a) { return *this = *this + a; return *this; }
        inline fixed& operator -=(double a) { return *this = *this - a; return *this; }
        inline fixed& operator *=(double a) { return *this = *this * a; return *this; }
        inline fixed& operator /=(double a) { return *this = *this / a; return *this; }

        inline bool operator ==(const fixed& a) const { return g == a.g; }
        inline bool operator !=(const fixed& a) const { return g != a.g; }
        inline bool operator <=(const fixed& a) const { return g <= a.g; }
        inline bool operator >=(const fixed& a) const { return g >= a.g; }
        inline bool operator  <(const fixed& a) const { return g  < a.g; }
        inline bool operator  >(const fixed& a) const { return g  > a.g; }

        inline bool operator ==(float a) const { return g == fixed(a).g; }
        inline bool operator !=(float a) const { return g != fixed(a).g; }
        inline bool operator <=(float a) const { return g <= fixed(a).g; }
        inline bool operator >=(float a) const { return g >= fixed(a).g; }
        inline bool operator  <(float a) const { return g  < fixed(a).g; }
        inline bool operator  >(float a) const { return g  > fixed(a).g; }

        inline bool operator ==(double a) const { return g == fixed(a).g; }
        inline bool operator !=(double a) const { return g != fixed(a).g; }
        inline bool operator <=(double a) const { return g <= fixed(a).g; }
        inline bool operator >=(double a) const { return g >= fixed(a).g; }
        inline bool operator  <(double a) const { return g  < fixed(a).g; }
        inline bool operator  >(double a) const { return g  > fixed(a).g; }

        inline bool operator ==(int a) const { return g == fixed(a).g; }
        inline bool operator !=(int a) const { return g != fixed(a).g; }
        inline bool operator <=(int a) const { return g <= fixed(a).g; }
        inline bool operator >=(int a) const { return g >= fixed(a).g; }
        inline bool operator  <(int a) const { return g  < fixed(a).g; }
        inline bool operator  >(int a) const { return g  > fixed(a).g; }



        static fixed cos(fixed val)
        {
            return sin(val + XPIO4);
        }

        static fixed sin(fixed val)
        {
            long x = val.GetLong();
            x >> 7;
            // S(x) = x * ( (3<<p) - (x*x>>r) ) >> s
            // n : Q-pos for quarter circle             13
            // A : Q-pos for output                     12
            // p : Q-pos for parentheses intermediate   15
            // r = 2n-p                                 11
            // s = A-1-p-n                              17

            static const int qN = 13, qA= 12, qP= 15, qR= 2*qN-qP, qS= qN+qP+1-qA;

            x= x<<(30-qN);          // shift to full s32 range (Q13->Q30)

            if( (x^(x<<1)) < 0)     // test for quadrant 1 or 2
                x= (1<<31) - x;

            x= x>>(30-qN);

            long res = x * ( (3<<qP) - (x*x>>qR) ) >> qS;

            res << 8;
            return fixed(RAW,res);
        }





        //this function is referenced here
        //http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
        static fixed atan2x( fixed y, fixed x )
        {
            fixed coeff_1 = XPIO4;  //this is really XPIO4
            fixed coeff_2 = XPIO4X3;

            fixed abs_y = absx(y)+fixed(RAW,1);      // kludge to prevent 0/0 condition
            fixed r = 0;
            fixed angle = 0;

            if (x>=0)
            {
                r = (x - abs_y) / (x + abs_y);
                angle = coeff_1 - coeff_1 * r;
            }
            else
            {
                r = (x + abs_y) / (abs_y - x);
                angle = coeff_2 - coeff_1 * r;
            }
            if (y < 0)
                return(fixed(-1)*angle);     // negate if in quad III or IV
            else
                return(angle);
        }

        // implementation of asin
        static fixed asin(fixed arg)
        {
            fixed temp;
            int sign;

            sign = 0;
            if(arg < 0)
            {
                arg = -arg;
                sign++;
            }
            if(arg > fixed(1))
                return 0;
            temp = fixed::sqrt(fixed(1) - arg*arg);
            if(arg > fixed(0.7))
                temp = XPIO2 - fixed::atan2x(temp,arg);
            else
                temp =  fixed::atan2x(arg,temp);
            if(sign > 0)
                temp = -temp;
            return temp;
        }

        // implementation of acos
        static fixed acos(fixed arg)
        {
            if(arg > 1 || arg < -1)
                return 0;
            return XPIO2 - asin(arg);
        }


        static fixed absx( fixed p_Base )
        {
            if( p_Base < 0 ) 
                return p_Base * -1;

            return p_Base;
        }

        static fixed iExp2(fixed p_Base)
        {
            fixed w;
            fixed y;
            int num;

            for( w=1, y=1, num=1 ; y != y+w ; ++num )
                y += ( w *= p_Base / num );

            return y;
        }

        static fixed iLog2( fixed p_Base )
        {   
            fixed w = 0;
            fixed y = 0;
            fixed z = 0;
            int num = 1;
            int dec = 0;

            if( p_Base == 1 )
                return 0;

            for( dec=0 ; absx( p_Base ) >= 2 ; ++dec )
                p_Base /= XLN_E;

            p_Base -= 1;
            z = p_Base;
            y = p_Base;
            w = 1;

            while( y != y + w )
                y += ( w = ( z = fixed(0) - ( z * p_Base ) ) / ( num += 1 ) );

            return y + dec;
        }

        static fixed ilog10( fixed p_Base )
        {
            return iLog2( p_Base ) / XLN_10;
        }

        //this routine from http://www.devmaster.net/articles/fixed-point-optimizations/
        static fixed sqrt(fixed f)
        {
            long value = f.GetLong();
            if (value != 0) {
                long g = 0;
                short bshft = 15;
                long b = 1<<bshft;
                do {
                    long temp = (g+g+b)<<bshft;
                    if (value >= temp) {
                        g += b;
                        value -= temp;
                    }
                    b>>=1;
                } while (bshft--);

                return fixed(RAW,g*1000);
            }
            else
                return 0;
        }


        static fixed ipow( fixed p_Base, fixed p_Power )
        {
            if( p_Base < 0 && p_Power%2 != 0 )
                return  iExp2( (p_Power * iLog2( p_Base * -1 )) ) * -1;
            else
                return iExp2( (p_Power * iLog2(absx( p_Base ))) );
        }
    };

    inline fixed operator +(float a, const fixed& b) { return fixed(a)+b; }
    inline fixed operator -(float a, const fixed& b) { return fixed(a)-b; }
    inline fixed operator *(float a, const fixed& b) { return fixed(a)*b; }
    inline fixed operator /(float a, const fixed& b) { return fixed(a)/b; }

    inline bool operator ==(float a, const fixed& b) { return fixed(a) == b; }
    inline bool operator !=(float a, const fixed& b) { return fixed(a) != b; }
    inline bool operator <=(float a, const fixed& b) { return fixed(a) <= b; }
    inline bool operator >=(float a, const fixed& b) { return fixed(a) >= b; }
    inline bool operator  <(float a, const fixed& b) { return fixed(a)  < b; }
    inline bool operator  >(float a, const fixed& b) { return fixed(a)  > b; }



    inline fixed operator +(double a, const fixed& b) { return fixed(a)+b; }
    inline fixed operator -(double a, const fixed& b) { return fixed(a)-b; }
    inline fixed operator *(double a, const fixed& b) { return fixed(a)*b; }
    inline fixed operator /(double a, const fixed& b) { return fixed(a)/b; }

    inline bool operator ==(double a, const fixed& b) { return fixed(a) == b; }
    inline bool operator !=(double a, const fixed& b) { return fixed(a) != b; }
    inline bool operator <=(double a, const fixed& b) { return fixed(a) <= b; }
    inline bool operator >=(double a, const fixed& b) { return fixed(a) >= b; }
    inline bool operator  <(double a, const fixed& b) { return fixed(a)  < b; }
    inline bool operator  >(double a, const fixed& b) { return fixed(a)  > b; }


    inline int& operator +=(int& a, const fixed& b) { a = (fixed)a + b; return a; }
    inline int& operator -=(int& a, const fixed& b) { a = (fixed)a - b; return a; }
    inline int& operator *=(int& a, const fixed& b) { a = (fixed)a * b; return a; }
    inline int& operator /=(int& a, const fixed& b) { a = (fixed)a / b; return a; }

    inline long& operator +=(long& a, const fixed& b) { a = (fixed)a + b; return a; }
    inline long& operator -=(long& a, const fixed& b) { a = (fixed)a - b; return a; }
    inline long& operator *=(long& a, const fixed& b) { a = (fixed)a * b; return a; }
    inline long&  operator /=(long& a, const fixed& b) { a = (fixed)a / b; return a; }

    inline float& operator +=(float& a, const fixed& b) { a = a + b; return a; }
    inline float& operator -=(float& a, const fixed& b) { a = a - b; return a; }
    inline float& operator *=(float& a, const fixed& b) { a = a * b; return a; }
    inline float& operator /=(float& a, const fixed& b) { a = a / b; return a; }

    inline double& operator +=(double& a, const fixed& b) { a = a + b; return a; }
    inline double& operator -=(double& a, const fixed& b) { a = a - b; return a; }
    inline double& operator *=(double& a, const fixed& b) { a = a * b; return a; }
    inline double& operator /=(double& a, const fixed& b) { a = a / b; return a; }
			*/	

				
}
