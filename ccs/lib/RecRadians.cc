
#include "RecGeometry.h"
//#include <algorithm>


/*****************************************************************************
 * CONSTANTS
 *****************************************************************************/
#ifndef DEG2RAD
#define DEG2RAD		      0.017453292519943295474
#endif

#ifndef RAD2DEG
#define RAD2DEG		      57.295779513082322865
#endif

// Static Member Initialization
const double RecRadians::tolerance = 1e-6;

/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecRadians
 * DESCRIPTION: Constructors for the recRadians class
 *****************************************************************************/
RecRadians::RecRadians(void)
{
	radians = 0.0;
}

RecRadians::RecRadians(double rad)
{
	(*this) = rad;
}

RecRadians::RecRadians(const RecRadians& rad)
{
	(*this) = rad;
}


/******************************************************************************
 *      METHOD: operator double
 * DESCRIPTION: casts a radians object to a double
 *****************************************************************************/
RecRadians::operator double() const
{
	return radians;
}

/******************************************************************************
 *      METHOD: operator=
 * DESCRIPTION: radian assignment
 *****************************************************************************/
void RecRadians::operator=(double r)
{
	radians = r;
	scaleValue();
}

void RecRadians::operator=(const RecRadians& b)
{
	radians = b.radians;
}


/******************************************************************************
 *      METHOD: operator+=
 * DESCRIPTION: angle addition
 *****************************************************************************/
void RecRadians::operator+=(const RecRadians& b)
{
  (*this) = (*this) + b;
}
void RecRadians::operator+=(double b)
{
  (*this) = (*this) + b;
}

/******************************************************************************
 *      METHOD: operator-=
 * DESCRIPTION: angle subtraction
 *****************************************************************************/
void RecRadians::operator-=(const RecRadians& b)
{
  (*this) = (*this) - b;
}

void RecRadians::operator-=(double b)
{
  (*this) = (*this) - b;
}

/******************************************************************************
 *      METHOD: operator*=
 * DESCRIPTION: angle multiplication
 *****************************************************************************/
void RecRadians::operator*=(double b)
{
  (*this) = (*this) * b;
}

/******************************************************************************
 *      METHOD: operator/=
 * DESCRIPTION: angle division
 *****************************************************************************/
void RecRadians::operator/=(double b)
{
    if (b != 0)
    {
        (*this) = (*this) / b;
    }
}

/******************************************************************************
 *      METHOD: operator+
 * DESCRIPTION: angle addition
 *****************************************************************************/
RecRadians operator+(const RecRadians& a, const RecRadians& b)
{
	RecRadians r;

	r.radians = a.radians + b.radians;
	r.scaleValue();
	return r;
}

RecRadians operator+(const RecRadians& a, double b)
{
	return (a + RecRadians(b));
}

RecRadians operator+(double a, const RecRadians& b)
{
	return (RecRadians(a) + b);
}

/******************************************************************************
 *      METHOD: operator-
 * DESCRIPTION: angle subtraction
 *****************************************************************************/
RecRadians operator-(const RecRadians& a, const RecRadians& b)
{
	RecRadians r;

	r.radians = a.radians - b.radians;
	r.scaleValue();        
	return r;
}

RecRadians operator-(const RecRadians& a, double b)
{
	return (a - RecRadians(b));
}

RecRadians operator-(double a, const RecRadians& b)
{
	return (RecRadians(a) - b);
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: angle multiplication
 *****************************************************************************/
RecRadians operator*(const RecRadians& a, double b)
{
  RecRadians r;
  
  r.radians = a.radians * b;
  r.scaleValue();
	return r;
}

RecRadians operator*(double a, const RecRadians& b)
{
  return (b * a);
}

/******************************************************************************
 *      METHOD: operator/
 * DESCRIPTION: angle division by scaler
 *****************************************************************************/
RecRadians operator/(const RecRadians& a, double b)
{
  RecRadians r;
  
  if(b == 0.0)
  {
    cerr << "Attempting to divide by zero in recRadians division" << endl;
    return a;    //MAT 10-5-06 - return the unchanged angle if divide by zero is attempted
  }
  else
  {
    r.radians = a.radians / b;
    r.scaleValue();
    return r;    //MAT 10-5-06 - see above
  }
		
}

/******************************************************************************
 *      METHOD: operator==
 * DESCRIPTION: angle comparison
 *****************************************************************************/
bool operator==(const RecRadians& a, const RecRadians& b)
{
	return (fabs(double(a - b)) <= RecRadians::tolerance);
}

/******************************************************************************
 *      METHOD: operator!=
 * DESCRIPTION: angle comparison
 *****************************************************************************/
bool operator!=(const RecRadians& a, const RecRadians& b)
{
	return (!(a == b));
}

/******************************************************************************
 *      METHOD: operator>
 * DESCRIPTION: angle comparison
 *****************************************************************************/
bool operator>(const RecRadians& a, const RecRadians& b)
{
	return (((double(a) - double(b)) > 0) && (a != b));
}

/******************************************************************************
 *      METHOD: operator>=
 * DESCRIPTION: angle comparison
 *****************************************************************************/
bool operator>=(const RecRadians& a, const RecRadians& b)
{
	return ((a == b) || (a > b));
}

/******************************************************************************
 *      METHOD: operator<
 * DESCRIPTION: angle comparison
 *****************************************************************************/
bool operator<(const RecRadians& a, const RecRadians& b)
{
	return (b > a);
}

/******************************************************************************
 *      METHOD: operator<=
 * DESCRIPTION: angle comparison
 *****************************************************************************/
bool operator<=(const RecRadians& a, const RecRadians& b)
{
	return (b >= a);
}

/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: radian printing
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecRadians& a)
{
    return out <<  a.radians << " ";
}

/******************************************************************************
 *      METHOD: getDegrees
 * DESCRIPTION: convert the radian value to degrees and return as a double
 *****************************************************************************/
double RecRadians::getDegrees() const
{
	return (radians * RAD2DEG);
}

/******************************************************************************
 *      METHOD: setDegrees
 * DESCRIPTION: converts the passed degree value to radians and sets the 
 *              objects radian field to this value
 *****************************************************************************/
void RecRadians::setDegrees(double deg)
{
    radians = deg * DEG2RAD;
    scaleValue();
}

/*****************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: scaleValue
 * DESCRIPTION: scales the radian value between -PI and PI
 *****************************************************************************/
void RecRadians::scaleValue()
{
	int r;

	if(radians > -PI  && radians <= PI)
	{
		return;
	}

    if(fabs(radians) > 10e6)
    {
        cerr << "Warning: Scaling very large radian value " << radians <<
                "can result in significant round off error." << endl;
    }
 	r = (int)ceil((radians / (2.0 * PI)) - 0.5);
	radians -= (2 * PI * r);
}
