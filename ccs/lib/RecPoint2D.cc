
#include "RecGeometry.h"


using namespace std;
/*****************************************************************************
 * CONSTANTS
 *****************************************************************************/
const double RecPoint2D::tolerance = 1e-6;

/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecPoint2D
 * DESCRIPTION: no argument constructor, sets x and y to zero 
 *****************************************************************************/
RecPoint2D::RecPoint2D(void):x(0.0), y(0.0) { }

/******************************************************************************
 *      METHOD: RecPoint2D
 * DESCRIPTION: two argument constructor, sets x and y to passed x and y values 
 *****************************************************************************/
RecPoint2D::RecPoint2D(double inx, double iny):x(inx), y(iny) { }

/******************************************************************************
 *      METHOD: RecPoint2D
 * DESCRIPTION: copy constructor, copys the fields of the passed point 
 *****************************************************************************/
RecPoint2D::RecPoint2D(const RecPoint2D& pt2D):x(pt2D.x), y(pt2D.y) { }

/******************************************************************************
 *      METHOD: +=operator
 * DESCRIPTION: increments the fields of the point by the fields of the 
 *              passed point 
 *****************************************************************************/
void RecPoint2D::operator+=(const RecPoint2D& b)
{
	x += b.x;
	y += b.y; 
}

void RecPoint2D::operator+=(const RecVector2D &v)
{
    x+=v.x;
    y+=v.y;
}

/******************************************************************************
 *      METHOD: -=operator
 * DESCRIPTION: decrements the fields of the point by the fields of the 
 *              passed point 
 *****************************************************************************/
void RecPoint2D::operator-=(const RecPoint2D& b)
{
	x -= b.x;
	y -= b.y; 
}

void RecPoint2D::operator-=(const RecVector2D &v)
{
    x-=v.x;
    y-=v.y;
}


/******************************************************************************
 *      METHOD: *=operator
 * DESCRIPTION: multiplies the fields of the point by the passed double.
 *****************************************************************************/
void RecPoint2D::operator*=(double b)
{
	x *= b;
	y *= b; 
}

/******************************************************************************
 *      METHOD: /=operator
 * DESCRIPTION: divides the fields of the point by the passed double.  Does
 *              nothing if the passed value is zero
 *****************************************************************************/
void RecPoint2D::operator/=(double b)
{
	if(b == 0.0)
	{
		cerr << "Attempting to divide by zero in 2D point division." << endl;
		return;
	}
	x /= b;
	y /= b; 
}

/******************************************************************************
 *      METHOD: =operator
 * DESCRIPTION: sets the fields of the point to those of the passed point.
 *****************************************************************************/
void RecPoint2D::operator=(const RecPoint2D& b)
{
	x = b.x;
	y = b.y; 
}

/******************************************************************************
 *      METHOD: +operator
 * DESCRIPTION: adds the fields of the passed points and returns the result
 *              in a new point structure.
 *****************************************************************************/
RecPoint2D operator+(const RecPoint2D& a, const RecPoint2D& b)
{
    RecPoint2D c(a);
	c += b;
    return c;
}

RecPoint2D operator+(const RecPoint2D& a, const RecVector2D &v)
{
    RecPoint2D c(a);
    c+= v;
    return c;
}

/******************************************************************************
 *      METHOD: -operator
 * DESCRIPTION: subtracts the fields of the second point from the fields of the
 *              first point and returns the result in a new point structure.
 *****************************************************************************/
RecPoint2D operator-(const RecPoint2D& a, const RecPoint2D& b)
{
    RecPoint2D c(a);
    c -= b;
    return c;
}

RecPoint2D operator-(const RecPoint2D& a, const RecVector2D& v)
{
    RecPoint2D c(a);
    c -= v;
    return c;
}



/******************************************************************************
 *      METHOD: *operator
 * DESCRIPTION: multiplies the fields of the passed point by the passed double
 *              and returns the result in a new point structure.
 *****************************************************************************/
RecPoint2D operator*(const RecPoint2D& a, double b)
{
    RecPoint2D c(a);
	c *= b;
    return c;
}

/******************************************************************************
 *      METHOD: *operator
 * DESCRIPTION: multiplies the fields of the passed point by the passed double
 *              and returns the result in a new point structure.
 *****************************************************************************/
RecPoint2D operator*(double b, const RecPoint2D& a)
{
	return a * b;
}

/******************************************************************************
 *      METHOD: /operator
 * DESCRIPTION: divides the fields of the passed point by the passed double and
 *              returns the result in a new point structure.  Returns a copy
 *              of the passed point if the passed double is zero.
 *****************************************************************************/
RecPoint2D operator/(const RecPoint2D& a, double b)
{
    RecPoint2D c(a);
	c /= b;
    return c;
}

/******************************************************************************
 *      METHOD: ==operator
 * DESCRIPTION: compares two points and returns true if all the fields of the
 *              first point are within the tolerance of the second point 
 *****************************************************************************/
bool operator==(const RecPoint2D& a, const RecPoint2D& b)
{
    return( (fabs(a.x - b.x) < RecPoint2D::tolerance) &&
            (fabs(a.y - b.y) < RecPoint2D::tolerance) );
}

/******************************************************************************
 *      METHOD: !=operator
 * DESCRIPTION: compares two points and returns true if all the fields of the
 *              first point are not within the tolerance of the second point 
 *****************************************************************************/
bool operator!=(const RecPoint2D& a, const RecPoint2D& b)
{
	return !(a == b);
}

/******************************************************************************
 *      METHOD: distanceSq
 * DESCRIPTION: returns the squared distance between two points
 *****************************************************************************/
double RecPoint2D::distanceSq(const RecPoint2D& a) const
{
	return((x - a.x) * (x - a.x) + (y - a.y) * (y - a.y));
}

/******************************************************************************
 *      METHOD: distance
 * DESCRIPTION: returns the distance between two points
 *****************************************************************************/
double RecPoint2D::distance(const RecPoint2D& a) const
{
	return(sqrt(distanceSq(a)));
}


/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: puts the point with nice formatting to the passed output stream
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecPoint2D& a)
{
    return out << "( " << a.x << ", " << a.y << " )";
}

