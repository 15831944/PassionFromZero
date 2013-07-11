
#include "RecGeometry.h"

/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecVector2D
 * DESCRIPTION: no argument constructor, sets x and y to zero 
 *****************************************************************************/
RecVector2D::RecVector2D(void):x(0), y(0) { }

/******************************************************************************
 *      METHOD: RecVector2D
 * DESCRIPTION: two argument constructor, sets x and y to passed x and y values 
 *****************************************************************************/
RecVector2D::RecVector2D(double inx, double iny):x(inx),y(iny) { }

/******************************************************************************
 *      METHOD: RecPoint2D
 * DESCRIPTION: copy constructor, copys the fields of the passed vector 
 *****************************************************************************/
RecVector2D::RecVector2D(const RecVector2D& v2D):x(v2D.x), y(v2D.y) {}
RecVector2D::RecVector2D(const RecPoint2D& p2D):x(p2D.x),y(p2D.y) { }

void RecVector2D::operator+=(const RecVector2D& b)
{
    x+=b.x; y+=b.y;
}

void RecVector2D::operator-=(const RecVector2D& b)
{
    x-=b.x; y-=b.y;
}

void RecVector2D::operator*=( double b)
{
    x*=b; y*=b;
}

void RecVector2D::operator/=( double b)
{
    x/=b; y/=b;
}


/******************************************************************************
 *      METHOD: =operator
 * DESCRIPTION: sets the fields of the vector to those of the passed vector.
 *****************************************************************************/
void RecVector2D::operator=(const RecVector2D& b)
{
	x = b.x;
	y = b.y; 
}

/******************************************************************************
 *      METHOD: +operator
 * DESCRIPTION: adds the fields of the passed vectors and returns the result
 *              in a new vector structure.
 *****************************************************************************/
RecVector2D operator+(const RecVector2D& a, const RecVector2D& b)
{
    RecVector2D c(a);
	c += b;
    return c;
}

/******************************************************************************
 *      METHOD: -operator
 * DESCRIPTION: subtracts the fields of the second vector from the fields of 
 *              the first vector and returns the result in a new vector.
 *****************************************************************************/
RecVector2D operator-(const RecVector2D& a, const RecVector2D& b)
{
    RecVector2D c(a);
    c -= b;
    return c;
}

/******************************************************************************
 *      METHOD: *operator
 * DESCRIPTION: multiplies the fields of the passed vector by the passed double
 *              and returns the result in a new vector structure.
 *****************************************************************************/
RecVector2D operator*(const RecVector2D& a, double b)
{
    RecVector2D c(a);
	c *= b;
    return c;
}

/******************************************************************************
 *      METHOD: *operator
 * DESCRIPTION: multiplies the fields of the passed vector by the passed double
 *              and returns the result in a new vector structure.
 *****************************************************************************/
RecVector2D operator*(double b, const RecVector2D& a)
{
	return a * b;
}

/******************************************************************************
 *      METHOD: /operator
 * DESCRIPTION: divides the fields of the passed vector by the passed double 
 *              and returns the result in a new vector structure.  Returns a 
 *              copy of the passed vector if the passed double is zero.
 *****************************************************************************/
RecVector2D operator/(const RecVector2D& a, double b)
{
    RecVector2D c(a);
    //if (b!=0)		//MAT 10-5-06 - added divide by zero protection
    //{                 //MAT 10-9-06 - removed divide by zero protection
        c /= b;
    //}
    return c;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: computes and returns the dot product of the two passed vectors 
 *****************************************************************************/
double operator*(const RecVector2D& a, const RecVector2D& b)
{
	return(a.x * b.x + a.y * b.y);
}


bool operator==(const RecVector2D& a, const RecVector2D& b)
{
    return( (fabs(a.x - b.x) < RecPoint2D::tolerance) &&
            (fabs(a.y - b.y) < RecPoint2D::tolerance));
}

bool operator!=(const RecVector2D& a, const RecVector2D& b)
{
	return !(a == b);
}
/******************************************************************************
 *      METHOD: dot
 * DESCRIPTION: computes and returns the dot product of this vector with
 *              the passed vector 
 *****************************************************************************/
double RecVector2D::dot(const RecVector2D& a) const
{
	return((*this)*a);
}

/******************************************************************************
 *      METHOD: xprod
 * DESCRIPTION: computes and returns the cross product of this vector with
 *              the passed vector 
 *****************************************************************************/
double RecVector2D::xprod(const RecVector2D& a) const
{
    return (x*a.y) - (y*a.x);
}

/******************************************************************************
 *      METHOD: lengthSq
 * DESCRIPTION: computes and returns the square of the length of the vector 
 *****************************************************************************/
double RecVector2D::lengthSq(void) const
{
	return(x*x + y*y);
}

/******************************************************************************
 *      METHOD: length
 * DESCRIPTION: computes and returns the length of the vector 
 *****************************************************************************/
double RecVector2D::length(void) const
{
	return(sqrt(lengthSq()));
}

/******************************************************************************
 *      METHOD: normalize
 * DESCRIPTION: makes the vector a unit vector that has the same direction
 *              as the old vector
 *****************************************************************************/
void RecVector2D::normalize(void)
{
    //double newlength = length();
    //if (newlength!=0)		//MAT 10-5-06 - added divide by zero protection
    //{
    //	(*this) /= newlength;
    //}

    (*this) /= length(); 	//MAT 10-9-06 - removed divide by zero protection
}

RecVector2D RecVector2D::normal(void) const
{
    return RecVector2D(-y,x);
}

ostream& operator<<(ostream& out, const RecVector2D& a)
{
    return out << "( " << a.x << ", " << a.y << " )";
}

RecVector2D RecVector2D::getUnitVector(const RecRadians& radians)
{
    return RecVector2D(cos(radians),sin(radians));
}
