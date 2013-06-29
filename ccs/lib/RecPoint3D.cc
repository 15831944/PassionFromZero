#include "RecPoint3D.h"
#include <math.h>


//CONSTANTS
const double RecPoint3D::tolerance = 1e-6;

/******************************************************************************
 *      METHOD: RecPoint3D
 * DESCRIPTION: no argument constructor, sets x, y and z to zero
 *****************************************************************************/
RecPoint3D::RecPoint3D(void):x(0.0), y(0.0), z(0.0) { }

/******************************************************************************
 *      METHOD: RecPoint3D
 * DESCRIPTION: three argument constructor, sets x, y and z to passed
 *              x, y and z values
 *****************************************************************************/
RecPoint3D::RecPoint3D(double inx, double iny,
                       double inz):x(inx), y(iny), z(inz) { }

/******************************************************************************
 *      METHOD: RecPoint3D
 * DESCRIPTION: 2D point argument constructor, sets x, y to the x and y of the
 *              passed 2D point.  Sets z to zero.
 *****************************************************************************/
//RecPoint3D::RecPoint3D(const RecPoint2D& b):x(b.x), y(b.y), z(0.0) { }

/******************************************************************************
 *      METHOD: RecPoint3D
 * DESCRIPTION: copy constructor, copys the fields of the passed point
 *****************************************************************************/
RecPoint3D::RecPoint3D(const RecPoint3D& b):x(b.x), y(b.y), z(b.z) { }

/******************************************************************************
 *      METHOD: +=operator
 * DESCRIPTION: increments the fields of the point by the fields of the
 *              passed point
 *****************************************************************************/
void RecPoint3D::operator+=(const RecPoint3D& b)
{
    x += b.x;
    y += b.y;
    z += b.z;
}

/******************************************************************************
 *      METHOD: -=operator
 * DESCRIPTION: decrements the fields of the point by the fields of the
 *              passed point
 *****************************************************************************/
void RecPoint3D::operator-=(const RecPoint3D& b)
{
    x -= b.x;
    y -= b.y;
    z -= b.z;
}

/******************************************************************************
 *      METHOD: *=operator
 * DESCRIPTION: multiplies the fields of the point by the passed double.
 *****************************************************************************/
void RecPoint3D::operator*=(double b)
{
    x *= b;
    y *= b;
    z *= b;
}

/******************************************************************************
 *      METHOD: /=operator
 * DESCRIPTION: divides the fields of the point by the passed double.  Does
 *              nothing if the passed value is zero
 *****************************************************************************/
void RecPoint3D::operator/=(double b)
{
    if(b == 0.0)
    {
        cerr << "Attempting to divide by zero in 3D point division." << endl;
        return;
    }
    x /= b;
    y /= b;
    z /= b;
}

/******************************************************************************
 *      METHOD: =operator
 * DESCRIPTION: sets the fields of the point to those of the passed point.
 *****************************************************************************/
void RecPoint3D::operator=(const RecPoint3D& b)
{
    x = b.x;
    y = b.y;
    z = b.z;
}

/******************************************************************************
 *      METHOD: +operator
 * DESCRIPTION: adds the fields of the passed points and returns the result
 *              in a new point structure.
 *****************************************************************************/
RecPoint3D operator+(const RecPoint3D& a, const RecPoint3D& b)
{
  RecPoint3D c(a);
  c += b;
  return c;
}

/******************************************************************************
 *      METHOD: -operator
 * DESCRIPTION: subtracts the fields of the second point from the fields of the
 *              first point and returns the result in a new point structure.
 *****************************************************************************/
RecPoint3D operator-(const RecPoint3D& a, const RecPoint3D& b)
{
  RecPoint3D c(a);
    c -= b;
  return c;
}

/******************************************************************************
 *      METHOD: *operator
 * DESCRIPTION: multiplies the fields of the passed point by the passed double
 *              and returns the result in a new point structure.
 *****************************************************************************/
RecPoint3D operator*(const RecPoint3D& a, double b)
{
  RecPoint3D c(a);
    c *= b;
  return c;
}

/******************************************************************************
 *      METHOD: *operator
 * DESCRIPTION: multiplies the fields of the passed point by the passed double
 *              and returns the result in a new point structure.
 *****************************************************************************/
RecPoint3D operator*(double b, const RecPoint3D& a)
{
  return a * b;
}

/******************************************************************************
 *      METHOD: /operator
 * DESCRIPTION: divides the fields of the passed point by the passed double and
 *              returns the result in a new point structure.  Returns a copy
 *              of the passed point if the passed double is zero.
 *****************************************************************************/
RecPoint3D operator/(const RecPoint3D& a, double b)
{
  RecPoint3D c(a);
    if (b != 0)    // MAT 10-6-06- Added division by zero protection
    {
    c /= b;
    }
  return c;
}

/******************************************************************************
 *      METHOD: ==operator
 * DESCRIPTION: compares two points and returns true if all the fields of the
 *              first point are within the tolerance of the second point
 *****************************************************************************/
bool operator==(const RecPoint3D& a, const RecPoint3D& b)
{
 return((fabs(a.x - b.x) < RecPoint3D::tolerance) &&
        (fabs(a.y - b.y) < RecPoint3D::tolerance) &&
        (fabs(a.z - b.z) < RecPoint3D::tolerance));
}

/******************************************************************************
 *      METHOD: !=operator
 * DESCRIPTION: compares two points and returns true if all the fields of the
 *              first point are not within the tolerance of the second point
 *****************************************************************************/
bool operator!=(const RecPoint3D& a, const RecPoint3D& b)
{
  return !(a == b);
}

/******************************************************************************
 *      METHOD: distanceSq
 * DESCRIPTION: returns the squared distance between two points
 *****************************************************************************/
double RecPoint3D::distanceSq(const RecPoint3D& a) const
{
    return((x - a.x) * (x - a.x) + (y - a.y) * (y - a.y) +
        (z - a.z) * (z - a.z));
}

/******************************************************************************
 *      METHOD: distance
 * DESCRIPTION: returns the distance between two points
 *****************************************************************************/
double RecPoint3D::distance(const RecPoint3D& a) const
{
    return(sqrt(distanceSq(a)));
}

/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: puts the point with nice formatting to the passed output stream
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecPoint3D& a)
{
  return out << "( " << a.x << ", " << a.y << ", " << a.z << " )";
}







