
#include "RecGeometry.h"
#include <iomanip>
#include <memory.h>

/*****************************************************************************
 * CONSTANTS
 *****************************************************************************/
const double RecTransform2D::tolerance = 1e-6;

/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecTransform2D
 * DESCRIPTION: default constructor 
 *****************************************************************************/
RecTransform2D::RecTransform2D()
{
    identity();
}

/******************************************************************************
 *      METHOD: RecTransform2D
 * DESCRIPTION: copy constructor 
 *****************************************************************************/
RecTransform2D::RecTransform2D(const RecTransform2D& trans2D)
{
    memcpy(matrixPP, trans2D.matrixPP, sizeof(matrixPP));
}

/******************************************************************************
 *      METHOD: RecTransform2D
 * DESCRIPTION: constructor for converting a 2D pose into a 2D transform 
 *****************************************************************************/
RecTransform2D::RecTransform2D(const RecPose2D& pose)
{
    double ca;
    double sa;

    ca = cos(pose.rotZ);
    sa = sin(pose.rotZ);
  
    matrixPP[0][0] =  ca;
    matrixPP[0][1] = -sa;
    matrixPP[0][2] = pose.x;

    matrixPP[1][0] = sa;
    matrixPP[1][1] = ca;
    matrixPP[1][2] = pose.y;
}

/******************************************************************************
 *      METHOD: inverse
 * DESCRIPTION: computes and returns the inverse of the transform 
 *****************************************************************************/
RecTransform2D RecTransform2D::inverse(void)
{
    RecTransform2D inv;
    inv.matrixPP[0][0] = matrixPP[0][0];
    inv.matrixPP[0][1] = matrixPP[1][0];
    inv.matrixPP[0][2] = -matrixPP[0][0] * matrixPP[0][2] -
    matrixPP[1][0] * matrixPP[1][2];

    inv.matrixPP[1][0] = matrixPP[0][1];
    inv.matrixPP[1][1] = matrixPP[1][1];
    inv.matrixPP[1][2] = -matrixPP[0][1] * matrixPP[0][2] -
    matrixPP[1][1] * matrixPP[1][2];
    return inv;
}

/******************************************************************************
 *      METHOD: identity
 * DESCRIPTION: sets the rotation matrix to the identity matrix and the 
 *              translation vector to zero
 *****************************************************************************/
void RecTransform2D::identity(void)
{
	matrixPP[0][0] = 1.0;
	matrixPP[0][1] = 0.0;
	matrixPP[0][2] = 0.0;

	matrixPP[1][0] = 0.0;
	matrixPP[1][1] = 1.0;
	matrixPP[1][2] = 0.0;
}

/******************************************************************************
 *      METHOD: xAxis
 * DESCRIPTION: returns a vector of unit lenght that is the same direction
 *              as the transformed frame's x axis
 *****************************************************************************/
RecVector2D RecTransform2D::xAxis(void) const
{
	RecVector2D x(matrixPP[0][0], matrixPP[1][0]);
	return x;
}

/******************************************************************************
 *      METHOD: yAxis
 * DESCRIPTION: returns a vector of unit lenght that is the same direction
 *              as the transformed frame's y axis
 *****************************************************************************/
RecVector2D RecTransform2D::yAxis(void) const
{
	RecVector2D y(matrixPP[0][1], matrixPP[1][1]);
	return y;
}

/******************************************************************************
 *      METHOD: origin
 * DESCRIPTION: returns the vector that points to the origin of the transformed
 *              frame
 *****************************************************************************/
RecVector2D RecTransform2D::origin(void) const
{
	RecVector2D o(matrixPP[0][2], matrixPP[1][2]);
	return o;
}

/******************************************************************************
 *      METHOD: rotateAboutZAxis
 * DESCRIPTION: causes the transformed frame to be rotated about it's z axis
 *****************************************************************************/
void RecTransform2D::rotateAboutZAxis(RecRadians radAngle)
{
	double s = sin(double(radAngle));
	double c = cos(double(radAngle));
	RecTransform2D tempTrans(*this);
	
	matrixPP[0][0] = tempTrans[0][0] * c + tempTrans[0][1] * s;
	matrixPP[1][0] = tempTrans[1][0] * c + tempTrans[1][1] * s;
	matrixPP[0][1] = -tempTrans[0][0] * s + tempTrans[0][1] * c;
	matrixPP[1][1] = -tempTrans[1][0] * s + tempTrans[1][1] * c;	
}

/******************************************************************************
 *      METHOD: translate
 * DESCRIPTION: causes the transformed frame to translate by the passed vector
 *****************************************************************************/
void RecTransform2D::translate(const RecVector2D& transVect)
{
	matrixPP[0][2] += transVect.x;
	matrixPP[1][2] += transVect.y;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: multiplies two transforms together and returns the result in
 *              a new transform (thereby concatenating the transforms)
 *****************************************************************************/
RecTransform2D operator*(const RecTransform2D& b, const RecTransform2D& c)
{
    RecTransform2D a;
    a.matrixPP[0][0] = b.matrixPP[0][0]*c.matrixPP[0][0] + b.matrixPP[0][1]*c.matrixPP[1][0];
    a.matrixPP[0][1] = b.matrixPP[0][0]*c.matrixPP[0][1] + b.matrixPP[0][1]*c.matrixPP[1][1];
    a.matrixPP[0][2] = b.matrixPP[0][0]*c.matrixPP[0][2] + b.matrixPP[0][1]*c.matrixPP[1][2] + b.matrixPP[0][2];

    a.matrixPP[1][0] = b.matrixPP[1][0]*c.matrixPP[0][0] + b.matrixPP[1][1]*c.matrixPP[1][0];
    a.matrixPP[1][1] = b.matrixPP[1][0]*c.matrixPP[0][1] + b.matrixPP[1][1]*c.matrixPP[1][1];
    a.matrixPP[1][2] = b.matrixPP[1][0]*c.matrixPP[0][2] + b.matrixPP[1][1]*c.matrixPP[1][2] + b.matrixPP[1][2];

    return a;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: Transforms a 2D point by a transform
 *****************************************************************************/
RecPoint2D operator*(const RecTransform2D& tr, const RecPoint2D& pt)
{
    RecPoint2D newPt;
    newPt.x = tr.matrixPP[0][0]*pt.x + tr.matrixPP[0][1]*pt.y + tr.matrixPP[0][2];
    newPt.y = tr.matrixPP[1][0]*pt.x + tr.matrixPP[1][1]*pt.y + tr.matrixPP[1][2];
    return newPt;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: Transforms a 3D point by a transform, leaving z untouched
 *****************************************************************************/
RecPoint3D operator*(const RecTransform2D& tr, const RecPoint3D& pt)
{
    RecPoint3D newPt;
    newPt.x = tr.matrixPP[0][0]*pt.x + tr.matrixPP[0][1]*pt.y + tr.matrixPP[0][2];
    newPt.y = tr.matrixPP[1][0]*pt.x + tr.matrixPP[1][1]*pt.y + tr.matrixPP[1][2];
    return newPt;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: Transforms a 2D pose by a transform
 *****************************************************************************/
RecPose2D operator*(const RecTransform2D& trans, const RecPose2D& p)
{
    RecTransform2D transp(p);
    transp = trans * transp;
    return transp;
}

/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: puts the transform with nice formatting to the passed 
 *              output stream
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecTransform2D& trans2D)
{
    const int fw = 10; // output field width
    out << '[' << ' ' << setw(fw) << trans2D.matrixPP[0][0] << ' '
        << setw(fw) << trans2D.matrixPP[0][1] << ' '
        << setw(fw) << trans2D.matrixPP[0][2] << ' '
        << ']' << endl;
    out << '[' << ' ' << setw(fw) << trans2D.matrixPP[1][0] << ' '
        << setw(fw) << trans2D.matrixPP[1][1] << ' '
        << setw(fw) << trans2D.matrixPP[1][2] << ' '
        << ']' << endl;
    out << '[' << ' ' << setw(fw) << 0.0 << ' '
        << setw(fw) << 0.0 << ' '
        << setw(fw) << 1.0 << ' '
        << ']' << endl;
    return out;
}

/******************************************************************************
 *      METHOD: ==operator
 * DESCRIPTION: compares two transforms and returns true if all values of the
 *              first transform matrix are within the tolerance of the second
 *              transform matrix values 
 *****************************************************************************/
bool operator==(const RecTransform2D& a, const RecTransform2D& b)
{
    return( (fabs(a.matrixPP[0][0] - b.matrixPP[0][0]) < RecTransform2D::tolerance) &&
            (fabs(a.matrixPP[0][1] - b.matrixPP[0][1]) < RecTransform2D::tolerance) &&
            (fabs(a.matrixPP[0][2] - b.matrixPP[0][2]) < RecTransform2D::tolerance) &&
            (fabs(a.matrixPP[1][0] - b.matrixPP[1][0]) < RecTransform2D::tolerance) &&
            (fabs(a.matrixPP[1][1] - b.matrixPP[1][1]) < RecTransform2D::tolerance) &&
            (fabs(a.matrixPP[1][2] - b.matrixPP[1][2]) < RecTransform2D::tolerance) );
}

/******************************************************************************
 *      METHOD: ==operator
 * DESCRIPTION: compares two transforms and returns false if all values of the
 *              first transform matrix are within the tolerance of the second
 *              transform matrix values 
 *****************************************************************************/
bool operator!=(const RecTransform2D& a, const RecTransform2D& b)
{
	return !(a == b);
}
