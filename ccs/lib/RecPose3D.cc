
/*****************************************************************************
 * NOTES

    Note that the default Euler Angle Order is ZYX, as defined in the constructors 
    for this class in recGeometry.h (they use ZYX when EulerAngleOrder isn't specified)

    This means that the pose MUST be constructed as such when using the default:

        RecPose3D Pose( X, Y, Z, Yaw, Pitch, Roll);

    This is equivalent to:

        RecPose3D Pose( X, Y, Z, Yaw, Pitch, Roll, RecPose3D::ZYX );

    While this ordering of angles isn't what we would typically use (roll-pitch-yaw
    just rolls off the tongue better), it follows the SAE coordinate system.  This prescribes
    that when transforming points, we rotate about the z axis (yaw), then the new Y axis 
    (body pitch), then about the doubly new X axis (body roll).  X is considered forward, 
    Y is to the right and Z is down.

    A key advantage of this ordering is that the pose angles are simply the yaw, pitch,
    and roll relative to the original coordinate frame and no additional calcs are needed.

    Example:  A sensor faces 0.2 radians to the right, is tilted down 0.1 radians, and is
    tilted 0.4 radians CCW (when viewed from inside the vehicle) relative to the vehicle 
    body.  In addition, the sensor is 1 meter forward, 0.5 meters to the right and 0.25 
    meters above the vehicle origin. To represent this pose, use the following line:

        RecPose3D sensorPose( 1.0, 0.5, -0.25, 0.2, -0.1, -0.4, RecPose3D::ZYX);

    The Euler Angle Order is specified here just in case the defaults change one day.

    To develop a transform to convert the sensor points to vehicle coord points we use:

        RecTransform sensorToVehicle( sensorPose );

    Applying this to a point from the sensor:
       
        VehiclePoint = sensorToVehicle*SensorPoint;

    Returns that point in the Vehicle Coordinate Frame

 *****************************************************************************/

#include "RecGeometry.h"

//#include <iomanip>
//#include <assert.h>
//#include <memory.h>
//#include <stdio.h>

#ifndef PRINT_DEGREES
#define PRINT_DEGREES 1
#endif

 
/*****************************************************************************
 * RecPose3D Functions
 *****************************************************************************/
/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecPose3D
 * DESCRIPTION: no argument constructor, all fields to zero.  This function 
 *              takes an optional argument for the pose's Euler angle order.
 *              If no angle is specified, the default, specified in 
 *              recGeometry.h, will be used.  
 *****************************************************************************/
RecPose3D::RecPose3D(EulerAngleOrder order)
                    :x(0.0), y(0.0), z(0.0),
                     rot1(0.0), rot2(0.0), rot3(0.0), angleOrder(order) { } 

/******************************************************************************
 *      METHOD: RecPose3D
 * DESCRIPTION: six argument constructor, sets x, y, z, rot1, rot2 and and rot3
 *              to passed values.  This function takes an optional argument for
 *              the pose's Euler angle order.  If no angle is specified, the 
 *              default, specified in recGeometry.h, will be used.  
 *****************************************************************************/
RecPose3D::RecPose3D(double ix, double iy, double iz, RecRadians r1, RecRadians r2, RecRadians r3, EulerAngleOrder order)
                    :x(ix), y(iy), z(iz),
                     rot1(r1), rot2(r2), rot3(r3), angleOrder(order) { } 

/******************************************************************************
 *      METHOD: RecPose3D
 * DESCRIPTION: copy constructor, copys the fields of the passed pose
 *****************************************************************************/
RecPose3D::RecPose3D(const RecPose3D& b)
                    :x(b.x), y(b.y), z(b.z),
                     rot1(b.rot1), rot2(b.rot2), rot3(b.rot3), angleOrder(b.angleOrder) { }

/******************************************************************************
 *      METHOD: RecPose3D
 * DESCRIPTION: transform constructor, converts a 3D Transform to a 3D pose.
 *              The rotations returned are those specified by the angle order
 *              parameter.  If no angle order is specified, the default ,
 *              specified in recGeometry.h, is used.
 *****************************************************************************/
RecPose3D::RecPose3D(const RecTransform3D& trans, EulerAngleOrder order)
                    :x(trans.matrixPP[0][3]), y(trans.matrixPP[1][3]), z(trans.matrixPP[2][3]), angleOrder(order)
{
    double sb;
	double cb;
    double k1;

	switch(order) 
	{
    case XYX:
        k1 = sqrt((trans.matrixPP[0][1]*trans.matrixPP[0][1]) + (trans.matrixPP[0][2]*trans.matrixPP[0][2]));
    
        rot2 = atan2(k1, trans.matrixPP[0][0]);
        sb = sin(rot2);
    
        if(sb == 0.0)
        {
            rot1 = 0.0;
            rot3 = atan2(-trans.matrixPP[1][2], trans.matrixPP[1][1]);
        }
        else
        {
            rot1 = atan2((trans.matrixPP[1][0]/sb), (-trans.matrixPP[2][0]/sb));
            rot3 = atan2((trans.matrixPP[0][1]/sb), (trans.matrixPP[0][2]/sb));
        }
		break;

    case XYZ:
        k1 = sqrt((trans.matrixPP[0][0]*trans.matrixPP[0][0]) + (trans.matrixPP[0][1]*trans.matrixPP[0][1]));
    
        rot2 = atan2(trans.matrixPP[0][2], k1);
        cb = cos(rot2);
    
        if (cb == 0.0)
        {
            rot1 = 0.0;
            rot3 = atan2(trans.matrixPP[1][0], trans.matrixPP[1][1]);
        }
        else
        {
            rot1 = atan2((-trans.matrixPP[1][2]/cb), (trans.matrixPP[2][2]/cb));
            rot3 = atan2((-trans.matrixPP[0][1]/cb), (trans.matrixPP[0][0]/cb));
        }
		break;

    case XZX:
        k1 = sqrt((trans.matrixPP[0][1]*trans.matrixPP[0][1]) + (trans.matrixPP[0][2]*trans.matrixPP[0][2]));
    
        rot2 = atan2(k1, trans.matrixPP[0][0]);
        sb = sin(rot2);
    
        if(sb == 0.0)
        {
            rot1 = 0.0;
            rot3 = atan2(trans.matrixPP[2][1], trans.matrixPP[2][2]);
        }
        else
        {
            rot1 = atan2((trans.matrixPP[2][0]/sb), (trans.matrixPP[1][0]/sb));
            rot3 = atan2((trans.matrixPP[0][2]/sb), (-trans.matrixPP[0][1]/sb));
        }
		break;

	case XZY:
        k1 = sqrt((trans.matrixPP[0][0]*trans.matrixPP[0][0]) + (trans.matrixPP[0][2]*trans.matrixPP[0][2]));
    
        rot2 = atan2(-trans.matrixPP[0][1], k1);
        cb = cos(rot2);
    
        if (cb == 0.0)
        {
            rot1 = 0.0;
            rot3 = atan2(-trans.matrixPP[2][0], trans.matrixPP[2][2]);
        }
        else
        {
            rot1 = atan2((trans.matrixPP[2][1]/cb), (trans.matrixPP[1][1]/cb));
            rot3 = atan2((trans.matrixPP[0][2]/cb), (trans.matrixPP[0][0]/cb));
        }
		break;

	case YXY:
        k1 = sqrt((trans.matrixPP[1][0]*trans.matrixPP[1][0]) + (trans.matrixPP[1][2]*trans.matrixPP[1][2]));
    
        rot2 = atan2(k1, trans.matrixPP[1][1]);
        sb = sin(rot2);
    
        if(sb == 0.0)
        {
            rot1 = 0.0;
            rot3 = atan2(-trans.matrixPP[0][2], trans.matrixPP[0][0]);
        }
        else
        {
            rot1 = atan2((trans.matrixPP[0][1]/sb), (trans.matrixPP[2][1]/sb));
            rot3 = atan2((trans.matrixPP[1][0]/sb), (-trans.matrixPP[1][2]/sb));
        }
		break;

	case YXZ:
        k1 = sqrt((trans.matrixPP[1][0]*trans.matrixPP[1][0]) + (trans.matrixPP[1][1]*trans.matrixPP[1][1]));
    
        rot2 = atan2(-trans.matrixPP[1][2], k1);
        cb = cos(rot2);
    
        if (cb == 0.0)
        {
            rot1 = 0.0;
            rot3 = atan2(-trans.matrixPP[0][1], trans.matrixPP[0][0]);
        }
        else
        {
            rot1 = atan2((trans.matrixPP[0][2]/cb), (trans.matrixPP[2][2]/cb));
            rot3 = atan2((trans.matrixPP[1][0]/cb), (trans.matrixPP[1][1]/cb));
        }
		break;

	case YZX:
        k1 = sqrt((trans.matrixPP[1][1]*trans.matrixPP[1][1]) + (trans.matrixPP[1][2]*trans.matrixPP[1][2]));
    
        rot2 = atan2(trans.matrixPP[1][0], k1);
        cb = cos(rot2);
    
        if (cb == 0.0)
        {
            rot1 = 0.0;
            rot3 = atan2(trans.matrixPP[2][1], trans.matrixPP[2][2]);
        }
        else
        {
            rot1 = atan2((-trans.matrixPP[2][0]/cb), (trans.matrixPP[0][0]/cb));
            rot3 = atan2((-trans.matrixPP[1][2]/cb), (trans.matrixPP[1][1]/cb));
        }
		break;

	case YZY:
        k1 = sqrt((trans.matrixPP[1][0]*trans.matrixPP[1][0]) + (trans.matrixPP[1][2]*trans.matrixPP[1][2]));
		
		rot2 = atan2(k1, trans.matrixPP[1][1]);
		sb = sin(rot2);
		
		if(sb == 0.0)
		{
			rot1 = 0.0;
			rot3 = atan2(-trans.matrixPP[2][0], trans.matrixPP[2][2]);
		}
		else
		{
			rot1 = atan2((trans.matrixPP[2][1]/sb), (-trans.matrixPP[0][1]/sb));
			rot3 = atan2((trans.matrixPP[1][2]/sb), (trans.matrixPP[1][0]/sb));
		}
		break;

	case ZXY:
        k1 = sqrt((trans.matrixPP[2][0]*trans.matrixPP[2][0]) + (trans.matrixPP[2][2]*trans.matrixPP[2][2]));
    
        rot2 = atan2(trans.matrixPP[2][1], k1);
        cb = cos(rot2);

        if (cb == 0.0)
        {
            rot1 = 0.0;
            rot3 = atan2(trans.matrixPP[0][2], trans.matrixPP[0][0]);
        }
        else
        {
            rot1 = atan2((-trans.matrixPP[0][1]/cb), (trans.matrixPP[1][1]/cb));
            rot3 = atan2((-trans.matrixPP[2][0]/cb), (trans.matrixPP[2][2]/cb));
        }
		break;

	case ZXZ:
        k1 = sqrt((trans.matrixPP[2][0]*trans.matrixPP[2][0]) + (trans.matrixPP[2][1]*trans.matrixPP[2][1]));
		
		rot2 = atan2(k1, trans.matrixPP[2][2]);
		sb = sin(rot2);
		
		if(sb == 0.0)
		{
			rot1 = 0.0;
			rot3 = atan2(-trans.matrixPP[0][1], trans.matrixPP[0][0]);
		}
		else
		{
			rot1 = atan2((trans.matrixPP[0][2]/sb), (-trans.matrixPP[1][2]/sb));
			rot3 = atan2((trans.matrixPP[2][0]/sb), (trans.matrixPP[2][1]/sb));
		}
		break;

	case ZYX:
        k1 = sqrt((trans.matrixPP[0][0]*trans.matrixPP[0][0]) + (trans.matrixPP[1][0]*trans.matrixPP[1][0]));

		rot2 = atan2(-trans.matrixPP[2][0], k1);
		cb = cos(rot2);

		if (cb == 0.0) 
		{
			rot1 = 0.0;
			rot3 = atan2(-trans.matrixPP[1][2], trans.matrixPP[1][1]);
		} 
		else 
		{
			rot1 = atan2((trans.matrixPP[1][0]/cb), (trans.matrixPP[0][0]/cb));
			rot3 = atan2((trans.matrixPP[2][1]/cb), (trans.matrixPP[2][2]/cb));
		}
		break;

	case ZYZ:
        k1 = sqrt((trans.matrixPP[2][0]*trans.matrixPP[2][0]) + (trans.matrixPP[2][1]*trans.matrixPP[2][1]));
		
		rot2 = atan2(k1, trans.matrixPP[2][2]);
		sb = sin(rot2);
		
		if(sb == 0.0)
		{
			rot1 = 0.0;
			rot3 = atan2(trans.matrixPP[1][0], trans.matrixPP[1][1]);
		}
		else
		{
			rot1 = atan2((trans.matrixPP[1][2]/sb), (trans.matrixPP[0][2]/sb));
			rot3 = atan2((trans.matrixPP[2][1]/sb), (-trans.matrixPP[2][0]/sb));
		}
		break;

	default:
		cout << "Error in pose constructor: invalid angle order for pose\n";
		break;
	}

}

/******************************************************************************
 *      METHOD: inverse
 * DESCRIPTION: computes and returns the inverse of a pose by converting the
 *              pose to a transform, taking the inverse of this transform and
 *              converting the inverted transform into a 3D pose.
 *****************************************************************************/
RecPose3D RecPose3D::inverse(void)
{
    RecTransform3D trans(*this);
    RecPose3D pose(trans.inverse());
    return pose;
}


/**\brief Accessor to project to a 3DOF planar pose
 * \return The equivalent planar pose
 *
 * Note: the Euler angle enumeration is crazy, and we only handle the ZYX (default) case right now
 */
bool RecPose3D::getPlanarPose(RecPose2D &pose) const
{
    if(angleOrder != ZYX)
    {
        std::cerr<<"RecPose3D: Warning: getPlanarPose does not support EulerAngle Order "<<angleOrder<<std::endl;
        return false;
    }
    pose.x = this->x;
    pose.y = this->y;
    pose.rotZ = this->rot1;
    return true;
}


/******************************************************************************
 *      METHOD: operator+=
 * DESCRIPTION: adds the passed differential pose to the pose  
 *****************************************************************************/
void RecPose3D::operator+=(const RecDifferentialPose3D& b) 
{
	x += b.x; 
	y += b.y; 
	z += b.z; 
    rot1 += b.rot1;
	rot2 += b.rot2; 
	rot3 += b.rot3;
}

/******************************************************************************
 *      METHOD: operator-=
 * DESCRIPTION: subtracts the passed differential pose from the pose  
 *****************************************************************************/
void RecPose3D::operator-=(const RecDifferentialPose3D& b) 
{
	x -= b.x; 
	y -= b.y; 
	z -= b.z;
    rot1 -= b.rot1;
	rot2 -= b.rot2; 
	rot3 -= b.rot3;
}

/******************************************************************************
 *      METHOD: operator=
 * DESCRIPTION: assigns one pose to another by copying all fields
 *****************************************************************************/
void RecPose3D::operator=(const RecPose3D& b) 
{
	x = b.x; 
	y = b.y; 
	z = b.z; 
    rot1 = b.rot1;
	rot2 = b.rot2; 
	rot3 = b.rot3;
	angleOrder = b.angleOrder;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this pose with the passed pose
 *****************************************************************************/
void RecPose3D::operator*=(const RecPose3D& b) 
{
    RecTransform3D transThis(*this);
    RecTransform3D transb(b);
    RecTransform3D transa(transThis*transb);
    RecPose3D a(transa);
    (*this) = a;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this pose with the passed transform
 *****************************************************************************/
void RecPose3D::operator*=(const RecTransform3D& b) 
{
    RecTransform3D transThis(*this);
    RecTransform3D transa(transThis*b);
    RecPose3D a(transa);
    (*this) = a;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this pose with the passed quaternion
 *****************************************************************************/
void RecPose3D::operator*=(const RecQuaternion& b) 
{
    RecTransform3D transThis(*this);
    RecTransform3D transb(b);
    RecTransform3D transa(transThis*transb);
    RecPose3D a(transa);
    (*this) = a;
}


/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: returns the pose that is the concatenation of the two passed
 *              poses
 *****************************************************************************/
RecPose3D operator*(const RecPose3D& b, const RecPose3D& c)
{
    RecTransform3D transb(b);
    RecTransform3D transc(c);
    RecTransform3D transa(transb*transc);
    RecPose3D a(transa);
    return a;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: transforms the passed point by the passed pose and returns
 *              the transformed point
 *****************************************************************************/
RecPoint3D operator*(const RecPose3D& pose, const RecPoint3D& pt)
{
    RecTransform3D t(pose);
    return(t*pt);
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: transforms the 2D passed point by the passed pose and returns
 *              the transformed point
 *****************************************************************************/
RecPoint2D operator*(const RecPose3D& pose, const RecPoint2D& pt)
{
    RecTransform3D t(pose);
    return(t*pt);
}

/******************************************************************************
 *      METHOD: operator+
 * DESCRIPTION: adds the passed differential pose to the passed pose and 
 *              returns the result  
 *****************************************************************************/
RecPose3D operator+(const RecPose3D& a, const RecDifferentialPose3D& b)
{
    RecPose3D c(a);
    c += b;
    return c;
}

/******************************************************************************
 *      METHOD: operator+
 * DESCRIPTION: adds the passed differential pose to the passed pose and 
 *              returns the result  
 *****************************************************************************/
RecPose3D operator+(const RecDifferentialPose3D& a, const RecPose3D& b)
{
    return b+a;
}

/******************************************************************************
 *      METHOD: operator-
 * DESCRIPTION: subtracts the passed differential pose from the passed pose and 
 *              returns the result  
 *****************************************************************************/
RecPose3D operator-(const RecPose3D& a, const RecDifferentialPose3D& b)
{
    RecPose3D c(a);
    c -= b;
    return c;
}

/******************************************************************************
 *      METHOD: operator==
 * DESCRIPTION: converts the two poses to transforms and returns the result
 *              of transform comparison
 *****************************************************************************/
bool operator==(const RecPose3D& a, const RecPose3D& b)
{
	return(RecTransform3D(a) == RecTransform3D(b));
}

/******************************************************************************
 *      METHOD: operator!=
 * DESCRIPTION: returns the complement of the == operator
 *****************************************************************************/
bool operator!=(const RecPose3D& a, const RecPose3D& b)
{
	return(!(a==b));
}

/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: puts the pose with nice formatting to the passed output stream
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecPose3D& a)
{
    if(PRINT_DEGREES)
    {
        return out << "(x,y,z,r1d,r2d,r3d) = ( " << a.x << ", " << a.y << ", "
                   << a.z << ", " << a.rot1.getDegrees() << ", " << a.rot2.getDegrees()
                   << ", " << a.rot3.getDegrees() << " )";
    }
    else
    {
        return out << "(x,y,z,r1,r2,r3) = ( " << a.x << ", " << a.y << ", "
                   << a.z << ", " << a.rot1 << ", " << a.rot2 << ", " << a.rot3 << " )";
    }
}



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*****************************************************************************
 * RecDifferentialPose3D Functions
 *****************************************************************************/
/*****************************************************************************
 * CLASS CONSTANT
 *****************************************************************************/
const double RecDifferentialPose3D::tolerance = 10e-6;
/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecDifferentialPose3D
 * DESCRIPTION: no argument constructor, all fields to zero.
 *****************************************************************************/
RecDifferentialPose3D::RecDifferentialPose3D(void)
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    rot1 = 0.0;
    rot2 = 0.0;
    rot3 = 0.0;
}

/******************************************************************************
 *      METHOD: RecDifferentialPose3D
 * DESCRIPTION: six argument constructor, sets x, y, z, rot1, rot2 and and rot3
 *              to passed values.  
 *****************************************************************************/
RecDifferentialPose3D::RecDifferentialPose3D(double inx, double iny, 
                                             double inz, RecRadians r1, 
                                             RecRadians r2, RecRadians r3)
{
    x = inx;
    y = iny;
    z = inz;
    rot1 = r1;
    rot2 = r2;
    rot3 = r3;
}

/******************************************************************************
 *      METHOD: RecDifferentialPose3D
 * DESCRIPTION: Copy constructor, copys all fields of the passed pose 
 *****************************************************************************/
RecDifferentialPose3D::RecDifferentialPose3D(const RecDifferentialPose3D& b)
{
    (*this) = b;
} 

/******************************************************************************
 *      METHOD: operator+=
 * DESCRIPTION: adds the passed differential pose to the differential pose  
 *****************************************************************************/
void RecDifferentialPose3D::operator+=(const RecDifferentialPose3D& b) 
{
	x += b.x; 
	y += b.y; 
	z += b.z; 
    rot1 += b.rot1;
	rot2 += b.rot2; 
	rot3 += b.rot3;
}

/******************************************************************************
 *      METHOD: operator-=
 * DESCRIPTION: subtracts the passed differential pose from the 
 *              differential pose  
 *****************************************************************************/
void RecDifferentialPose3D::operator-=(const RecDifferentialPose3D& b) 
{
	x -= b.x; 
	y -= b.y; 
	z -= b.z;
    rot1 -= b.rot1;
	rot2 -= b.rot2; 
	rot3 -= b.rot3;
}

/******************************************************************************
 *      METHOD: operator=
 * DESCRIPTION: assigns one differential pose to another by copying all fields
 *****************************************************************************/
void RecDifferentialPose3D::operator=(const RecDifferentialPose3D& b) 
{
	x = b.x; 
	y = b.y; 
	z = b.z; 
    rot1 = b.rot1;
	rot2 = b.rot2; 
	rot3 = b.rot3;
}

/******************************************************************************
 *      METHOD: operator+
 * DESCRIPTION: adds the passed differential poses and 
 *              returns the result  
 *****************************************************************************/
RecDifferentialPose3D operator+(const RecDifferentialPose3D& a, const RecDifferentialPose3D& b)
{
    RecDifferentialPose3D c(a);
    c += b;
    return c;
}

/******************************************************************************
 *      METHOD: operator-
 * DESCRIPTION: subtracts the passed differential poses and 
 *              returns the result  
 *****************************************************************************/
RecDifferentialPose3D operator-(const RecDifferentialPose3D& a, const RecDifferentialPose3D& b)
{
    RecDifferentialPose3D c(a);
    c -= b;
    return c;
}

/******************************************************************************
 *      METHOD: operator==
 * DESCRIPTION: returns true if all fields of the 2 differential poses are
 *              approximately the same
 *****************************************************************************/
bool operator==(const RecDifferentialPose3D& a, const RecDifferentialPose3D& b)
{
    return ((fabs(a.x - b.x) <= RecDifferentialPose3D::tolerance) &&
            (fabs(a.y - b.y) <= RecDifferentialPose3D::tolerance) &&
            (fabs(a.z - b.z) <= RecDifferentialPose3D::tolerance) &&
            (fabs(a.rot1 - b.rot1) <= RecDifferentialPose3D::tolerance) &&
            (fabs(a.rot2 - b.rot2) <= RecDifferentialPose3D::tolerance) &&
            (fabs(a.rot3 - b.rot3) <= RecDifferentialPose3D::tolerance));
}

/******************************************************************************
 *      METHOD: operator!=
 * DESCRIPTION: returns the complement of the == operator
 *****************************************************************************/
bool operator!=(const RecDifferentialPose3D& a, const RecDifferentialPose3D& b)
{
	return(!(a==b));
}

/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: puts the pose with nice formatting to the passed output stream
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecDifferentialPose3D& a)
{
    return out << "(x,y,z,r1,r2,r3) = ( " << a.x << ", " << a.y << ", "
               << a.z << ", " << a.rot1 << ", " << a.rot2 << ", " << a.rot3 << " )";
}






