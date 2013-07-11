#ifndef _RECGEOMETRY_H_
#define _RECGEOMETRY_H_


#include <iostream>
#include <vector>
#include <list>
#include <math.h>

using namespace std;

#ifndef PI
#define PI 3.14159265358979323846
#endif


/*****************************************************************************
 * HEADER CLASSES (included in this file)
 *****************************************************************************/
class RecRadians;             //1
class RecPoint2D;             //2
class RecPoint3D;             //3
class RecVector2D;            //4
class RecVector3D;            //5
class RecPose2D;              //6
class RecDifferentialPose2D;  //7
class RecPose3D;              //8
class RecDifferentialPose3D;  //9
class RecTransform2D;         //10
class RecTransform3D;         //11
class RecQuaternion;          //12
class RecLineSegment2D;       //13
class RecLineSegment3D;       //14
class RecBox2D;               //15
class RecAxisAlignedBox2D;    //16
class RecBox3D;               //17
class RecAxisAlignedBox3D;    //18
class RecPolygon2D;           //19
//class RecPolygon3D;         //*20



//*****************************************************************************
/*!
* \brief A radians class that enforces -PI < value <= PI
*
* A RecRadians object is designed to look as much like a double value as
* possible.  The object allows for direct casting to/from a double value
* and supports all of the arithmetic operators.  The object always inforces
* the the radian value is scaled between -PI and PI.  This combined with the
* support for automatic casting to doubles allows for a RecRadian to be passed
* as an argument to any math.h trig function that takes a radian argument.
* @ingroup recGeometryGroup
*/
//*****************************************************************************
class RecRadians
{
public:
    // Static Member
	static const double tolerance; //!< tolerance for which two RecRadians objects are considered equal

    // Constructor
	RecRadians(void); //!< default constructor which sets radian value to zero
    RecRadians(double rad); //!< constructs a RecRadians object from a double.  Scales the double (-PI ~ PI) when necessary.
	RecRadians(const RecRadians& rad); //!< copy constructor

    // Operator Overload
    operator double() const;  //!< casting a radian to a double
    void operator+=(const RecRadians& b); //!< angle addition
    void operator+=(double b); //!< angle addition
    void operator-=(const RecRadians& b); //!< angle subtraction
    void operator-=(double b); //!< angle subtraction
    void operator*=(double b); //!< multiplication of angle by a scalar
    void operator/=(double b); //!< division of angle by a scalar
    void operator=(const RecRadians& b); //!< assignment operator with RecRadians argument
    void operator=(double b); //!< assignment operator with double argument

    friend RecRadians operator+(const RecRadians& a, const RecRadians& b); //!< angle addition
    friend RecRadians operator+(const RecRadians& a, double b); //!< angle addition
    friend RecRadians operator+(double a, const RecRadians& b); //!< angle addition
    friend RecRadians operator-(const RecRadians& a, const RecRadians& b); //!< angle subtraction
    friend RecRadians operator-(const RecRadians& a, double b); //!< angle subtraction
    friend RecRadians operator-(double a, const RecRadians& b); //!< angle subtraction
    friend RecRadians operator*(const RecRadians& a, double b); //!< multiplication of angle by a scalar
    friend RecRadians operator*(double a, const RecRadians& b); //!< multiplication of angle by a scalar
    friend RecRadians operator/(const RecRadians& a, double b); //!< division of angle by a scalar

    friend bool operator==(const RecRadians& a, const RecRadians& b); //!< Returns true when the magnitude of the difference between the two radian values is less than or equal to the object's tolerance
    friend bool operator!=(const RecRadians& a, const RecRadians& b); //!< Returns true when the magnitude of the difference between the two radian values is greater than the object's tolerance
	friend bool operator>(const RecRadians& a, const RecRadians& b); //!< Returns true if the first argument is strictly greater than the second argument
    friend bool operator>=(const RecRadians& a, const RecRadians& b); //!< Returns true if the first argument is greater than or equal to the second argument
	friend bool operator<(const RecRadians& a, const RecRadians& b); //!< Returns true if the first argument is strictly less than the second argument
    friend bool operator<=(const RecRadians& a, const RecRadians& b); //!< Returns true if the first argument is less than or equal to the second argument

	// Stream Operators
    friend std::ostream& operator<<(std::ostream& out, const RecRadians& a); //!< writes the radian object to the passed stream

	// other methods
	double getDegrees() const; //!< converts this object's radian value to degrees and returns the result
    void setDegrees(double deg); //!< converts the passed argument from degrees to radians and sets the object's value to the result

protected:
	// Member Variables
	double radians; //!< The objects radian value
	void scaleValue(); //!< method which scales the object's radian value between -PI and PI
};



//*****************************************************************************
/*!
* \brief A 2D point
*
* A RecPoint2D object represents a point in 2D cartesian space.  Basic linear
* algebraic operations are defined for adding/subtracting and multiplying/dividing
* by a scalar.  RecPoint2D objects can also be operated upon by 2D transforms
* and poses to produce new 2D points.
* @ingroup recGeometryGroup
*/
//*****************************************************************************
class RecPoint2D
{
public:
    double x; //!< The x coordinate of the point
    double y; //!< The y coordinate of the point

	// Object wide constant
	static const double tolerance; //!< tolerance for which two RecPoint2D objects are considered equal

    // Constructor
	RecPoint2D(void); //!< Default Constructor, both the x and y coordinates are set to zero
    RecPoint2D(double inx, double iny); //!< Sets the x and y coordinates to the passed x and y values
    RecPoint2D(const RecPoint2D& pt2D); //!< Copy Constructor

    // Operator Overload
    void operator+=(const RecPoint2D& b); //!< Point Addition
    void operator-=(const RecPoint2D& b); //!< Point Subtraction

    void operator+=(const RecVector2D& v); //!< Adding a vector to a point
    void operator-=(const RecVector2D& v); //!< Adding a vector to a point

    void operator/=(double b); //!< Division by a scalar
    void operator*=(double b); //!< Multiplication by a scalar
    void operator=(const RecPoint2D& b); //!< Point Assignment

	friend RecPoint2D operator/(const RecPoint2D& a, double b); //!< Division by a scalar
    friend RecPoint2D operator*(const RecPoint2D& a, double b); //!< Multiplication by a scalar
	friend RecPoint2D operator*(double a, const RecPoint2D& b); //!< Multiplication by a scalar
    friend RecPoint2D operator+(const RecPoint2D& a, const RecPoint2D& b); //!< Point Addition
    friend RecPoint2D operator+(const RecPoint2D& a, ///< adding a vector to a point
                                const RecVector2D& v);

    friend RecPoint2D operator-(const RecPoint2D& a,
                                const RecPoint2D& b); //!< Point Subtraction

    friend RecPoint2D operator-(const RecPoint2D& a, ///< subtracting a vector from a point
                                const RecVector2D& v);

    friend bool operator==(const RecPoint2D& a, const RecPoint2D& b); //!< Point Comparison
    friend bool operator!=(const RecPoint2D& a, const RecPoint2D& b); //!< Point Comparison

    // Other Methods
	double distance(const RecPoint2D& a) const; //!< Returns the cartesian distance between this point and the passed argument
	double distanceSq(const RecPoint2D& a) const; //!< Returns the square of the cartesian distance between this point and the passed argument

    // Stream Operators
    friend std::ostream& operator<<(std::ostream& out, const RecPoint2D& a);//!< writes the 2D point object to the passed stream
};





//*****************************************************************************
/*!
* \brief A 3D point
*
* A RecPoint3D object represents a point in 3D cartesian space.  Basic linear
* algebraic operations are defined for adding/subtracting and multiplying/dividing
* by a scalar.  RecPoint3D objects can also be operated upon by 3D transforms,
* poses and quaternions to produce new 3D points.
* @ingroup recGeometryGroup
*/
//*****************************************************************************
class RecPoint3D
{
public:
    double x; //!< The x coordinate of the point
    double y; //!< The y coordinate of the point
    double z; //!< The z coordinate of the point

	// Object wide constant
	static const double tolerance; //!< tolerance for which two RecPoint3D objects are considered equal

    // Constructor
    RecPoint3D(void); //!< Default Constructor, the x, y and z coordinates are set to zero
    RecPoint3D(double inx, double iny, double inz); //!< Sets the x, y and z coordinates to the passed x, y and z values
    RecPoint3D(const RecPoint2D& b); //!< Converts a 2D point to a 3D one by setting the z coordinate of the 3D point to zero.
    RecPoint3D(const RecPoint3D& b); //!< Copy Constructor

    // Operator Overload
    void operator+=(const RecPoint3D& b); //!< Point Addition
    void operator-=(const RecPoint3D& b); //!< Point Subtraction
    void operator*=(double b); //!< Multiplication by scalar
    void operator/=(double b); //!< Division by scalar
    void operator=(const RecPoint3D& b); //!< Point Assignment

	friend RecPoint3D operator/(const RecPoint3D& a, double b); //!< Division by scalar
    friend RecPoint3D operator*(const RecPoint3D& a, double b); //!< Multiplication by scalar
    friend RecPoint3D operator*(double a, const RecPoint3D& b); //!< Multiplication by scalar
    friend RecPoint3D operator+(const RecPoint3D& a, const RecPoint3D& b); //!< Point Addition
    friend RecPoint3D operator-(const RecPoint3D& a, const RecPoint3D& b); //!< Point Subtraction

    friend bool operator==(const RecPoint3D& a, const RecPoint3D& b); //!< Point Comparison
    friend bool operator!=(const RecPoint3D& a, const RecPoint3D& b); //!< Point Comparison

    // Other Methods
	double distance(const RecPoint3D& a) const; //!< Returns the cartesian distance between this point and the passed argument
	double distanceSq(const RecPoint3D& a) const; //!< Returns the square of the cartesian distance between this point and the passed argument

    // Stream Operators
    friend std::ostream& operator<<(std::ostream& out, const RecPoint3D& a);//!< writes the 3D point object to the passed stream
};




//*****************************************************************************
/*!
* \brief A 2D vector
*
* A RecVector2D object represents a vector in 2D cartesian space.  Basic linear
* algebraic operations are defined for adding/subtracting and multiplying/dividing
* by a scalar.  RecVector2D objects can also be operated upon by 2D transforms
* and poses to produce new 2D vectors.  The RecVector2D also defines methods for
* normalizing, computing length and taking dot products.
* @ingroup recGeometryGroup
*/
//*****************************************************************************
class RecVector2D
{
public:
    double x;
    double y;

    // Constructor
    RecVector2D(void); //!< Default Constructor, both the x and y coordinates are set to zero
    RecVector2D(double inx, double iny); //!< Sets the x and y coordinates to the passed x and y values
    RecVector2D(const RecVector2D& v2D); //!< Copy Constructor
    RecVector2D(const RecPoint2D& pt2D); //!< Converts a 2D point object to a 2D vector object

    // Operator Overload
    void operator+=(const RecVector2D& b); //!< Vector Addition
    void operator-=(const RecVector2D& b); //!< Vector Subtraction
    void operator/=(double b); //!< Division by a scalar
    void operator*=(double b); //!< Multiplication by a scalar
    void operator=(const RecVector2D& b); //!< Vector Assignment
    friend RecVector2D operator/(const RecVector2D& a, double b); //!< Division by scalar
    friend RecVector2D operator*(const RecVector2D& a, double b); //!< Multiplication by scalar
    friend RecVector2D operator*(double a, const RecVector2D& b); //!< Multiplication by scalar
    friend RecVector2D operator+(const RecVector2D& a, const RecVector2D& b); //!< Vector Addition
    friend RecVector2D operator-(const RecVector2D& a, const RecVector2D& b); //!< Vector Subtraction
    friend bool operator==(const RecVector2D& a, const RecVector2D& b); //!< Point Comparison
    friend bool operator!=(const RecVector2D& a, const RecVector2D& b); //!< Point Comparison

    friend double operator*(const RecVector2D& a, const RecVector2D& b); //!< Inner Dot Product of Vectors

    // Other Methods
    void normalize(void); //!< Normalizes the vector by keeping the direction of the vector constant while giving the vector unit magnitude
    double length(void) const; //!< Returns the magnitude of the vector
    double lengthSq(void) const; //!< Returns the square of the magnitude of the vector
    double dot(const RecVector2D& a) const; //!< computes the dot product of this vector with the passed vector
    double xprod(const RecVector2D & a) const;//!< computes the cross product of this vector with the passed vector
    RecVector2D normal(void) const; ///< returns a vector rotated by 90 degrees counter clockwise

    friend std::ostream & operator<<(std::ostream & out, const RecVector2D&a);///<writes the vector to the passed stream

    ///\brief static utility to build a unit vector along an angle
    static RecVector2D getUnitVector(const RecRadians& radians);
};




//*****************************************************************************
/*!
* \brief A 3D vector
*
* A RecVector3D object represents a vector in 3D cartesian space.  Basic linear
* algebraic operations are defined for adding/subtracting and multiplying/dividing
* by a scalar.  RecVector3D objects can also be operated upon by 3D transforms,
* poses and quaternions to produce new 3D vectors.  The RecVector3D also defines
* methods for normalizing, computing length and taking dot and cross products.
* @ingroup recGeometryGroup
*/
//*****************************************************************************
class RecVector3D:public RecPoint3D
{
public:
    // Constructor
    RecVector3D(void); //!< Default Constructor, the x, y and z coordinates are set to zero
    RecVector3D(double inx, double iny, double inz); //!< Sets the x, y and z coordinates to the passed x, y and z values
    RecVector3D(const RecPoint2D& b); //!< Converts a 2D point to a 3D vector by setting the z coordinate of the 3D point to zero
    RecVector3D(const RecPoint3D& b); //!< Converts a 3D point to a 3D vector
	RecVector3D(const RecVector2D& v2D); //!< Converts a 2D vector to a 3D vector by setting the z coordinate of the 3D point to zero
    RecVector3D(const RecVector3D& v3D); //!< Copy Constructor

    // Operator Overload
    void operator^=(const RecVector3D& a); //!< Vector Outer Cross Product
    void operator=(const RecVector3D& b); //!< Vector Assignment

    friend RecVector3D operator/(const RecVector3D& a, double b); //!< Division by scalar
    friend RecVector3D operator*(const RecVector3D& a, double b); //!< Multiplication by scalar
    friend RecVector3D operator*(double a, const RecVector3D& b); //!< Multiplication by scalar
    friend RecVector3D operator+(const RecVector3D& a, const RecVector3D& b); //!< Vector Addition
    friend RecVector3D operator-(const RecVector3D& a, const RecVector3D& b); //!< Vector Subtraction

    friend double operator*(const RecVector3D& a, const RecVector3D& b); //!< Vector Inner Dot Product
    friend RecVector3D operator^(const RecVector3D& a, const RecVector3D& b); //!< Vector Cross Product

    // Other Methods
    void normalize(void);  //!< Normalizes the vector by keeping the direction of the vector constant while giving the vector unit magnitude
    double length(void) const; //!< Returns the magnitude of the vector
    double lengthSq(void) const; //!< Returns the square of the magnitude of the vector
    double dot(const RecVector3D& a) const; //!< computes the dot product of this vector with the passed vector
	RecVector3D cross(const RecVector3D& a) const; //!< computes the cross product of this vector with the passed vector
};




//*****************************************************************************
/*!
* \brief A 2D pose
*
* A RecPose2D object represents a position and orientation in 2D cartesian space.
* 2D Poses can be operate on 2D points or other 2D poses.  They also can be
* incremented/decremented by a differential 2D pose object.
* @ingroup recGeometryGroup
*/
//*****************************************************************************
class RecPose2D
{
public:
    double x;  //!< The x coordinate of position
    double y;  //!< The y coordinate of position
    RecRadians rotZ; //!< The orientation represented as a rotation about the Z axis

    // Constructor
    RecPose2D(void); //!< Default constructor which sets the position and orientation to zero
    RecPose2D(double inx, double iny, RecRadians rotZ); //!< Sets the position and orientation to the passed position and orientation values
    RecPose2D(const RecPose2D& b); //!< Copy constructor
    RecPose2D(const RecTransform2D& trans); //!< Constructor that converts a 2D Transform to a 2D Pose.

    // Operator Overload
    void operator+=(const RecDifferentialPose2D& b); //!< Adds a differential pose to this pose
    void operator-=(const RecDifferentialPose2D& b); //!< Subtracts a differential pose from this pose
    void operator=(const RecPose2D& b); //!< Assignment Operator

    friend RecPose2D operator*(const RecPose2D& b, const RecPose2D& c); //!< Concatenate the two passed poses and return the resultant pose
    friend RecPose2D operator+(const RecPose2D& a, const RecDifferentialPose2D& b); //!< Adds a differential pose to the passed pose and returns the resultant pose
    friend RecPose2D operator+(const RecDifferentialPose2D& a, const RecPose2D& b); //!< Adds a differential pose to the passed pose and returns the resultant pose
    friend RecPose2D operator-(const RecPose2D& a, const RecDifferentialPose2D& b); //!< Subtracts a differential pose to the passed pose and returns the resultant pose

    friend bool operator==(const RecPose2D& a, const RecPose2D& b); //!< Pose comparison
    friend bool operator!=(const RecPose2D& a, const RecPose2D& b); //!< Pose comparison

    // Stream Operators
    friend std::ostream& operator<<(std::ostream& out, const RecPose2D& a); //!< writes the 2D pose object to the passed stream

    // Other Methods
    RecPose2D inverse(void) const; //!< computes and returns the inverse of this pose
    RecPoint2D getPoint() const;
};


//*****************************************************************************
/*!
* \brief A differential 2D pose
*
* A RecDifferentialPose2D object represents a differential change in position
* and orientation.  It contains delta x, delta y and delta rotZ.  They can be
* used to represent the differential motion produced by encoder readings.
* 2D Differential Poses can be added to and subtracted from RecPose2D objects.
* @ingroup recGeometryGroup
*/
//*****************************************************************************
class RecDifferentialPose2D: public RecPose2D
{
public:
    // Constructors
    RecDifferentialPose2D(void); //!< Default constructor which sets the delta x, delta y and delta rotZ to zero.
    RecDifferentialPose2D(double inx, double iny, RecRadians inw); //!< Sets the change in pose and orientation to the passed values
    RecDifferentialPose2D(const RecDifferentialPose2D& b); //!< Copy Constructor
};





/*****************************************************************************
 *       CLASS: RecPose3D
 */
/**
 * @brief A 3D pose with rotations represented by Euler Angles
 * @ingroup recGeometryGroup
 */
/* *****************************************************************************/
class RecPose3D
{
public:
    double x;
    double y;
    double z;
    RecRadians rot1;
    RecRadians rot2;
    RecRadians rot3;

    enum EulerAngleOrder{ XYX, XYZ, XZX, XZY, YXY, YXZ, YZX, YZY, ZXY, ZXZ, ZYX, ZYZ }; // Class Scope Enumerated type
    EulerAngleOrder angleOrder;

    // Constructors
    RecPose3D(EulerAngleOrder order = RecPose3D::ZYX);
    RecPose3D(double ix, double iy, double iz, RecRadians r1, RecRadians r2, RecRadians r3, EulerAngleOrder order = RecPose3D::ZYX);
    RecPose3D(const RecPose3D& b);
	RecPose3D(const RecTransform3D& trans, EulerAngleOrder order = RecPose3D::ZYX);

    // Operator Overload
    void operator+=(const RecDifferentialPose3D& b);
    void operator-=(const RecDifferentialPose3D& b);
    void operator=(const RecPose3D& b);
    void operator*=(const RecPose3D& p);
    void operator*=(const RecTransform3D& t);
    void operator*=(const RecQuaternion& q);

    friend RecPose3D operator+(const RecDifferentialPose3D& a, const RecPose3D& b);
    friend RecPose3D operator+(const RecPose3D& a, const RecDifferentialPose3D& b);
    friend RecPose3D operator-(const RecDifferentialPose3D& a, const RecPose3D& b);
    friend RecPose3D operator*(const RecPose3D& b, const RecPose3D& c);
    friend RecPoint3D operator*(const RecPose3D& pose, const RecPoint3D& pt);
    friend RecPoint2D operator*(const RecPose3D& pose, const RecPoint2D& pt);
	friend bool operator==(const RecPose3D& a, const RecPose3D& b);
    friend bool operator!=(const RecPose3D& a, const RecPose3D& b);

    // Stream Operators
    friend std::ostream& operator<<(std::ostream& out, const RecPose3D& a);

    // Other Methods
    RecPose3D inverse(void);
    bool getPlanarPose(RecPose2D &pose) const;
};


/*****************************************************************************
 *       CLASS: RecDifferentialPose3D
 *****************************************************************************/
/**@brief A Differential 3D Pose (contains delta x, delta y delta z, delta r delta p and delta w)
 * @ingroup recGeometryGroup
 */
class RecDifferentialPose3D
{
public:
    double x;
    double y;
    double z;
    RecRadians rot1;
    RecRadians rot2;
    RecRadians rot3;

    // Object wide constant
    static const double tolerance;

	// Constructors
	RecDifferentialPose3D(void);
    RecDifferentialPose3D(double inx, double iny, double inz, RecRadians inr, RecRadians inp, RecRadians inw);
    RecDifferentialPose3D(const RecDifferentialPose3D& b);

    // Operator Overload
    void operator+=(const RecDifferentialPose3D& b);
    void operator-=(const RecDifferentialPose3D& b);
    void operator=(const RecDifferentialPose3D& b);

    friend RecDifferentialPose3D operator+(const RecDifferentialPose3D& a, const RecDifferentialPose3D& b);
    friend RecDifferentialPose3D operator-(const RecDifferentialPose3D& a, const RecDifferentialPose3D& b);
    friend bool operator==(const RecDifferentialPose3D& a, const RecDifferentialPose3D& b);
    friend bool operator!=(const RecDifferentialPose3D& a, const RecDifferentialPose3D& b);

    // Stream Operators
    friend std::ostream& operator<<(std::ostream& out, const RecDifferentialPose3D& a);
};





/*****************************************************************************
 *       CLASS: RecTransform2D
 *****************************************************************************/
/**@brief A 2D euclidian transform
 * @ingroup recGeometryGroup
 */
class RecTransform2D
{
public:
    double matrixPP[2][3];

    // Object wide constant
    static const double tolerance;

    // Constructors
    RecTransform2D(void);
    RecTransform2D(const RecTransform2D& trans2D);
    RecTransform2D(const RecPose2D& pose);

    // Accessors
    double* operator[](int index) { return matrixPP[index]; };
    RecVector2D xAxis(void) const;
    RecVector2D yAxis(void) const;
    RecVector2D origin(void) const;

    // Methods
    RecTransform2D inverse(void);
    void identity(void);
    void rotateAboutZAxis(RecRadians radAngle);
    void translate(const RecVector2D& transVect);

    // Operator Overload
    friend std::ostream& operator<<(std::ostream& out, const RecTransform2D& trans2D);
    friend RecPoint2D operator*(const RecTransform2D& trans, const RecPoint2D& pt);
    friend RecPoint3D operator*(const RecTransform2D& trans, const RecPoint3D& pt);
    friend RecTransform2D operator*(const RecTransform2D& b, const RecTransform2D& c);
    friend RecPose2D operator*(const RecTransform2D& trans, const RecPose2D& p);

    friend bool operator==(const RecTransform2D& a, const RecTransform2D& b);
    friend bool operator!=(const RecTransform2D& a, const RecTransform2D& b);
};




/*****************************************************************************
 *       CLASS: RecTransform3D
 *****************************************************************************/
/**@brief A 3D euclidian transform
 * @ingroup recGeometryGroup
 */
class RecTransform3D
{
public:
    double matrixPP[3][4];

	// Object wide constant
	static const double tolerance;

    // Constructors
    RecTransform3D(void);
    RecTransform3D(const RecVector3D& vX, const RecVector3D& vY, const RecVector3D& vZ);
    RecTransform3D(const RecVector3D& vX, const RecVector3D& vY, const RecVector3D& vZ, const RecVector3D& origin);

    RecTransform3D(const RecTransform3D& trans3D);
    RecTransform3D(const RecTransform2D& trans2D);
	RecTransform3D(const RecPose3D& pose);
    RecTransform3D(const RecQuaternion& quat);

    // Operator Overload
    void operator*=(const RecPose3D& p);
    void operator*=(const RecTransform3D& t);
    void operator*=(const RecQuaternion& q);
    double* operator[](int index) {return matrixPP[index];};

    friend std::ostream& operator<<(std::ostream& out, const RecTransform3D& trans3D);
    friend RecTransform3D operator*(const RecTransform3D& b, const RecTransform3D& c);
    friend RecPoint3D operator*(const RecTransform3D& tr, const RecPoint3D& pt);
    friend RecPoint2D operator*(const RecTransform3D& tr, const RecPoint2D& pt);

    friend bool operator==(const RecTransform3D& a, const RecTransform3D& b);
    friend bool operator!=(const RecTransform3D& a, const RecTransform3D& b);

    // Methods
    RecTransform3D inverse(void);
	void identity(void);
	void rotateAboutXAxis(RecRadians radAngle);
	void rotateAboutYAxis(RecRadians radAngle);
	void rotateAboutZAxis(RecRadians radAngle);
	void translate(const RecVector3D& transVect);
    RecVector3D xAxis(void) const;
    RecVector3D yAxis(void) const;
    RecVector3D zAxis(void) const;
    RecVector3D origin(void) const;
};




/*****************************************************************************
 *       CLASS: RecQuaternion
 *****************************************************************************/
/**@brief A quaternion class
 * @ingroup recGeometryGroup
 */
class RecQuaternion
{
public:
    double quat[4];

    // constructors
    RecQuaternion(void);
    RecQuaternion(const RecQuaternion& q);
    RecQuaternion(const RecTransform3D& trans);
    RecQuaternion(const RecPose3D& pose);
    //\TODO There should be a constructor that takes in the explicit values of the quaternion.

    // Operator Overload
    double operator[](int index) {return quat[index];};
    void operator=(const RecQuaternion& b);
    void operator*=(const RecPose3D& p);
    void operator*=(const RecTransform3D& t);
    void operator*=(const RecQuaternion& q);

    friend RecQuaternion operator*(const RecQuaternion& b, const RecQuaternion& c);
    friend RecPoint3D operator*(const RecQuaternion& q, const RecPoint3D& pt);

    // Methods
    void getAxisAndAngle(RecVector3D* v, RecRadians* phi) const;
    void setAxisAndAngle(const RecVector3D& v, const RecRadians& a);
    void normalize();
    double magnitude() const;    //Spelling correction:  was 'magintude'  MAT 9/19/06

    // Stream Operator
    friend std::ostream& operator<<(std::ostream& out, const RecQuaternion& q);
};




/*****************************************************************************
 *       CLASS: RecLineSegment2D
 *****************************************************************************/
/**@brief A line segment consisting of 2 2D points
 * @ingroup recGeometryGroup
 */
class RecLineSegment2D
{
public:
    // Constructors
    RecLineSegment2D();
    RecLineSegment2D(const RecPoint2D& a, const RecPoint2D& b);
    RecLineSegment2D(const RecLineSegment2D &segP);
    RecLineSegment2D(const RecPoint2D & base, const RecVector2D & direction, double length, bool needsNormalize=true);

    // Methods
    RecPoint2D getClosestPointLine(const RecPoint2D & p) const;
    RecPoint2D getClosestPointSegment(const RecPoint2D &p) const;

    double getClosestPointLine(const RecPoint2D &p, RecPoint2D & close) const;
    double getClosestPointSegment(const RecPoint2D &p, RecPoint2D & close) const;

    int whichSideIsPointOn(const RecPoint2D &p) const;

    double getDistanceToLine(const RecPoint2D &p) const;
    double getDistanceToSegment(const RecPoint2D &p) const;

    double length() const;
    double lengthSq() const;

    int getIntersection(const RecLineSegment2D& l2, RecPoint2D &inter, RecPoint2D &intIfLine) const;
    void setPoints(const RecPoint2D & a, const RecPoint2D &b);

    inline  const RecPoint2D & p1(void) const {return p1_;};
    inline const RecPoint2D & p2(void) const {return p2_;} ;
    inline const RecVector2D  & v(void) const {return vv_;} ;

protected:
    RecPoint2D p1_;
    RecPoint2D p2_;
    RecVector2D vv_; ///< direction of the line segment, is redundant information, but could be useful for efficiency reasons
};




/*****************************************************************************
 *       CLASS: RecLineSegment3D
 *****************************************************************************/
/**@brief A line segment consisting of 2 3D points
 * @ingroup recGeometryGroup
 */
class RecLineSegment3D
{
public:
    RecPoint3D p1;
    RecPoint3D p2;

    RecLineSegment3D();
    RecLineSegment3D(const RecPoint3D& a, const RecPoint3D& b);
    RecLineSegment3D(const RecLineSegment2D& segP);
    RecLineSegment3D(const RecLineSegment3D& segP);

    double length() const;
    double lengthSq() const;
};




/*****************************************************************************
 *       CLASS: RecBox2D
 *****************************************************************************/
/**@brief An abstract box class
 * @ingroup recGeometryGroup
 */
class RecBox2D {
public:
    virtual ~RecBox2D() {};
    virtual double area() const = 0;
    virtual bool   isInside(const RecPoint2D& a) const = 0;
    virtual bool   isOutside(const RecPoint2D& a) const = 0;
    virtual bool   isOnBox(const RecPoint2D& a) const = 0;
};


/*****************************************************************************
 *       CLASS: RecAxisAlignedBox2D
 *****************************************************************************/
/**@brief A 2D axis aligned box
 * @ingroup recGeometryGroup
 */
class RecAxisAlignedBox2D:public RecBox2D
{
private:
    RecPoint2D min;
    RecPoint2D max;

public:
    RecAxisAlignedBox2D();
    RecAxisAlignedBox2D(double minX, double maxX, double minY, double maxY);
    RecAxisAlignedBox2D(const RecPoint2D& minPt, const RecPoint2D& maxPt);
    RecAxisAlignedBox2D(const RecLineSegment2D& seg);
    RecAxisAlignedBox2D(const RecAxisAlignedBox2D& box);
    ~RecAxisAlignedBox2D();

    // Method
    double area() const;
    bool   isInside(const RecPoint2D& a) const;
    bool   isInsideTiled(const RecPoint2D& a) const;
    bool   isOutside(const RecPoint2D& a) const;
    bool   isOutsideTiled(const RecPoint2D& a) const;
    bool   isOnBox(const RecPoint2D& a) const;
    bool   overlap(const RecAxisAlignedBox2D& box2) const;

    RecPoint2D getMinPoint() const;
    void setMinPoint(const RecPoint2D& minPt);
    RecPoint2D getMaxPoint() const;
    void setMaxPoint(const RecPoint2D& maxPt);
    void setMinMaxPoints( const RecPoint2D& minPt, const RecPoint2D& maxPt );

    void normalize();
    RecAxisAlignedBox2D overlapArea(const RecAxisAlignedBox2D & otherBox) const;
};




/*****************************************************************************
 *       CLASS: RecBox3D
 *****************************************************************************/
/**@brief An abstract box class
 * @ingroup recGeometryGroup
 */
class RecBox3D
{
public:
    virtual ~RecBox3D() = 0;
    virtual double volume() const = 0;
    virtual bool   isInside(const RecPoint3D& a) const = 0;
    virtual bool   isOutside(const RecPoint3D& a) const = 0;
    virtual bool   isOnBox(const RecPoint3D& a) const = 0;
};

/**
 * @brief RecBox3D destructor
 *
 * Even if a destructor is pure virtual, it needs an implementation -- Tug 09/17/06
 */
inline RecBox3D::~RecBox3D() {}


/*****************************************************************************
 *       CLASS: RecAxisAlignedBox3D
 *****************************************************************************/
/**@brief A 2D axis aligned box
 * @ingroup recGeometryGroup
 */
class RecAxisAlignedBox3D:public RecBox3D
{
private:
    RecPoint3D min;
    RecPoint3D max;

public:
    RecAxisAlignedBox3D();
    RecAxisAlignedBox3D(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
    RecAxisAlignedBox3D(const RecPoint3D& minPt, const RecPoint3D& maxPt);
    RecAxisAlignedBox3D(const RecLineSegment3D& seg);
    RecAxisAlignedBox3D(const RecAxisAlignedBox3D& box);
    ~RecAxisAlignedBox3D() {};

    double volume() const;
    bool   isInside(const RecPoint3D& a) const;
    bool   isOutside(const RecPoint3D& a) const;
    bool   isOnBox(const RecPoint3D& a) const;
    RecPoint3D getMin() const;
    RecPoint3D getMax() const;
    void normalize();                             // MAT- added 10-6-06
};





/*****************************************************************************
 *       CLASS: RecPolygon2D
 *****************************************************************************/
/**@brief A planar polygon
 * @ingroup recGeometryGroup
 */
class RecPolygon2D
{
public:
    // Constructor
    RecPolygon2D();
    RecPolygon2D(const RecPolygon2D &polyP);
    template <class Pt>
    RecPolygon2D(const std::vector<Pt> &vertices);
    ~RecPolygon2D();

    // Methods
    int getNumVertices()const;
    int getVertex(int index, RecPoint2D& vertex) const;
    void getCenter(RecPoint2D& center) const;
    void getBoundingBox(RecPoint2D& min, RecPoint2D &max) const;

    void addVertex(const RecPoint2D& vertex);
    void sortVerticesCCW();
    void clearVertices();

    int containsPoint(const RecPoint2D& point) const;
    double distanceToPoint(const RecPoint2D &point) const;
    int hasOverlap(const RecPolygon2D& poly) const;

    double computeArea() const;

    void computeMinEnclosingRectangle(const RecPolygon2D& poly);
    void computeIntersection(RecPolygon2D& poly1, RecPolygon2D& poly2);
    void computeUnion(RecPolygon2D& poly1, RecPolygon2D& poly2);
    void computeConvexHull(RecPolygon2D &hullP);

    // Operator Overload
    RecPolygon2D& operator=(const RecPolygon2D& p);

    const std::vector<RecPoint2D> & getVertices() const { return verticesP; }
    RecPolygon2D transform(const RecTransform2D & trans);

private:
    int numVertices;

    std::vector<RecPoint2D> verticesP;
    RecPoint2D  center;
    RecPoint2D  min;
    RecPoint2D  max;

    std::vector<double> angleToVertexP;

    // Methods
    void updateCenterAndBBox();
    void updateVertexAngles();
    int isConvexCorner(const RecPoint2D& p1, const RecPoint2D& p2, const RecPoint2D& p3);

    unsigned int findSmallestAnglePositive(std::vector<RecPoint2D> & pts, unsigned int startIndex);
    unsigned int findSmallestAngleNegative(std::vector<RecPoint2D> & pts, unsigned int startIndex);
    std::vector<RecPoint2D> jarvisMarch(const std::vector<RecPoint2D> &inPts);
};



/*****************************************************************************
 *       CLASS: RecPolygon3D
 * DESCRIPTION: A 3D polygon - not fully supported yet
 *****************************************************************************/
/*
class RecPolygon3D {
public:
  RecPolygon3D();
  RecPolygon3D(RecPolygon3D &polyP);
  ~RecPolygon3D();

  int getNumVertices()const;
  int getVertex(int index, RecPoint3D& vertex) const;
  void getBoundingBox(RecPoint3D& min, RecPoint3D &max) const;

  void addVertex(const RecPoint2D& vertex);
  void clearVertices();

private:
  int numVertices;
  RecPoint3D* verticesP;
  RecPoint3D  min;
  RecPoint3D  max;
};
*/



#endif //#ifndef _RECGEOMETRY_H_
