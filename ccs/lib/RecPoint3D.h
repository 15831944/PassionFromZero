#ifndef _RECPOINT3D_H_
#define _RECPOINT3D_H_


#include <iostream>

using namespace std;


/*** Data structure: RecPoint3D ***/
class RecPoint3D
{
public:
    double x; //!< The x coordinate of the point
    double y; //!< The y coordinate of the point
    double z; //!< The z coordinate of the point

    // Object wide constant
    static const double tolerance; //!< tolerance for which two RecPoint3D objects are considered equal

  //@{
  /*!
  \name Constructors
  Create a RecPoint3D object.
  */
  RecPoint3D(void); //!< Default Constructor, the x, y and z coordinates are set to zero
  RecPoint3D(double inx, double iny, double inz); //!< Sets the x, y and z coordinates to the passed x, y and z values
//  RecPoint3D(const RecPoint2D& b); //!< Converts a 2D point to a 3D one by setting the z coordinate of the 3D point to zero.
  RecPoint3D(const RecPoint3D& b); //!< Copy Constructor
  //@}

  //@{
  /*!
  \name Arithmetic Operators
  Methods for operating on 2D points.
  */
  void operator+=(const RecPoint3D& b); //!< Point Addition
  void operator-=(const RecPoint3D& b); //!< Point Subtraction
  void operator*=(double b); //!< Multiplication by scalar
  void operator/=(double b); //!< Division by scalar
  void operator=(const RecPoint3D& b); //!< Point Assignment

  // Friend Arithmetic Operators
    friend RecPoint3D operator/(const RecPoint3D& a, double b); //!< Division by scalar
  friend RecPoint3D operator*(const RecPoint3D& a, double b); //!< Multiplication by scalar
  friend RecPoint3D operator*(double a, const RecPoint3D& b); //!< Multiplication by scalar
  friend RecPoint3D operator+(const RecPoint3D& a, const RecPoint3D& b); //!< Point Addition
  friend RecPoint3D operator-(const RecPoint3D& a, const RecPoint3D& b); //!< Point Subtraction

  friend bool operator==(const RecPoint3D& a, const RecPoint3D& b); //!< Point Comparison
  friend bool operator!=(const RecPoint3D& a, const RecPoint3D& b); //!< Point Comparison
  //@}

  // Other Methods
    double distance(const RecPoint3D& a) const; //!< Returns the cartesian distance between this point and the passed argument
    double distanceSq(const RecPoint3D& a) const; //!< Returns the square of the cartesian distance between this point and the passed argument

  // Stream Operators
  friend ostream& operator<<(ostream& out, const RecPoint3D& a);//!< writes the 3D point object to the passed stream
};

#endif //#ifndef _RECPOINT3D_H_
