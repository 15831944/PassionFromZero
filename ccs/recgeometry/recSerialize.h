/**
 * @file recSerialize.h
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date: 09/10/2006
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _RECSERIALIZE_H_
#define _RECSERIALIZE_H_

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include "recGeometry.h"

namespace boost 
{
  namespace serialization 
  {


/**
 * @brief functions to serialize RecRadians
 */
    template<class Archive>
    void save(Archive & ar, const RecRadians &r, unsigned int version)
    {
      double tmp = (double)(r);
      ar << tmp;
    };
    
    template<class Archive>
    void load(Archive & ar, RecRadians & r, unsigned int version)
    {
      double tmp;
      ar >> tmp;
      r = tmp;
    };


/**
 * @brief functions to serialize point2d
 */
    template<class Archive>
    void serialize(Archive &ar, RecPoint2D &r, unsigned int version)
    {
      ar & r.x & r.y;
    }

/**
 * @brief functions to serialize point3d
 */
    template<class Archive>
    void serialize(Archive &ar, RecPoint3D &r, unsigned int version)
    {
      ar & r.x & r.y & r.z;
    }
    
/**
 * @brief functions to serialize vector2d
 */
    template<class Archive>
    void serialize(Archive &ar, RecVector2D &r, unsigned int version)
    {
        ar & r.x & r.y;
    }

/**
 * @brief functions to serialize vector3d
 */
    template<class Archive>
    void serialize(Archive &ar, RecVector3D &r, unsigned int version)
    {
      ar & boost::serialization::base_object<RecPoint3D>(r);
    }
    
/**
 * @brief functions to serialize pose2d
 */
    template<class Archive>
    void serialize(Archive &ar, RecPose2D &r, unsigned int version)
    {
      ar & r.x & r.y & r.rotZ;
    }

/**
 * @brief functions to serialize differentialpose2d
 */
    template<class Archive>
    void serialize(Archive &ar, RecDifferentialPose2D &r, unsigned int version)
    {
      ar & boost::serialization::base_object<RecPose2D>(r);
    }

/**
 * @brief functions to serialize pose3d
 */
    template<class Archive>
    void serialize(Archive &ar, RecPose3D &r, unsigned int version)
    {
      ar & r.x & r.y & r.z;
      ar & r.rot1 & r.rot2 & r.rot3;
      ar & r.angleOrder;
    }

/**
 * @brief functions to serialize differentialPose3d
 */
    template<class Archive>
    void serialize(Archive &ar, RecDifferentialPose3D &r, unsigned int version)
    {
      ar & r.x & r.y & r.z;
      ar & r.rot1 & r.rot2 & r.rot3;
    }

/**
 * @brief functions to serialize transform2d
 */
    template<class Archive>
    void serialize(Archive &ar, RecTransform2D &r, unsigned int version)
    {
      ar & r.matrixPP;
    } 

/**
 * @brief functions to serialize transform3d
 */
    template<class Archive>
    void serialize(Archive &ar, RecTransform3D &r, unsigned int version)
    {
      ar & r.matrixPP;
    }
    
/**
 * @brief functions to serialize quaternion
 */
    template<class Archive>
    void serialize(Archive &ar, RecQuaternion &r, unsigned int version)
    {
      ar & r.quat;
    }

/**
 * @brief functions to serialize linesegment3d
 */
    template<class Archive>
    void serialize(Archive &ar, RecLineSegment3D &r, unsigned int version)
    {
      ar & r.p1 & r.p2;   
    }

    template<class Archive>
    void serialize(Archive &ar, RecBox2D &r, unsigned int version) 
    {
    }

/**
 * @brief functions to serialize axisAlignedBox2D
 */
    template<class Archive>
    void serialize(Archive &ar, RecAxisAlignedBox2D &r, unsigned int version)
    {
      RecPoint2D min, max;
      min = r.getMinPoint(); max = r.getMaxPoint();
      ar & min & max;
      r = RecAxisAlignedBox2D(min,max);
    }

/**
 * @brief functions to serialize axisAlignedBox3D
 */
    template<class Archive>
    void serialize(Archive &ar, RecAxisAlignedBox3D &r, unsigned int version)
    {
      RecPoint3D min, max;
      min = r.getMin(); max = r.getMax();
      ar & min & max;
      r = RecAxisAlignedBox3D(min,max);
    }
/**
 * @brief functions to serialize polygon2D
 */
    template<class Archive>
    void save(Archive & ar, const RecPolygon2D &r, unsigned int version)
    {
        if(version < 2)
        {
            RecPoint2D pt;
            const int numVertices = r.getNumVertices();
            ar << numVertices;
            for (int vv=0; vv<r.getNumVertices(); vv++)
            {
                r.getVertex(vv,pt);
                ar << (const RecPoint2D)pt;
            }
        } else {
            ar << r.getVertices();
        }
    }

    template<class Archive>
    void load(Archive & ar, RecPolygon2D &r, unsigned int version)
    {
        if(version < 2)
        {
            RecPoint2D pt;
            int numVertices ;
            ar >> numVertices;
            for (int vv=0; vv<numVertices; vv++)
            {
                ar >> pt;
                r.addVertex(pt);
            }
        } else {
            std::vector<RecPoint2D> vertVec;
            ar >> vertVec;
            r = RecPolygon2D(vertVec);
        }
    }
  }
}

BOOST_SERIALIZATION_SPLIT_FREE(RecRadians);
BOOST_SERIALIZATION_SPLIT_FREE(RecPolygon2D);
BOOST_CLASS_VERSION(RecPolygon2D,2);

#endif //ifndef _RECSERIALIZE_H_
