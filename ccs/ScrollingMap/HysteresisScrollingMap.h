/**
 * @file      HysteresisScrollingMap.h
 * @author    Jason Ziglar
 * @date      2007/07/19
 *
 * @attention Copyright (C) 2006
 * @attention Carnegie Mellon University
 * @attention Developed under contract for ____________
 * @attention All rights reserved
 *
 * @attention LIMITED RIGHTS:
 * @attention The Government is granted Limited Rights to this Data.
 *            Use, duplication, or disclosure is subject to the
 *            restrictions as stated in Agreement _______
 *            between Carnegie Mellon University and the Government.
 */

#ifndef _HYSTERESISSCROLLINGMAP_H_
#define _HYSTERESISSCROLLINGMAP_H_

#include <task/Task.h>
#include <ScrollingMap/ScrollingMap.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

#include <utility>
#include <iostream>

template <class TT>
class HysteresisScrollingMap
{
    public:
        // a new constructor to signify that we're doing row-column scrolling
        HysteresisScrollingMap(double size_m, double cellSize_m, bool RC, TT voidedValue);

        // old constructors
        HysteresisScrollingMap(double size_m, double cellSize_m);
        HysteresisScrollingMap(const HysteresisScrollingMap<TT>& otherMap);
        HysteresisScrollingMap();
        virtual ~HysteresisScrollingMap();
        void operator=(const HysteresisScrollingMap& other);

        void setUseShallowCopy() {dataMap_.setUseShallowCopy(); timeMap_.setUseShallowCopy();};
        void setCompressOnSerialize(void) {dataMap_.setCompressOnSerialize(); timeMap_.setCompressOnSerialize();};
        void setLifeTime(const long & seconds, const long & milliseconds);
        void setLifeTime(const double & seconds);
        void setLifeTime(const boost::posix_time::time_duration & lifeTime);
        boost::posix_time::time_duration getLifeTime();
        
        TT & get(const RecPoint2D & globalPt, bool &isValid);
        TT & get(const RecPoint2D & globalPt, boost::posix_time::ptime & timeStamp, bool & isValid);
        TT & get(const RecPoint3D & globalPt, bool &isValid);
        TT & get(const RecPoint3D & globalPt, boost::posix_time::ptime & timeStamp, bool & isValid);

        TT & set(const RecPoint2D & globalPt, const TT & value);
        TT & set(const RecPoint2D & globalPt, const boost::posix_time::ptime & timeStamp, const TT & value);
        TT & set(const RecPoint3D & globalPt, const TT & value);
        TT & set(const RecPoint3D & globalPt, const boost::posix_time::ptime & timeStamp, const TT & value);

        bool isValid(const RecPoint2D & globalPt);
        bool isTimeValid(const RecPoint2D & globalPt);
        bool isSpaceValid(const RecPoint2D & globalPt);

        bool isValid(const RecPoint3D & globalPt);
        bool isTimeValid(const RecPoint3D & globalPt);
        bool isSpaceValid(const RecPoint3D & globalPt);

        double getCellSize();
        double getSize();
        int getDimension();
        RecAxisAlignedBox2D getBounds() const {return dataMap_.getBounds();};
        void convertGlobalPointToIndices(const RecPoint2D & globalPt, long &row,  long &col) const;
        void convertGlobalPointToIndices(const RecPoint3D & globalPt, long &row,  long &col) const;

        void getCellCenter(const RecPoint2D & globalPt, RecPoint2D & centerPoint);
        void getCellCenter(const RecPoint3D & globalPt, RecPoint3D & centerPoint);

        void centerAt(const RecPoint2D & point);

        bool checkTimeStamp(const boost::posix_time::ptime & time, const boost::posix_time::time_duration & lifetime);
        void clearByTime();

        ScrollingMap<TT> & getDataMap() {return dataMap_;} //< Accessor to data map - if the clearByTime() function is called first, this is a map with only valid points in it.

        class iterator : public std::iterator<std::forward_iterator_tag, TT>
        {
            public:
                iterator & operator++(); ///< pre increment operator
                iterator operator++(int); ///< post increment operator
                iterator & operator=(iterator const &other); ///< assignment operator
                bool operator==(iterator const &other) const;
                bool operator!=(iterator const &other) const;

                inline std::pair<TT,bool> operator*() 
                {
                    std::pair<TT,bool> data(*dataIter_);
                    std::pair<boost::posix_time::ptime, bool> time(*timeIter_);
                    data.second = data.second && this->isTimeValid();
                    return data;
                }
                iterator(const iterator & other);
                iterator(); ///< this constructors sole purpose is to set the off end
                ~iterator() {map_ = NULL;}
                bool isValid();
                bool isTimeValid() {return (*timeIter_).second && map_->checkTimeStamp((*timeIter_).first, map_->getLifeTime());}
                bool isSpaceValid(){return (*dataIter_).second;}
                inline void set( const TT& val ) {dataIter_.set(val); timeIter_.set(task::Task::getSystemTime());}
                inline void set( const TT& val , const boost::posix_time::ptime & time) {dataIter_.set(val); timeIter_.set(time);}
                inline int getRow() const {return dataIter_.getRow();}
                inline int getCol() const {return dataIter_.getCol();}
                inline RecPoint2D getCenterPoint() const {return dataIter_.getCenterPoint();}

                inline std::pair<TT,bool> getNeighbor(int rOffset, int cOffset) const
                {
                    std::pair<TT, bool> cell(dataIter_.getNeighbor(rOffset, cOffset));
                    std::pair<boost::posix_time::ptime, bool> time(timeIter_.getNeighbor(rOffset, cOffset));
                    return ( std::make_pair( cell.value, cell.second && map_->checkTimeStamp(time.first, map_->getLifeTime())) );
                }

                inline void setNeighbor( int rOffset, int cOffset, const TT& val ) 
                {
                    dataIter_.setNeighbor(rOffset, cOffset, val);
                    timeIter_.setNeighbor(rOffset, cOffset, task::Task::getSystemTime());
                }
                
                inline void setNeighbor( int rOffset, int cOffset, const TT& val, const boost::posix_time::ptime & time ) 
                {
                    dataIter_.setNeighbor(rOffset, cOffset, val);
                    timeIter_.setNeighbor(rOffset, cOffset, time);
                }

            protected:
                friend class HysteresisScrollingMap;
                iterator(const typename ScrollingMap<TT>::iterator & data, const typename ScrollingMap<boost::posix_time::ptime>::iterator & time, HysteresisScrollingMap<TT> & map);
                iterator(const IndexRange & range, HysteresisScrollingMap<TT> & map);
                HysteresisScrollingMap<TT> * map_;
                typename ScrollingMap<TT>::iterator dataIter_;
                ScrollingMap<boost::posix_time::ptime>::iterator timeIter_;
        };

        class LineIterator : public std::iterator<std::forward_iterator_tag, TT>
        {
            public:
                LineIterator & operator++(); ///<prefix increment, steps along line
                LineIterator operator++(int);///<postfix increment, steps along line
                LineIterator & operator=(LineIterator const &other); ///<assignment operator
                bool operator==(LineIterator const &other) const;///< equality operator
                bool operator!=(LineIterator const &other) const {return !((*this)==other);} ///<inequality operator
                LineIterator(const LineIterator &other); ///< copy constructor
                LineIterator(void); ///< creates a end of line iterator
                inline int getRow() const {return dataIter_.getRow();};
                inline int getCol() const {return dataIter_.getCol();};
                // NOTE:  The * operator no longer returns a reference, just the value. See LineInterator::set below
                inline std::pair<TT,bool> operator*() 
                { 
                    std::pair<TT, bool> cell(*dataIter_);
                    cell.second = cell.second && isTimeValid();
                    return cell; 
                }
                // NOTE:  The * operator no longer returns a pointer, just the value. See LineInterator::set below
                //inline TT operator->() const {return curCell_->value;};  NOW ITS DEAD
                bool isValid(); ///< reports true if the current cell is valid
                bool isTimeValid() {return (*timeIter_).second && map_->checkTimeStamp((*timeIter_).first, map_->getLifeTime());}
                bool isSpaceValid() {return (*dataIter_).second;}
                // This is the new method for setting the value in a cell using a line iterator.  This method not only sets the
                // the value, but it also directs the cell to be at the location of the line iterator.
                inline void set(const TT& val) {this->set(val, task::Task::getSystemTime());}
                inline void set(const TT& val, const boost::posix_time::ptime & timestamp) {dataIter_.set(val); timeIter_.set(timestamp);}

                inline RecPoint2D globalCoord() { return dataIter_.globalCoord();}

            private:
                friend class HysteresisScrollingMap;
                LineIterator(const typename ScrollingMap<TT>::LineIterator & data, const typename ScrollingMap<boost::posix_time::ptime>::LineIterator & time, HysteresisScrollingMap<TT> & map);
                LineIterator(const IndexRange &range, HysteresisScrollingMap<TT> & map);
                HysteresisScrollingMap * map_;
                typename ScrollingMap<TT>::LineIterator dataIter_;
                typename ScrollingMap<boost::posix_time::ptime>::LineIterator timeIter_;

        };
        LineIterator beginLine(const RecPoint2D & start, const RecPoint2D & end);
        LineIterator endLine();
        iterator begin();
        iterator begin(const RecPoint2D & min, const RecPoint2D & max);
        iterator begin(const RecAxisAlignedBox2D & box);
        iterator end(void);

    protected:
        ScrollingMap<TT> dataMap_;
        boost::posix_time::time_duration lifetime_;
        ScrollingMap<boost::posix_time::ptime> timeMap_;
        friend class boost::serialization::access;
        template <class Archive>
        void save(Archive & ar, unsigned int version) const
        {
            ar << dataMap_;
            ar << lifetime_;
            ar << timeMap_;
        }

        template <class Archive>
        void load(Archive & ar, unsigned int version)
        {
            ar >> dataMap_;
            ar >> lifetime_;
            ar >> timeMap_;
        }
        BOOST_SERIALIZATION_SPLIT_MEMBER();
};

typedef HysteresisScrollingMap<unsigned char> HysteresisScrollingByteMap;
typedef HysteresisScrollingMap<double> HysteresisScrollingDoubleMap;

/**
 * @brief Returns the map's hysteresis window. This is useful for iterators to be able to seamlessly test time
 * hysteresis during isValid evaluation.
 */
template <class TT>
boost::posix_time::time_duration HysteresisScrollingMap<TT>::getLifeTime()
{
    return lifetime_;
}

/**
 * Set the time hysteresis for the map with seconds and milliseconds.
 */
template <class TT>
void HysteresisScrollingMap<TT>::setLifeTime(const long & tSeconds, const long & tMilliseconds)
{
    lifetime_ = boost::posix_time::seconds(tSeconds) + boost::posix_time::milliseconds(tMilliseconds);
}

/**
 * Set the time hysteresis for the map with seconds, which will be broken down into hours, minutes, seconds, and
 * microseconds.
 */
template <class TT>
void HysteresisScrollingMap<TT>::setLifeTime(const double & seconds)
{
    double totalSeconds(seconds);
    long tdHours((long)(totalSeconds/3600));
    totalSeconds -= tdHours*3600;
    long tdMinutes((long)(totalSeconds/60));
    totalSeconds -= tdMinutes*60;
    long tdSeconds((long)totalSeconds);
    totalSeconds -= tdSeconds;
    long tdMilliseconds((long)(totalSeconds*1e3));
    lifetime_ = hours(tdHours) + minutes(tdMinutes) + boost::posix_time::seconds(tdSeconds) + milliseconds(tdMilliseconds);
}

/**
 * Set the time hysteresis for the map with a boost::posix_time::time_duration. In short, a copy constructor style
 * mutator.
 */
template <class TT>
void HysteresisScrollingMap<TT>::setLifeTime(const boost::posix_time::time_duration & lifeTime)
{
    lifetime_ = lifeTime;
}

/**
 * Tests a ptime against a lifetime, seeing if the timestamp is within the map's lifetime.
 * @return true if timeStamp is still valid, false if it is invalid.
 */
template <class TT>
bool HysteresisScrollingMap<TT>::checkTimeStamp(const boost::posix_time::ptime & timeStamp, const boost::posix_time::time_duration & lifetime)
{
    bool status(false);
    //If lifetime is not set, assume the hysteresis is set to forever. Always return true
    if(lifetime.is_special())
    {
        status = true;
    }
    //Special timestamp means point has never been written to, or was written with an invalid timestamp. These points
    //should be rejected.
    else if(!(timeStamp.is_special()))
    {
        //This check could be optimized by doing the lifetime subtraction in the time map, but then we are not robust
        //against playback. What to do?
        status = (task::Task::getSystemTime() - lifetime <= timeStamp);
    }
    return status;
}

/**
 * @brief Default constructor creates a 50mx50m scrolling map with 0.25m cells, with hysteresis turned off.
 */
template <class TT>
HysteresisScrollingMap<TT>::HysteresisScrollingMap()
    :dataMap_(50, 0.25),
    lifetime_(boost::posix_time::pos_infin),
    timeMap_(50, 0.25)
{
}

/**
 * Creates a map that is size_m x size_m large, with cellSize_m resolution, selectively enabling row-column scrolling,
 * and with a given default value. Hysteresis is turned off by default.
 */
template <class TT>
HysteresisScrollingMap<TT>::HysteresisScrollingMap(double size_m, double cellSize_m, bool RC, TT voidedValue)
:dataMap_(size_m, cellSize_m, RC, voidedValue),
    lifetime_(boost::posix_time::pos_infin),
    timeMap_(size_m, cellSize_m, RC, boost::posix_time::ptime())
{
}

/**
 * Copy constructor, which does as advertised.
 */
template <class TT>
HysteresisScrollingMap<TT>::HysteresisScrollingMap(const HysteresisScrollingMap<TT> & otherMap)
    :dataMap_(otherMap.dataMap_),
    lifetime_(otherMap.lifetime_),
    timeMap_(otherMap.timeMap_)
{
}

/**
 * Creates a map which is size_m x size_m large, and cellSize_m resolution. Hysteresis is turned off by default.
 */
template <class TT>
HysteresisScrollingMap<TT>::HysteresisScrollingMap(double size_m, double cellSize_m)
    :dataMap_(size_m, cellSize_m),
    lifetime_(boost::posix_time::pos_infin),
    timeMap_(size_m, cellSize_m)
{
}

template <class TT>
HysteresisScrollingMap<TT>::~HysteresisScrollingMap()
{
}

/**
 * Assignment operator, which copies the map as you would expect. This doesn't do any fancy tricks to speed up copying.
 */
template <class TT>
void HysteresisScrollingMap<TT>::operator=(const HysteresisScrollingMap<TT>& other)
{
    lifetime_ = other.lifetime_;
    dataMap_ = other.dataMap_;
    timeMap_ = other.timeMap_;
}

/**
 * Accessor for a map cell, using a RecPoint2D to reference the cell.
 * @return reference to value in data cell, and isValid set to true if point is within the map and within the timestamp
 * lifetime, false otherwise.
 */
template <class TT>
TT & HysteresisScrollingMap<TT>::get(const RecPoint2D & globalPt, bool &isValid)
{
    bool validTime(true);
    TT & data(dataMap_.get(globalPt, isValid));
    boost::posix_time::ptime timeVal(timeMap_.get(globalPt, validTime));

    isValid = validTime && isValid && checkTimeStamp(timeVal, lifetime_);

    return data;
}

/**
 * Accessor for a map cell, using a RecPoint3D to reference the cell. In this case, the z value is ignored, and the
 * function is provided to minimize the amount of converting RecPoint3D's to RecPoint2D's for interfacing.
 * @return reference to value in data cell, and isValid set to true if point is within the map and within the timestamp
 * lifetime, false otherwise.
 */
template <class TT>
TT & HysteresisScrollingMap<TT>::get(const RecPoint3D & globalPt, bool &isValid)
{
    bool validTime(true);
    TT &data(dataMap_.get(globalPt, isValid));
    boost::posix_time::ptime timeVal(timeMap_.get(globalPt, validTime));

    isValid = validTime && isValid && checkTimeStamp(timeVal, lifetime_);

    return data;
}

/**
 * Mutator for a map cell, which allows setting both the timestamp and the value for a cell.
 * @return reference to the data value within the cell.
 */
template <class TT>
TT & HysteresisScrollingMap<TT>::set(const RecPoint2D & globalPt, const boost::posix_time::ptime & timeStamp, const TT & value)
{
    timeMap_.set(globalPt, timeStamp);
    return dataMap_.set(globalPt, value);
}

/**
 * Mutator for a map cell, which sets the data map cell to value, and the time map to task::Task::getSystemTime(). This
 * is a convenience function which calls set(const RecPoint2D & globalPt, const boost::posix_time::ptime & timeStamp,
 * const TT & value).
 */
template <class TT>
TT & HysteresisScrollingMap<TT>::set(const RecPoint2D & globalPt, const TT & value)
{
    return this->set(globalPt, task::Task::getSystemTime(), value);
}

/**
 * Mutator for a map cell, which allows setting both the timestamp and value for a cell.
 * @return reference to the data value within the cell.
 */
template <class TT>
TT & HysteresisScrollingMap<TT>::set(const RecPoint3D & globalPt, const boost::posix_time::ptime & timeStamp, const TT & value)
{
    timeMap_.set(globalPt, timeStamp);
    return dataMap_.set(globalPt, value);
}

/**
 * Mutator for a map cell, which allows setting the value of the cell. This sets the timestamp to
 * task::Task::getSystemTime(). This is a convenience function which allows the usage of RecPoint3D.
 */
template <class TT>
TT &  HysteresisScrollingMap<TT>::set(const RecPoint3D & globalPt, const TT & value)
{
    return this->set(globalPt, task::Task::getSystemTime(), value);
}

/**
 * @brief creates an iterator that will step over the region [min,max) points (note: max specifically not included)
 */
template <class TT>
typename HysteresisScrollingMap<TT>::iterator
HysteresisScrollingMap<TT>::begin(const RecPoint2D & min,
                        const RecPoint2D & max)
{
    typename ScrollingMap<TT>::iterator data(dataMap_.begin(min, max));
    typename ScrollingMap<boost::posix_time::ptime>::iterator time(timeMap_.begin(min, max));
    return iterator(data, time, *this);
}

/**
 * @brief returns an iterator to step over the given rectangular region
 */
template <class TT>
typename HysteresisScrollingMap<TT>::iterator
HysteresisScrollingMap<TT>::begin(const RecAxisAlignedBox2D & box)
{
    return begin(box.getMinPoint(),box.getMaxPoint());
}

/**
 * @brief returns an iterator to step over the entire map as-is
 */
template <class TT>
typename HysteresisScrollingMap<TT>::iterator
HysteresisScrollingMap<TT>::begin()
{
    return begin(this->getBounds());
}

/**
 * @brief returns an iterator that can be used to test against when iterating over regions
 */
template <class TT>
typename HysteresisScrollingMap<TT>::iterator
HysteresisScrollingMap<TT>::end(void)
{
    iterator ret;
    return ret;
}

template <class TT>
void HysteresisScrollingMap<TT>::clearByTime()
{
    typename HysteresisScrollingMap<TT>::iterator ii(this->begin());
    for(; ii != this->end(); ++ii)
    {
        if(!(ii.isTimeValid()))
        {
            ii.set(0, boost::posix_time::ptime(boost::posix_time::not_a_date_time));
        }
    }
}

/**
 * Test if a given point is valid both spatially and within the time hysteresis lifetime.
 */
template <class TT>
bool HysteresisScrollingMap<TT>::isValid(const RecPoint2D & globalPt)
{
    return isTimeValid(globalPt) && isSpaceValid(globalPt);
}

/**
 * Test if a given point is within the time hysteresis lifetime.
 */
template <class TT>
bool HysteresisScrollingMap<TT>::isTimeValid(const RecPoint2D & globalPt)
{
    bool timeValid(false);
    boost::posix_time::ptime time;

    time = timeMap_.get(globalPt, timeValid);

    return timeValid && checkTimeStamp(time, lifetime_);
}

/**
 * Test if a given point is valid spatially.
 */
template <class TT>
bool HysteresisScrollingMap<TT>::isSpaceValid(const RecPoint2D & globalPt)
{
    bool dataValid(false);
    dataMap_.get(globalPt, dataValid);
    return dataValid;
}

/**
 * Test if a given point is valid both spatially and within the time hysteresis lifetime.
 */
template <class TT>
bool HysteresisScrollingMap<TT>::isValid(const RecPoint3D & globalPt)
{
    return isTimeValid(globalPt) && isSpaceValid(globalPt);
}

/**
 * Test if a given point is within the time hysteresis lifetime.
 */
template <class TT>
bool HysteresisScrollingMap<TT>::isTimeValid(const RecPoint3D & globalPt)
{
    return isTimeValid(RecPoint2D(globalPt.x, globalPt.y));
}

/**
 * Test if a given point is valid spatially.
 */
template <class TT>
bool HysteresisScrollingMap<TT>::isSpaceValid(const RecPoint3D & globalPt)
{
    return isSpaceValid(RecPoint2D(globalPt.x, globalPt.y));
}

template <class TT>
TT & HysteresisScrollingMap<TT>::get(const RecPoint2D & globalPt, boost::posix_time::ptime & timeStamp, bool & isValid)
{
    timeStamp = timeMap_.get(globalPt, isValid);
    return get(globalPt, isValid);
}

template <class TT>
TT & HysteresisScrollingMap<TT>::get(const RecPoint3D & globalPt, boost::posix_time::ptime & timeStamp, bool & isValid)
{
    timeStamp = timeMap_.get(globalPt, isValid);
    return get(globalPt, isValid);
}

template <class TT>
void HysteresisScrollingMap<TT>::centerAt(const RecPoint2D & point)
{
    timeMap_.centerAt(point);
    dataMap_.centerAt(point);
}
#include <ScrollingMap/HysteresisScrollingMap.def.h>

#endif
