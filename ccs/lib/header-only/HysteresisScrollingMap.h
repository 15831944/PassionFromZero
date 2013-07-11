
#ifndef _HYSTERESISSCROLLINGMAP_H_
#define _HYSTERESISSCROLLINGMAP_H_


#include "ScrollingMap.h"
#include <boost/date_time/posix_time/ptime.hpp>
//#include <boost/date_time/posix_time/time_serialize.hpp>
#include <iostream>
#include <utility>

using namespace boost::posix_time;

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
//                inline void set( const TT& val ) {dataIter_.set(val); timeIter_.set(task::Task::getSystemTime());}
                  inline void set( const TT& val ) {ptime t(microsec_clock::local_time());
                                                    dataIter_.set(val); timeIter_.set(t);}

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
//                    timeIter_.setNeighbor(rOffset, cOffset, task::Task::getSystemTime());
                      ptime t(microsec_clock::local_time());
                      timeIter_.setNeighbor(rOffset, cOffset, t);
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
//                inline void set(const TT& val) {this->set(val, task::Task::getSystemTime());}
                  inline void set(const TT& val) {ptime t(microsec_clock::local_time());
                                                  this->set(val, t);}



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
/*
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
*/
};

typedef HysteresisScrollingMap<unsigned char> HysteresisScrollingByteMap;
typedef HysteresisScrollingMap<double> HysteresisScrollingDoubleMap;


#include "HysteresisScrollingMap.def.h"
#include "HysteresisScrollingMap.def2.h"

#endif  //#ifndef _HYSTERESISSCROLLINGMAP_H_
