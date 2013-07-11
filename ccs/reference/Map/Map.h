/**
 * @file Map.h
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date: 9/19/2006
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _MAP_H_
#define _MAP_H_

#include <iostream>
#include <recgeometry/recGeometry.h>
#include <iterator>
#include <TimeStamp/TimeStamp.h>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/binary_object.hpp>
#include <zlib.h>
#include <opencv/cv.h>

/**
 * @brief a set of indices bounding some part of a map
 */
struct IndexRange {
    long rs, re; ///<row index ranges, should be considered rs<=range<re
    long cs, ce; ///<column index ranges, should be considered cs<=range<ce
    void normalize(); //! ensures that rs<=re & cs<=ce
};


/**
 * @brief a templatized 2D map class that allows for indexing by global coordinates
 * @ingroup perceptionTypesGroup
 */
template <class TT> 
class Map 
{

  public:

    class LineIterator:
        public std::iterator<std::forward_iterator_tag, TT>
    {
      public:
        LineIterator & operator++(); ///<prefix increment, steps along line
        LineIterator operator++(int);///<postfix increment, steps along line
        LineIterator & operator=(LineIterator const &other); ///<assignment operator
        bool operator==(LineIterator const &other) const;///< equality operator
        bool operator!=(LineIterator const &other) const {return !((*this)==other);} ///<inequality operator
        LineIterator(const LineIterator &other); ///< copy constructor
        LineIterator(void); ///< creates a end of line iterator
        inline int getRow() const {return row_;};
        inline int getCol() const {return col_;};
        inline TT & operator*() const {return *curCell_;};
        inline TT * operator->() const {return curCell_;};
        bool isValid(void); ///< reports true if the current cell is valid

      private:
        friend class Map;
        LineIterator(const IndexRange &range, const Map<TT> & map);
        TT *curCell_; ///< the current cell we're point to
        IndexRange range_; ///< the iterator will travel from rs,cs to re,ce
        bool offEnd_; ///< indicates the iterator has run its course
        bool almostOffEnd_; ///< a fix to make the line drawing inclusive of end points
        int row_,col_; ///< row and column index
        long numRows_; ///< number of rows in the source map
        long numCols_; ///< number of columns in the source map
        int dr_,dc_; ///< step directions for the iterator
        int fraction_; ///< ratio of row/column motion required
        int stepRow_, stepCol_; ///< the step direction for moving through the map
    };

    LineIterator beginLine(const RecPoint2D& start, 
                           const RecPoint2D& end) const;
    LineIterator endLine(void ) const;

    
    /**
     * @brief an iterator we can use to step over the map
     */
    class iterator: 
        public std::iterator<std::forward_iterator_tag, TT> 
    {
      public:
        iterator & operator++(); ///< pre increment operator
        iterator operator++(int); ///< post increment operator
        iterator & operator=(iterator const &other); ///< assignment operator
        bool operator==(iterator const &other) const;
        bool operator!=(iterator const &other) const;
        inline TT &operator*() const { return *curCell_;}; 
        inline TT *operator->() const {return curCell_;};
        inline int getRow() const {return row_;};
        inline int getCol() const {return col_;};
        iterator(const iterator & other);
        iterator(void); ///< this constructors sole purpose is to set the off end 

      private:
        friend class Map;
        iterator(const IndexRange & range, const Map<TT> & map);

        TT * curCell_; ///< a pointer to the current cell
        IndexRange range_; ///< the range over which the iterator is moving
        bool offEnd_;///< means the iterator is off the end of the range
        int row_,col_; ///< current row and column
        int columnStep_; ///< step when we get to the end of the column, to get to start of next row
    };

    iterator begin(void) const; 
    iterator begin(const RecPoint2D & min, 
                   const RecPoint2D & max) const;
    iterator begin(const RecAxisAlignedBox2D & box) const;
    iterator end(void) const;

    void fill(const TT & value);
    void fill(const RecAxisAlignedBox2D & box, const TT & value);

    void setOrigin(const RecPoint2D & ori);
    inline RecPoint2D getOrigin(void) {return origin_;};

    inline double getCellSize(void) const {return cellSize_m_;};
    inline double getWidth(void) const {return width_m_;};
    inline double getLength(void) const {return length_m_;};
    RecAxisAlignedBox2D getBounds(void) const;


    inline ptime getTimeStamp(void) const {return timeStamp_;};
    inline void setTimeStamp(const ptime & t) {timeStamp_ = t;};

    bool inBounds(const RecPoint2D & globalPt) const;
   
    void convertGlobalPointToIndices(const RecPoint2D & globalPt, long &row,  long &col) const;
       inline int getNumRows(void) const {return numRows_;};
    inline int getNumCols(void) const {return numCols_;};
    void convertIndicesToGlobalPoint(const int &row, const int &col, double &x, double &y) const;
    void convertIndicesToGlobalPoint(const double &row, const double &col, double &x, double &y) const;

    // These are the operators for getting and setting values in the map.  The usage is map(point) = value or
    // value = map(point);
    TT & operator()(const RecPoint2D & globalPt);
    const TT & operator()(const RecPoint2D & globalPt) const;
    
    // return a pointer to the cell at (row, col)
    const TT & operator()(int row, int col) const;
    TT & operator()(int row, int col);

    Map();
    Map(const Map & otherMap);
    Map(const RecPoint2D &origin, double length_m, double width_m, double cellSize_m);

    void operator=(const Map &otherMap);
    virtual ~Map(void);
    void setUseShallowCopy(void) {useShallowCopy_=true;};
    void setCompressOnSeriallize(void) {compressOnSerialize_ = true;};

    void calculateChangeMap(const Map & otherMap, Map<bool> &changeMap);
    
    void toIplImage(IplImage*& image); ///< BIG HACK! To convert to an IplImage from a ByteMap - WILL DIE IF USED WITH ANYTHING BUT char MAP!
    
  protected:
    double cellSize_m_; ///< the width/length of a cell
    double invCellSize_; ///< 1/cellSize_m_ 
    double width_m_; ///< the easting extent of the map
    double widthBy2_m_; ///< half the easting extent of the map
    double length_m_; ///< the northing extent of the map
    double lengthBy2_m_; ///< half the northing extent of the map
    TT * cells_; ///< the cell content of the map
    RecPoint2D origin_; ///< the origin of the map

    ptime timeStamp_; ///< a timestamp associated with the map

    long numRows_, numCols_; ///< the dimensions of the map in cells
    bool useShallowCopy_; ///< if true, the user has said the contained data type 
    bool compressOnSerialize_; ///< if true, data will be compressed before being serialized 
   
    
  private:

    int eastingsToCol(double mapE_m) const;
    int northingsToRow(double mapN_m) const;
    double colToEastings(int c) const;
    double rowToNorthings(int r) const;
    double colToEastings(double c) const;
    double rowToNorthings(double r) const;
       
    void mapSetup(const RecPoint2D &origin, double length_m, double width_m, double cellSize_m_, 
                  bool useShallowCopy =false, bool compressOnSerialize = false, bool needAlloc = true);
  
    friend class boost::serialization::access;
    template <class Archive>
    void load(Archive & ar, unsigned int version);
    template <class Archive>
    void save(Archive & ar, unsigned int version) const;
    BOOST_SERIALIZATION_SPLIT_MEMBER()
};

typedef Map<double> DoubleMap;
typedef Map<unsigned char> ByteMap;
typedef Map<bool> BoolMap;

/**
 * @brief converts map relative eastings to a column index
 */
template <class TT>
inline int Map<TT>::eastingsToCol(double mapE_m) const
{
    return static_cast<int>(floor((mapE_m+widthBy2_m_)*invCellSize_));
};

/**
 * @brief converts map relative northings to a row index
 */
template <class TT>
inline int Map<TT>::northingsToRow(double mapN_m) const
{
    return static_cast<int>(floor((mapN_m+lengthBy2_m_)*invCellSize_));
};

/**
 * @brief converts a column index to map relative eastings
 */
template <class TT>
inline double Map<TT>::colToEastings(int c) const
{
    return -widthBy2_m_ + c*cellSize_m_ + cellSize_m_/2;
};

/**
 * @brief converts a column position to map relative eastings
 */
template <class TT>
inline double Map<TT>::colToEastings(double c) const
{
    return -widthBy2_m_ + c*cellSize_m_ + cellSize_m_/2;
};

/**
 * @brief converts a row index to map relative northings
 */

template <class TT>
inline double Map<TT>:: rowToNorthings(int r) const
{
    return -lengthBy2_m_ + r*cellSize_m_ + cellSize_m_/2;
};

/**
 * @brief converts a row position to map relative northings
 */

template <class TT>
inline double Map<TT>:: rowToNorthings(double r) const
{
    return -lengthBy2_m_ + r*cellSize_m_ + cellSize_m_/2;
};

/**
 * @brief sets the map origin to a point near the passed origin that is a multiple of the specified cell size
 */
template <class TT>
inline void Map<TT>::setOrigin(const RecPoint2D & ori)
{
    int xCellCnt  = static_cast<int>(ori.x/cellSize_m_+0.5);
    int yCellCnt = static_cast<int>(ori.y/cellSize_m_+0.5);
    origin_.x = xCellCnt*cellSize_m_;
    origin_.y = yCellCnt*cellSize_m_;
};



/**
 * @brief generates a lineiterator that will step through the map from start to end inclusively
 *
 * @param start the location of the first cell for the iterator to touch
 * @param end the location of the last cell the iterator will touch
 */
template <class TT>
typename Map<TT>::LineIterator
Map<TT>::beginLine(const RecPoint2D & start,
                   const RecPoint2D & end) const
{
    IndexRange range;
    convertGlobalPointToIndices(start,range.rs,range.cs);
    convertGlobalPointToIndices(end,range.re,range.ce);
    return LineIterator(range,*this);
}

/**
 * @brief generates an end LineIterator that can be used to check if a LineIterator has come to the end of a line
 */
template <class TT>
typename Map<TT>::LineIterator
Map<TT>::endLine(void) const
{
     LineIterator ret;
     return ret;
}

/**
 * @brief creates an iterator that will step over the entire map
 */
template <class TT>
typename Map<TT>::iterator
Map<TT>::begin(void) const
{
    IndexRange range;
    range.rs = 0; range.cs = 0;
    range.re = numRows_; range.ce = numCols_;
    return iterator(range,*this);
};


/**
 * @brief returns an iterator that will step over the cells in the defined rectangular region
 *
 * @return an iterator that will step over the passed rectangular region
 *
 * @param min the most south westerly point of the map we wish to iterate over
 * @param max the most north easterly point of the map we with to iterate over 
 *
 * this system will return an iterator that steps over the rectangular region defined by min and max.  
 * The region is clipped to the map dimensions.
 */
template <class TT>
typename Map<TT>::iterator Map<TT>::begin(const RecPoint2D &min,
                                          const RecPoint2D &max) const
{
    IndexRange range;
    convertGlobalPointToIndices(min,range.rs,range.cs);
    convertGlobalPointToIndices(max,range.re,range.ce);
    range.normalize();
    range.rs = std::min(std::max(0L,range.rs),numRows_);
    range.re = std::min(std::max(0L,range.re),numRows_);
    range.cs = std::min(std::max(0L,range.cs),numCols_);
    range.ce = std::min(std::max(0L,range.ce),numCols_);

    return iterator(range,*this);
}

/**
 * @brief returns an iterator that will step over the cells in the defined rectangular region
 */
template <class TT>
typename Map<TT>::iterator Map<TT>::begin(const RecAxisAlignedBox2D & box) const
{
    return begin(box.getMinPoint(),box.getMaxPoint());
}

/**
 * @brief returns an iterator that defines the end of an iterated region
 */
template <class TT>
typename Map<TT>::iterator Map<TT>::end() const
{
    iterator end;
    return end;
}

/**
 * @brief calculates the row and column for a passed global point
 */
template <class TT>
void Map<TT>::convertGlobalPointToIndices(const RecPoint2D & globalPt,long &row, long &col) const
{
    col = eastingsToCol(globalPt.y-origin_.y);
    row = northingsToRow(globalPt.x-origin_.x);
}

/**
 * @brief calculates the global point position for given row and column indices
 */
template <class TT>
void Map<TT>::convertIndicesToGlobalPoint(const int &row, const int &col, double &x, double &y) const
{
    x = rowToNorthings(row) + origin_.x;
    y = colToEastings(col) + origin_.y;
}

/**
 * @brief calculates the global point position for given row and column position (not necessarily integer indices)
 */
template <class TT>
void Map<TT>::convertIndicesToGlobalPoint(const double &row, const double &col, double &x, double &y) const
{
    x = rowToNorthings(row) + origin_.x;
    y = colToEastings(col) + origin_.y;
}

/**
 * @brief checks wether the passed point is within the bound of the map
 */
template <class TT>
bool  Map<TT>::inBounds(const RecPoint2D & globalPt) const
{
    int row,col;
    col = eastingsToCol(globalPt.y-origin_.y);
    row = northingsToRow(globalPt.x-origin_.x);
    return ((col >= 0 && col <numCols_) && (row >= 0 && row < numRows_));
}

/**
 * @brief returns a reference to the content of the cell at the globalPt.  
 *
 * @return a reference to the cell
 *
 * @param globalPt the location we want to get a cell for
 *
 * if the point is out side of the map, theis function will fail an assertion so it is important to check inBounds 
 * before calling operator()
 */
template <class TT>
TT & Map<TT>::operator()(const RecPoint2D & globalPt) 
{
    int row, col;
    col = eastingsToCol(globalPt.y-origin_.y);
    row = northingsToRow(globalPt.x-origin_.x);
    BOOST_ASSERT(col >= 0 && col <numCols_);
    BOOST_ASSERT(row >= 0 && row < numRows_);
    return cells_[col+row*numCols_];
}

/**
 * @brief returns a const reference to the contetn of the cell at globalPt
 */
template <class TT>
const TT & Map<TT>::operator()(const RecPoint2D & globalPt) const
{
    int row, col;

    col = eastingsToCol(globalPt.y-origin_.y);
    row = northingsToRow(globalPt.x-origin_.x);
    BOOST_ASSERT(col >= 0 && col <numCols_);
    BOOST_ASSERT(row >= 0 && row < numRows_);
    return cells_[col+row*numCols_];
}

/**
 * @brief returns a reference to the contents of the cell at (row,col)
 */
template <class TT>
TT & Map<TT>::operator()(int row, int col)
{
    BOOST_ASSERT(col >= 0 && col <numCols_);
    BOOST_ASSERT(row >= 0 && row < numRows_);
    return cells_[col+row*numCols_];
}

/**
 * @brief returns a const reference to the contents of the cell at (row,col)
 */
template <class TT>
const TT & Map<TT>::operator()(int row, int col) const
{
    BOOST_ASSERT(col >= 0 && col <numCols_);
    BOOST_ASSERT(row >= 0 && row < numRows_);
    return cells_[col+row*numCols_];
}


template <class TT>
Map<TT>::Map(void)
{
    mapSetup(RecPoint2D(0,0),0,0,0);
}

/**
 * @brief constructs a map with it's center at the passed orign, and with given width, length and cell size.
 */
template <class TT>
Map<TT>::Map(const RecPoint2D &origin, double length_m, double width_m, double cellSize_m) 
{
    mapSetup(origin,length_m,width_m,cellSize_m);
}

/**
 * @brief assignment operator
 * if useShallowCopy is set, a memcopy is performed rather than an element by element copy.
 */
template <class TT>
void Map<TT>::operator=(const Map<TT> & otherMap)
{
    if (this==&otherMap)
        return;

    bool needReAlloc= (otherMap.numRows_!=numRows_) || (otherMap.numCols_!=numCols_);


    
    if (NULL!=cells_ || !needReAlloc)
        delete[] cells_;
    mapSetup(otherMap.origin_, otherMap.length_m_, 
             otherMap.width_m_,otherMap.cellSize_m_, needReAlloc);
    
    if (useShallowCopy_)
    {
        memcpy(static_cast<void*>(cells_),static_cast<void*>(otherMap.cells_),sizeof(TT)*numRows_*numCols_);
    }
    else
    {
        int numCells = numRows_*numCols_;
        for (int i=0;i<numCells;i++)
        {
            cells_[i] = otherMap.cells_[i];
        }
    }
}

/**
 * @brief copy constructor
 */
template <class TT>
Map<TT>::Map(const Map<TT> & otherMap)
{
    if (this==&otherMap)
        return;
 
    mapSetup(otherMap.origin_, otherMap.length_m_, otherMap.width_m_,otherMap.cellSize_m_);
    
    if (useShallowCopy_)
    {
        memcpy(static_cast<void*>(cells_),static_cast<void*>(otherMap.cells_),sizeof(TT)*numRows_*numCols_);
    }
    else
    {
        int numCells = numRows_*numCols_;
        for (int i=0;i<numCells;i++)
        {
            cells_[i] = otherMap.cells_[i];
        }
    }
}

/**
 * @brief virtual destructor
 */
template <class TT>
Map<TT>::~Map(void) 
{
    delete[] cells_;
} 

/**
 * @brief sets up the map and allocates memory 
 */
template <class TT>
void Map<TT>::mapSetup(const RecPoint2D & origin, double length_m, double width_m, double cellSize_m, bool useShallowCopy, bool compressOnSerialize, bool needAllocation)
{
    useShallowCopy_ = useShallowCopy; 
    compressOnSerialize_ = compressOnSerialize;

    if (cellSize_m ==0) {
        numRows_ = 0;
        numCols_ = 0;
        cells_ = NULL;
        cellSize_m_ =0.0;
        invCellSize_ = 0;
        width_m_=0.0;
        widthBy2_m_ = 0.0;
        length_m_ =0.0;
        lengthBy2_m_= 0.0;
  } else {
      numRows_ = (int)floor(length_m/cellSize_m +0.5);
      numCols_ = (int)floor(width_m/cellSize_m + 0.5);
      if (needAllocation)
          cells_ = new TT[numRows_*numCols_];
      
      cellSize_m_ = cellSize_m;
      invCellSize_ = 1/cellSize_m_;
      width_m_ = numCols_*cellSize_m_;
      widthBy2_m_ = width_m_/2.0;
      length_m_ = numRows_ *cellSize_m_;
      lengthBy2_m_ = length_m_/2.0;
  }
 
    setOrigin(origin);
}

/**
 * @brief returns the bounds of the map
 */
template <class TT>
RecAxisAlignedBox2D  Map<TT>::getBounds(void) const
{
    RecAxisAlignedBox2D  ret(origin_.x-widthBy2_m_,origin_.x+widthBy2_m_,
                             origin_.y-lengthBy2_m_,origin_.y+lengthBy2_m_);
    return ret;
}


/**
 * @brief fills all cells of the map with the given value
 */
template <class TT>
void Map<TT>::fill(const TT & value) 
{
    for (iterator it= begin(); it!=end();++it)
        *it =value;
}

/**
 * @brief fills the passed axis aligned bounding box
 */
template <class TT>
void Map<TT>::fill(const RecAxisAlignedBox2D & box, const TT & value)
{
    for (iterator it=begin(box); it!=end(); ++it) 
        *it = value;
}


/**
 * @brief calculates areas where data may have changed.
 *
 * @param otherMap to which this map is compared.  
 *
 * @param changeMap the map is filled with true or false indicating if the region has changed in otherMap compared to 
 * this map
 *
 * Any place the otherMap either differs from this map or any area where othermap doesn't overlap with this map will be 
 * marked as changed in changeMap
 */
template <class TT>
void Map<TT>::calculateChangeMap(const Map<TT> & otherMap, Map<bool> & changeMap)
{
    // anywhere the othermap overlaps the change map, there could be change
    changeMap.fill(otherMap.getBounds(),true);


    //calculate the area that is common to both maps and the change map
    RecAxisAlignedBox2D  overlapRegion = changeMap.getBounds().
        overlapArea(
            otherMap.getBounds()
            ).overlapArea(getBounds());

    // now iterate over the the three maps, setting the changed flag to true anywhere the maps differ
    iterator myIt, otherIt;
    Map<bool>::iterator changeIt;
    myIt = begin(overlapRegion);
    otherIt = otherMap.begin(overlapRegion);
    changeIt = changeMap.begin(overlapRegion);
    
    BOOST_ASSERT(changeMap.getCellSize()==getCellSize() &&
                 otherMap.getCellSize() == getCellSize() &&
                 "maps must have the same cellsizes");

    for (/**/;myIt!=end();++myIt, ++otherIt, ++changeIt) 
        *changeIt = *myIt!=*otherIt;

}

template<class TT>
void Map<TT>::toIplImage(IplImage*& image)
{
  // deallocate if the image is the wrong size - note that this assumes that
  // the image is either allocated and set up, but set up for a different sized
  // map, or it is not allocated at all.  Behavior is undefined if just the
  // header is passed in.
  if(image && 
     (image->width != numRows_ || image->height != numCols_)
     )
    {
      cvReleaseImage(&image);
    }

  // if the image is null (released from above or never allocated), then 
  // allocate the header and the image
  if(!image)
    {
      CvSize sze = cvSize(numRows_, numCols_);

      image = cvCreateImage(sze, IPL_DEPTH_8U, 1);
    }

  cvSetImageData(image, static_cast<void*>(cells_), numCols_);
}

/**
 * @brief tha map serialization save function
 */
template <class TT>
template <class Archive>
void Map<TT>::save(Archive &ar, unsigned int version) const
{
    static unsigned char *compressBuffer=NULL; // used to keep around a compression buffer that is only allocated once.
    static unsigned long compressBufferSize=0;


    ar << cellSize_m_ << width_m_
       << length_m_ << origin_ << timeStamp_ 
       << useShallowCopy_ << compressOnSerialize_;

    if (!compressOnSerialize_)
    {
        if (useShallowCopy_)
        {
            ar << boost::serialization::make_binary_object(static_cast<void *>(cells_),numRows_*numCols_*sizeof(TT));
        }
        else 
        {
            for (int i=0; i< numRows_ * numCols_; i++) 
                ar << const_cast<const TT *>(cells_)[i];
        }
    }
    else
    {
        unsigned long uncompressedMapSize = sizeof(TT)*numCols_*numRows_;
        unsigned long neededCompressBufferSize = compressBound(uncompressedMapSize);
        // check if we need to make our buffer bigger
        if (neededCompressBufferSize> compressBufferSize)
        {
            if (compressBuffer)
                delete[] compressBuffer;
            compressBuffer = new unsigned char[neededCompressBufferSize];
            compressBufferSize = neededCompressBufferSize;
        }
        
        // now we do the compression
        unsigned long compressedDataSize = compressBufferSize;
        int zerror;
        zerror = compress(compressBuffer, &compressedDataSize,
                          reinterpret_cast<unsigned char* >(cells_), uncompressedMapSize);
        if (zerror!=Z_OK)
            std::cerr << "compression error!" << std::endl;
        ar << compressedDataSize;
        ar << boost::serialization::make_binary_object(static_cast<void *>(compressBuffer),
                                                       compressedDataSize);
        
    }



}

/**
 * @brief the map serialization load function
 */
template <class TT>
template < class Archive>
void Map<TT>::load(Archive &ar, unsigned int version) 
{
    static unsigned char * uncompressBuffer =NULL;
    static unsigned long uncompressBufferSize =0;
    
    double oldCellSize, oldWidth, oldLength;
    oldCellSize= cellSize_m_; oldWidth = width_m_; oldLength = length_m_;


    ar >> cellSize_m_  >> width_m_
       >> length_m_ >> origin_ >> timeStamp_
       >> useShallowCopy_ >> compressOnSerialize_;


    mapSetup(origin_,length_m_,width_m_,cellSize_m_,useShallowCopy_, compressOnSerialize_,
             !(oldCellSize==cellSize_m_ && oldWidth ==width_m_ && oldLength ==length_m_));

    if (!compressOnSerialize_)
    {
        if (useShallowCopy_)
        {
            ar >> boost::serialization::make_binary_object(static_cast<void *>(cells_),numRows_*numCols_*sizeof(TT));
        }
        else 
        {
        for (int i=0; i< numRows_ * numCols_; i++) 
            ar >> cells_[i];
        }
    } 
    else
    {
        unsigned long compressedDataSize;
        unsigned long uncompressedMapSize = sizeof(TT)*numRows_*numCols_;
        int zerror;
        ar >> compressedDataSize;
        if (compressedDataSize > uncompressBufferSize)
        {
            if (uncompressBuffer)
                delete[] uncompressBuffer;
            // we don't bother allocating just to the size needed, we allocate to the maxium data size
            // for this map size- the assumption is we will probably be sending several maps this size
            // and we don't want to reallocate, just because the first map we get is efficiently compressed
            uncompressBuffer = new unsigned char[compressBound(uncompressedMapSize)];
        }
        ar >> boost::serialization::make_binary_object(static_cast<void *>(uncompressBuffer),
                                                       compressedDataSize);
        zerror =uncompress(reinterpret_cast<unsigned char*>(cells_),&uncompressedMapSize,
                           uncompressBuffer,compressedDataSize);
        if (zerror!=Z_OK)
            std::cerr << "uncompression error!" << std::endl;

    }

}




#include "Map.def.h"
#endif //ifndef _MAP_H_
