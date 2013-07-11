
#ifndef _MAP_H_
#define _MAP_H_

#include "../RecGeometry.h"
#include "TimeStamp.h"
#include <iostream>
#include <iterator>


//#include <zlib.h>
#include <opencv/cv.h>

/**
 * @brief a set of indices bounding some part of a map
 */
struct IndexRange
{
    long rs, re; ///<row index ranges, should be considered rs<=range<re
    long cs, ce; ///<column index ranges, should be considered cs<=range<ce
    void normalize(); //! ensures that rs<=re & cs<=ce
};

/**
 * @brief this ensures that row and column starts are smaler than row and colum ends
 */
inline void IndexRange::normalize(void)
{
    if (rs > re)
    {
        int tmp = rs;
        rs = re;
        re = tmp;
    }
    if (cs > ce)
    {
        int tmp = cs;
        cs = ce;
        ce = tmp;
    }
}



/**
 * @brief a templatized 2D map class that allows for indexing by global coordinates
 * @ingroup perceptionTypesGroup
 */
template <class TT> 
class Map 
{
public:
    class LineIterator: public std::iterator<std::forward_iterator_tag, TT>
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
    LineIterator beginLine(const RecPoint2D& start, const RecPoint2D& end) const;
    LineIterator endLine(void ) const;

    
    /**
     * @brief an iterator we can use to step over the map
     */
    class iterator: public std::iterator<std::forward_iterator_tag, TT>
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
    iterator begin(const RecPoint2D & min, const RecPoint2D & max) const;
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
/*
    friend class boost::serialization::access;
    template <class Archive>
    void load(Archive & ar, unsigned int version);
    template <class Archive>
    void save(Archive & ar, unsigned int version) const;
    BOOST_SERIALIZATION_SPLIT_MEMBER()
*/
};

typedef Map<double> DoubleMap;
typedef Map<unsigned char> ByteMap;
typedef Map<bool> BoolMap;



#include "Map.def.h"
#include "Map.def2.h"

#endif //ifndef _MAP_H_
