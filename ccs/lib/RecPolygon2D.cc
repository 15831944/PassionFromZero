
#include "RecGeometry.h"
#include <algorithm>

/*****************************************************************************
 * PUBLIC FUNCTIONS
 *****************************************************************************/
/*****************************************************************************
 *    FUNCTION: RecPolygon2D
 * DESCRIPTION: constructor for recPolygon2D
 *****************************************************************************/
RecPolygon2D::RecPolygon2D():
    verticesP ( ),
    center(0, 0),
    min(0, 0),
    max(0, 0),
    angleToVertexP ( )
{
}

/*****************************************************************************
 *    FUNCTION: RecPolygon2D
 * DESCRIPTION: copy constructor for RecPolygon2D
 *****************************************************************************/
RecPolygon2D::RecPolygon2D(const RecPolygon2D &poly)
    : verticesP(poly.verticesP),
      center(poly.center),
      min(poly.min),
      max(poly.max),
      angleToVertexP(poly.angleToVertexP)
{
    updateCenterAndBBox();
}

/***********************************************************************************************
 * \brief Anonymous template batch constructor for a vector of anything with .x and .y members
 * \param vertices the vertices to add.  Vertices will be the 2D projection of the passed points.
 *
 * This is for those who are making huge polygons and don't want to stuff the vertices in one at
 * a time, which is otherwise an O(n^2) operation due to internal bookkeeping.
 ***********************************************************************************************/
template <class Pt>
RecPolygon2D::RecPolygon2D(const std::vector<Pt> &vertices):
    verticesP(),
    center(0, 0),
    min(0, 0),
    max(0, 0),
    angleToVertexP()
{
    verticesP.reserve(vertices.size());
    for(typename std::vector<Pt>::const_iterator ii = vertices.begin();
        ii != vertices.end(); ++ii)
    {
        verticesP.push_back(RecPoint2D(ii->x,ii->y));
    }

    // make sure we have a closed loop
    if(verticesP.size() > 0 && verticesP[0] != verticesP[verticesP.size() - 1])
    {
        verticesP.push_back(RecPoint2D(vertices[0].x,vertices[0].y));
    }

    // perform extra work to maintain internal invariants
    updateCenterAndBBox();
    updateVertexAngles();
}


/*****************************************************************************
 *    FUNCTION: ~RecPolygon2D
 * DESCRIPTION: destructor for RecPolygon2D
 *****************************************************************************/
RecPolygon2D::~RecPolygon2D()
{
}

/*****************************************************************************
 *    FUNCTION: getNumVertices
 * DESCRIPTION: returns the number of vertices
 *****************************************************************************/
int RecPolygon2D::getNumVertices() const
{
    if (!verticesP.size())
        return 0;
    else
        return verticesP.size() - 1;
}

/*****************************************************************************
 *    FUNCTION: getVertex
 * DESCRIPTION: returns a vertex given an index. Returns -1 if error
 *****************************************************************************/
int RecPolygon2D::getVertex(int index, RecPoint2D &vertex) const
{
    if (index < 0 || index > (int)verticesP.size()) {
        vertex.x = 0;
        vertex.y = 0;
        return -1;
    }

    vertex = verticesP[index];

    return 0;
}

/*****************************************************************************
 *    FUNCTION: getCenter
 * DESCRIPTION: returns the center of the polygon
 *****************************************************************************/
void RecPolygon2D::getCenter(RecPoint2D &centerExt) const
{
    centerExt = center;
}

/*****************************************************************************
 *    FUNCTION: getBoundingBox
 * DESCRIPTION: returns the bounding box of the polygon (min and max)
 *****************************************************************************/
void RecPolygon2D::getBoundingBox(RecPoint2D& minExt, RecPoint2D& maxExt) const
{
    minExt = min;
    maxExt = max;
}

/*****************************************************************************
 *    FUNCTION: addVertex
 * DESCRIPTION: adds a new vertex to a polygon
 *****************************************************************************/
void RecPolygon2D::addVertex(const RecPoint2D& vertex)
{
    /*
     * check if new vertex is within tolerance of existing vertices
     */
    if (verticesP.size())
        // we don't need to check the last vertex since it is a copy of the first
        for (unsigned int i = 0; i < verticesP.size() - 1; i++) {
            if (((vertex.x - verticesP[i].x) *
                 (vertex.x - verticesP[i].x) +
                 (vertex.y - verticesP[i].y) *
                 (vertex.y - verticesP[i].y)) < 1.0e-6) {
              return;
            }
        }

    if (verticesP.size())
        verticesP.pop_back();
    verticesP.push_back(vertex);
    verticesP.push_back(verticesP[0]);

    updateCenterAndBBox();
    updateVertexAngles();
}

/*****************************************************************************
 *    FUNCTION: sortVerticesCCW
 * DESCRIPTION: sorts vertices in counter clockwise order
 *****************************************************************************/
void RecPolygon2D::sortVerticesCCW()
{
    if (!getNumVertices())
        return;

    /*
     * update center, bounding box, and angles
     */
    updateCenterAndBBox();
    updateVertexAngles();

    /*
     * sort vertices by angle
     */
    for (unsigned int i = 0; i < angleToVertexP.size(); i++) {
        double minAngleValue = angleToVertexP[i];
        unsigned int minAngleIndex = i;

        /*
         * find the min angle from i to the end of array
         */
        for (unsigned int j = i + 1; j < angleToVertexP.size(); j++) {
            if (angleToVertexP[j] < minAngleValue) {
                minAngleValue = angleToVertexP[j];
                minAngleIndex = j;
            }
        }

        /*
         * swap the data from minAngleIndex to i
         */
        if (minAngleIndex != i) {
            double temp = angleToVertexP[i];
            angleToVertexP[i] = angleToVertexP[minAngleIndex];
            angleToVertexP[minAngleIndex] = temp;

            RecPoint2D tempPoint = verticesP[i];
            verticesP[i] = verticesP[minAngleIndex];
            verticesP[minAngleIndex] = tempPoint;
        }
    }

    /*
     * copy the first vertex into the last (for easier computation)
     */
    verticesP[verticesP.size() - 1] = verticesP[0];
}

/*****************************************************************************
 *    FUNCTION: clearVertices
 * DESCRIPTION: clears vertices
 *****************************************************************************/
void RecPolygon2D::clearVertices()
{
    verticesP.clear();
    angleToVertexP.clear();

    center.x = 0;
    center.y = 0;

    min.x = 0;
    max.x = 0;
    min.y = 0;
    max.y = 0;
}

/*****************************************************************************
 *    FUNCTION: containsPoint
 * DESCRIPTION: returns 1 if point is in polygon, 0 if not
 *****************************************************************************/
int RecPolygon2D::containsPoint(const RecPoint2D& point) const
{
    unsigned int i, j;
    int c = 0;

    // a poly with less than two vertices contains nothing
    if (getNumVertices() < 2)
        return 0;

    // do a bounding box check before our O(N) check
    if(point.x >= min.x && point.y >= min.y &&
       point.x <= max.x && point.y <= max.y)
    {
        for (i = 0, j = verticesP.size() - 2; i < verticesP.size() - 1; j = i++) {
            if (((verticesP[i].y <= point.y) && (point.y < verticesP[j].y)) ||
                ((verticesP[j].y <= point.y) && (point.y < verticesP[i].y)))
            {
                if (point.x < ((verticesP[j].x - verticesP[i].x) *
                               (point.y - verticesP[i].y) /
                               (verticesP[j].y - verticesP[i].y) + verticesP[i].x))
                {
                    c = !c;
                }
            }
        }
    }

    return c;
}

/*****************************************************************************
 *    FUNCTION: hasOverlap
 * DESCRIPTION: returns 1 if polygon overlaps with given polygon, 0 if not
 *****************************************************************************/
int RecPolygon2D::hasOverlap(const RecPolygon2D& poly) const
{
    unsigned int n1, n2;
    unsigned int i1, i2;
    unsigned int j1, j2;
    RecPoint2D minPoly, maxPoly;
    RecPoint2D v1;
    RecPoint2D inter, intIfLine;
    RecLineSegment2D l1;
    RecLineSegment2D l2;

    n1 = this->getNumVertices();
    n2 = poly.getNumVertices();

    /*
     * do an idiot test
     */
    if(n1 == 0 || n2 == 0)
    {
        /* no overlap with empty polygons */
        return 0;
    }

    /*
     * do a bounding box test
     */
    poly.getBoundingBox(minPoly, maxPoly);

    if (min.x > maxPoly.x || maxPoly.x < min.x) {
        return 0;
    }

    if (min.y > maxPoly.y || maxPoly.y < min.y) {
        return 0;
    }

    /*
     * actual testing of intersections and interior vertices
     */
    for (i1 = verticesP.size() - 2, i2 = 0; i2 < verticesP.size() - 1; i1 = i2++) {
        for (j1 = n2 - 1, j2 = 0; j2 < n2; j1 = j2++) {
            l1.setPoints(verticesP[i1], verticesP[i2]);

            RecPoint2D p1, p2;
            poly.getVertex(j1, p1);
            poly.getVertex(j2, p2);
            l2.setPoints(p1, p2);

            if (l1.getIntersection(l2, inter, intIfLine)) {
                return 1;
            }
        }
    }

    /*
     * not sure what this does
     * NOTE: This checks to see if all of one poly's points are in the other.
     *       Only one point from each set is needed
     */
    if (poly.containsPoint(verticesP[verticesP.size() - 2])) {
        return 1;
    }

    poly.getVertex(n2-1, v1);

    if (containsPoint(v1)) {
        return 1;
    }

    return 0;
}

/*****************************************************************************
 *    FUNCTION: computeArea
 * DESCRIPTION: computes the area of the polygon
 *****************************************************************************/
double RecPolygon2D::computeArea() const
{
    double area = 0.0;

    if (verticesP.size())
        for (unsigned int i = 0; i < verticesP.size() - 1; i++) {
            area += (verticesP[i].x   * verticesP[i+1].y - verticesP[i+1].x * verticesP[i].y);
        }

    return (fabs(0.5 * area));
}

/*****************************************************************************
 *    FUNCTION: computeIntersection
 * DESCRIPTION: computes the intersection of two overlapping polygons.
 *              Returns intersection in "this"
 *****************************************************************************/
void RecPolygon2D::computeIntersection(RecPolygon2D& poly1, RecPolygon2D& poly2)
{
    int i, j;
    int n1, n2;
    int numIntersections;
    RecPoint2D v;
    RecPoint2D inter;
    RecPoint2D intIfLine;
    RecLineSegment2D l1;
    RecLineSegment2D l2;

    /*
     * initialize polygon
     */
    clearVertices();

    /*
     * sort vertices in each polygon
     */
    poly1.sortVerticesCCW();
    poly2.sortVerticesCCW();

    /*
     * get number of vertices
     */
    n1 = poly1.getNumVertices();
    n2 = poly2.getNumVertices();

    /*
     * make sure there are enough vertices
     */
    if (n1 < 2 || n2 < 2) {
        return;
    }

    /*
     * check for vertices of one polygon that fall inside the other
     */
    for (i=0; i<n1; i++)
    {
        poly1.getVertex(i, v);
        if (poly2.containsPoint(v))
        {
            addVertex(v);
        }
    }

    for (i=0; i<n1; i++)
    {
        poly2.getVertex(i, v);
        if (poly1.containsPoint(v))
        {
            addVertex(v);
        }
    }

    /*
     * brute force check for intersections of line segments
     */
    for (i=0; i<n1; i++) {
        RecPoint2D p1, p2;
        poly1.getVertex(i, p1);
        poly1.getVertex(i+1, p2);
        l1.setPoints(p1, p2);

        for (j=0; j<n2; j++) {

            poly2.getVertex(j, p1);
            poly2.getVertex(j+1, p2);
            l2.setPoints(p1, p2);

            numIntersections = l1.getIntersection(l2, inter, intIfLine);

            if (numIntersections == 1)
            {
                addVertex(inter);
            }

            if (numIntersections == 2)
            {
                addVertex(intIfLine);
            }
        }
    }

    sortVerticesCCW();
}


/*****************************************************************************
 *    FUNCTION: computeUnion
 * DESCRIPTION: computes the union of two overlapping polygons. Returns
 *              the union in "this"
 *****************************************************************************/
void RecPolygon2D::computeUnion(RecPolygon2D& poly1, RecPolygon2D& poly2)
{
    int i, j;
    int n1, n2;
    int numIntersections;
    RecPoint2D v;
    RecPoint2D inter;
    RecPoint2D intIfLine;
    RecLineSegment2D l1;
    RecLineSegment2D l2;

    /*
     * initialize polygon
     */
    clearVertices();

    /*
     * sort vertices in each polygon
     */
    poly1.sortVerticesCCW();
    poly2.sortVerticesCCW();

    /*
     * get number of vertices
     */
    n1 = poly1.getNumVertices();
    n2 = poly2.getNumVertices();

    /*
     * make sure there are enough vertices
     */
    if (n1 < 2 || n2 < 2) {
        return;
    }

    /*
     * check for vertices of one polygon that fall outside the other
     * add to union
     */
    for (i=0; i<n1; i++)
    {
        poly1.getVertex(i, v);
        if (poly2.containsPoint(v) == 0) {
            addVertex(v);
        }
    }

    for (i=0; i<n2; i++)
    {
        poly2.getVertex(i, v);
        if (poly1.containsPoint(v) == 0)
        {
            addVertex(v);
        }
    }

    /*
     * brute force check for intersections of line segments
     */
    for (i=0; i<n1; i++) {
        RecPoint2D p1, p2;
        poly1.getVertex(i, p1);
        poly1.getVertex(i+1, p2);
        l1.setPoints(p1, p2);


        for (j=0; j<n2; j++) {

            poly2.getVertex(j, p1);
            poly2.getVertex(j+1, p2);
            l2.setPoints(p1, p2);

            numIntersections = l1.getIntersection(l2, inter, intIfLine);

            if (numIntersections == 1) {
                addVertex(inter);
            }

            if (numIntersections == 2) {
                addVertex(intIfLine);
            }
        }
    }

    sortVerticesCCW();
}



class RecPoint2DLessY
{
  public:
    bool operator()(const RecPoint2D &lhs, const RecPoint2D &rhs)
        {
            if (lhs.y == rhs.y)
                return lhs.x < rhs.x;
            return lhs.y < rhs.y;
        }

};

unsigned int RecPolygon2D::findSmallestAnglePositive(vector<RecPoint2D> & pts, unsigned int startIndex)
{
    double smallestTheta = 2*M_PI;
    unsigned int retIndex =0;
    double x = pts[startIndex].x;
    double y = pts[startIndex].y;
    for (unsigned int ii=startIndex+1; ii<pts.size();ii++)
    {
        double theta = atan2(pts[ii].y-y,pts[ii].x-x);
        if (theta < smallestTheta)
        {
            retIndex = ii;
            smallestTheta = theta;
        }
    }

    return retIndex;
}

unsigned int RecPolygon2D::findSmallestAngleNegative(vector<RecPoint2D> & pts, unsigned int startIndex)
{
    double smallestTheta = 2*M_PI;
    unsigned int retIndex =0;
    double x = pts[startIndex].x;
    double y = pts[startIndex].y;
    for (int ii=startIndex-1; ii>=0;ii--)
    {
        double theta = atan2(-(pts[ii].y-y),-(pts[ii].x-x));
        if (theta < smallestTheta)
        {
            retIndex = ii;
            smallestTheta = theta;
        }
    }

    return retIndex;
}




vector<RecPoint2D> RecPolygon2D::jarvisMarch(const vector<RecPoint2D> &inPts)
{
    vector<RecPoint2D> ret, pts;
    pts = inPts;

    // sort the points in increasing Y order
    sort(pts.begin(),pts.end(),RecPoint2DLessY());
    unsigned int currentIndex;
    currentIndex = 0;
    ret.push_back(pts[0]);
    while (currentIndex != pts.size()-1)
    {
        currentIndex = findSmallestAnglePositive(pts,currentIndex);
        ret.push_back(pts[currentIndex]);
    }

    while (currentIndex != 0)
    {
        currentIndex = findSmallestAngleNegative(pts,currentIndex);
        ret.push_back(pts[currentIndex]);
    }

    return ret;
}

/**
* @brief computes the convex hull of the points stored in the polygon using 
* jarvis march
*/
void RecPolygon2D::computeConvexHull(RecPolygon2D &hull)
    {
    if (getNumVertices() < 2) {
        return;
    }

    hull.clearVertices();

    vector<RecPoint2D> hullPts = jarvisMarch(verticesP);

    for (vector<RecPoint2D>::iterator it=hullPts.begin(); it!=hullPts.end(); ++it)
        hull.addVertex(*it);

    hull.sortVerticesCCW();

}



/*****************************************************************************
 *    FUNCTION: computeMinEnclosingRectangle
 * DESCRIPTION: computes the best rectangular bounding box. Assumes the
 *              input polygon is CONVEX. Returns rectangle in "this"
 *****************************************************************************/
void RecPolygon2D::computeMinEnclosingRectangle(const RecPolygon2D& poly)
{
    int i, i1, i2;
    int n;
    double bestTheta=0, theta;
    double st, ct;
    double smallestArea = -1;
    double area;
    double x, y;
    double minX, minY;
    double maxX, maxY;
    double bminX=0, bminY=0;
    double bmaxX=0, bmaxY=0;
    RecPoint2D v, v1, v2;

    clearVertices();

    n = poly.getNumVertices();

    /*
     * brute force check with candidate rectangles aligned at each edge
     */
    for (i1=n-1, i2=0; i2<n; i1=i2++) {
        poly.getVertex(i1, v1);
        poly.getVertex(i2, v2);

        theta = atan2((v2.y - v1.y), (v2.x - v1.x));

        st = sin(theta);
        ct = cos(theta);

        poly.getVertex(0, v);

        minX = maxX = ct * v.x + st * v.y;
        minY = maxY = -st * v.x + ct * v.y;

        for (i=1; i<n; i++) {
            poly.getVertex(i, v);

            x = ct * v.x + st * v.y;
            y = -st * v.x + ct * v.y;

            if (x > maxX) maxX = x;
            if (x < minX) minX = x;
            if (y > maxY) maxY = y;
            if (y < minY) minY = y;
        }

        area = (maxY - minY) * (maxX - minX);

        if (area < smallestArea || smallestArea == -1) {
            smallestArea = area;
            bestTheta = theta;
            bminX = minX;
            bminY = minY;
            bmaxX = maxX;
            bmaxY = maxY;
        }
    }

    st = sin(bestTheta);
    ct = cos(bestTheta);

    v.x = ct * bminX - st * bminY;
    v.y = st * bminX + ct * bminY;
    addVertex(v);

    v.x = ct * bmaxX - st * bminY;
    v.y = st * bmaxX + ct * bminY;
    addVertex(v);

    v.x = ct * bmaxX - st * bmaxY;
    v.y = st * bmaxX + ct * bmaxY;
    addVertex(v);

    v.x = ct * bminX - st * bmaxY;
    v.y = st * bminX + ct * bmaxY;
    addVertex(v);

    sortVerticesCCW();
}

/*****************************************************************************
 * PRIVATE FUNCTIONS
 *****************************************************************************/
/*****************************************************************************
 *    FUNCTION: updateCenterAndBBox
 * DESCRIPTION: updates the center and bounding box of the polygon
 *****************************************************************************/
void RecPolygon2D::updateCenterAndBBox()
{
    if (getNumVertices() < 2) {
        return;
    }

    /*
     * compute the bounding box
     */
    min.x = (float) HUGE;
    min.y = (float) HUGE;
    max.x = (float) -HUGE;
    max.y = (float) -HUGE;

    for (unsigned int i = 0; i < verticesP.size() - 1; i++) {
        if (verticesP[i].x < min.x) min.x = verticesP[i].x;
        if (verticesP[i].x > max.x) max.x = verticesP[i].x;
        if (verticesP[i].y < min.y) min.y = verticesP[i].y;
        if (verticesP[i].y > max.y) max.y = verticesP[i].y;
    }

    /*
     * the center is taken to be the center of the bounding box
     */
    center.x = (min.x + max.x) / 2.0;
    center.y = (min.y + max.y) / 2.0;
}

/*****************************************************************************
 *    FUNCTION: updaateVertexAngle
 * DESCRIPTION: updates the angle of the vertex. The angle is the angle
 *              formed by the line from center to vertex, and horizontal
 *****************************************************************************/
void RecPolygon2D::updateVertexAngles()
{
    angleToVertexP.resize(getNumVertices());

    if (getNumVertices() == 1) {
        angleToVertexP[0] = 0.0;
        return;
    }

    for (unsigned int i = 0; i < angleToVertexP.size(); i++) {
        angleToVertexP[i] = atan2((verticesP[i].y - center.y),
                                  (verticesP[i].x - center.x));
    }
}

/*****************************************************************************
 *    FUNCTION: isConvexCorner
 * DESCRIPTION: returns 1 if corner is convex, 0 otherwise
 *****************************************************************************/
int RecPolygon2D::isConvexCorner(const RecPoint2D& p1, const RecPoint2D& p2,
                                 const RecPoint2D& p3)
{
    double area=0.0;

    area += p1.x * p2.y - p2.x * p1.y;
    area += p2.x * p3.y - p3.x * p2.y;
    area += p3.x * p1.y - p1.x * p3.y;

    return (area > 0);
}


/*****************************************************************************
 *    FUNCTION: operator=
 * DESCRIPTION:
 *****************************************************************************/
RecPolygon2D&
RecPolygon2D::operator=(const RecPolygon2D& p)
{
    verticesP = p.getVertices();
    updateCenterAndBBox();
    updateVertexAngles();
    return *this;
}

/**\brief Compute the distance from the point to the polygon.
 * \param point The query point
 * \return The distance to the specified point, 0.0 if the point is inside the polygon or the polygon is empty
 */
double RecPolygon2D::distanceToPoint(const RecPoint2D &point) const
{
    unsigned int i,j;

    // a poly with less than two vertices contains nothing
    if (getNumVertices() < 2)
        return 0.0;

    // check for inside this is O(N)
    if(this->containsPoint(point))
        return 0.0;

    // seed with the distance to the first point
    double minDistance = point.distance(verticesP[0]);

    // iterate over the line segments, the closest distance is our distance
    for (i = 0, j = verticesP.size() - 2; i < verticesP.size() - 1; j = i++)
    {
        // the line segment is j->i
        RecLineSegment2D segment(verticesP[j],verticesP[i]);

        double dd = segment.getDistanceToSegment(point);
        if(dd < minDistance)
            minDistance = dd;
    }

    return minDistance;
}


/**
 *  creates a copy of the polygon trasnformed by the trans argument.
 */
RecPolygon2D RecPolygon2D::transform(const RecTransform2D & trans)
{
    std::vector<RecPoint2D> newVertices;
    newVertices.insert(newVertices.begin(), verticesP.begin(), verticesP.end()-1);

    for (unsigned int ii=0; ii < newVertices.size(); ++ii)
    {
        newVertices[ii] = trans*newVertices[ii];
    }
    
    return RecPolygon2D(newVertices);
}
