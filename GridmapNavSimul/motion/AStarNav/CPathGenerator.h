#ifndef CPATHGENERATOR_H_
#define CPATHGENERATOR_H_

#include "TMotionPoint.h"
#include <stdio.h>
#include <vector>
#include <list>

using namespace std;

namespace motion
{
    class CPathGenerator
    {
    public:
        CPathGenerator(void);
        virtual ~CPathGenerator(void) {}

        /** Calculate the path from start point to target point.
         * @param start Start point.
         * @param target Target point.
         * @param isIgnoreCorner Is ignore corner obstacle.
         * @return point list which is exactly the path from start to target. NULL if no path found.
         */
        list<TMotionPoint*> calcPath(TMotionPoint* start,
                                     TMotionPoint* target,
                                     bool isIgnoreCorner);
        /** Get surrounding points as a vector.
         * @param curPoint Current point.
         * @param isIgnoreCorner Is ignore corner obstacle.
         * @return A vector that contains all of the surrounding points of current point.
         * @see canReach
         */
        vector<TMotionPoint*> surrounding(TMotionPoint* curPoint, bool isIgnoreCorner);
        /** If point is found in openList.
         * @param tmpStartPoint The point in the center.
         * @param curPoint Current point(it's one of the surrounding points).
         * @see calcG
         * @see calcH
         */
        void foundPoint(TMotionPoint* tmpStartPoint, TMotionPoint* curPoint);
        /** If point is not found in openList.
         * @param tmpStartPoint The point in the center.
         * @param target Target point.
         * @param curPoint Current point(it's one of the surrounding points).
         * @see calcG
         * @see calcH
         */
        void notFoundPoint(TMotionPoint* tmpStartPoint, TMotionPoint* target, TMotionPoint* curPoint);
        /** Is the given point reachable.
         * @param curPoint Current point.
         * @param x x axis of the point evaluated.
         * @param y y axis of the point evaluated.
         * @param isIgnoreCorner Is ignore corner obstacle.
         * @return A vector that contains all of the surrounding points of current point.
         * @see canReach
         * @see surrounding
         */
        bool canReach(TMotionPoint* curPoint, int x, int y, bool isIgnoreCorner);
        /** Is the given point reachable.
         * @param x x axis of the point evaluated.
         * @param y y axis of the point evaluated.
         */
        bool canReach(int x, int y);
        /** Calculate G value.
         * @param tmpStartPoint The point in the center.
         * @param curPoint Current point(it's one of the surrounding points).
         */
        int calcG(TMotionPoint* tmpStartPoint,TMotionPoint* curPoint);
        /** Calculate H value.
         * @param target The targe point.
         * @param curPoint Current point(it's one of the surrounding points).
         */
        int calcH(TMotionPoint* target,TMotionPoint* curPoint);
        /** Find the point with minimum F.
         * @return Point in openList with the minimum F.
         */
        TMotionPoint* findMin(void);
        /** Assert whether a point in the given list.
         * @param list A list to search.
         * @param point The wanted point.
		 * @return Ture if the given point is in the given list.
		 */
        TMotionPoint* isExist(list<TMotionPoint*> &list, TMotionPoint* point);

    private:
        list<TMotionPoint*> openList;  /**< Open list that contains points we may want to check later.*/
        list<TMotionPoint*> closeList;  /**< Stores points already checked.*/
    };
}
#endif /** CPATHGENERATOR_H_ */