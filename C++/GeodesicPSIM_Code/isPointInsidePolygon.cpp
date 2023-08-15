#include "headfile.h"


double getSlope(Point p1, Point p2) {
    return (p2.y - p1.y) / (p2.x - p1.x);
}

bool isPointInsidePolygon(Point point, std::vector<Point> polygon) {
    int intersections = 0;
    int numPoints = polygon.size();

    for (int i = 0; i < numPoints; i++) {
        Point currentPoint = polygon[i];
        Point nextPoint = polygon[(i + 1) % numPoints];

        if ((point.x == currentPoint.x && point.y == currentPoint.y) ||
            (point.x == nextPoint.x && point.y == nextPoint.y)) {
            return true;
        }

        if ((point.y > currentPoint.y && point.y > nextPoint.y) ||
            (point.y < currentPoint.y && point.y < nextPoint.y)) {
            continue;
        }

        double slope = getSlope(currentPoint, nextPoint);

        if (point.y == currentPoint.y && point.y == nextPoint.y &&
            ((point.x > currentPoint.x && point.x < nextPoint.x) ||
                (point.x < currentPoint.x && point.x > nextPoint.x))) {
            return true;
        }

        if ((point.y - currentPoint.y) / slope + currentPoint.x >= point.x) {
            intersections++;
        }
    }

    return intersections % 2 == 1;
}