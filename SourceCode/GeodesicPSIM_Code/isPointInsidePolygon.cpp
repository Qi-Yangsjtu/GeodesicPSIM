#include "headfile.h"




float calculateTriangleArea(Point a, Point b, Point c) {
    return abs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0);
}

bool isPointInsideTriangle(Point p, std::vector<Point> polygon) {
    Point a = polygon[0];
    Point b = polygon[1];
    Point c = polygon[2];

    float totalArea = calculateTriangleArea(a, b, c);
    float area1 = calculateTriangleArea(p, a, b);
    float area2 = calculateTriangleArea(p, b, c);
    float area3 = calculateTriangleArea(p, c, a);

    return (std::fabs(totalArea - (area1 + area2 + area3)) < 1e-1);
}
