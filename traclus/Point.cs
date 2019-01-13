using System;

namespace Traclus
{

    public struct Point2D
    {
        public double x;      // the coordinate of a point
        public double y;

        public Point2D(double xx, double yy) {
            x = xx;
            y = yy;
        }

        public static Point2D operator+(Point2D a, Point2D b) {
            return new Point2D(a.x + b.x, a.y + b.y);
        }

        public static Point2D operator-(Point2D a, Point2D b) {
            return new Point2D(a.x - b.x, a.y - b.y);
        }

        public static Point2D operator*(double a, Point2D b) {
            return new Point2D(a * b.x, a * b.y);
        }

        public static Point2D operator/(Point2D a, double b) {
            return new Point2D(a.x / b, a.y / b);
        }

        public double Dot(Point2D b) {
            return x * b.x + y * b.y;
        }

        public double Length() {
            return Math.Sqrt(x * x + y * y);
        }
    }

    public struct Segment {
        public Point2D start;
        public Point2D end;

        public Segment(Point2D s, Point2D e) {
            start = s;
            end = e;
        }
    }
}
