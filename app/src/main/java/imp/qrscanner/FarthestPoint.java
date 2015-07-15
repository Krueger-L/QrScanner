package imp.qrscanner;

import org.opencv.core.Point;


public class FarthestPoint {
    double distance;
    Point p;

    public double getDistance() {
        return distance;
    }

    public Point getP() {
        return p;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public void setP(Point p) {
        this.p = p;
    }
}
