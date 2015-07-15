package imp.qrscanner;

import org.opencv.core.Point;


public class CrossingPoint {
    Boolean crosses;
    Point crossPoint;

    public Boolean getCrosses() {
        return crosses;
    }

    public Point getCrossPoint() {
        return crossPoint;
    }

    public void setCrosses(Boolean crosses) {
        this.crosses = crosses;
    }

    public void setCrossPoint(Point crossPoint) {
        this.crossPoint = crossPoint;
    }
}
