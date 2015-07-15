package imp.qrscanner;


import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.*;

import java.util.ArrayList;
import java.util.List;




public class ImageProcessor {

    Mat originalMat;
    Mat debugMat;
    int northOrientation = 0;
    int eastOrientation = 1;
    int southOrientation = 2;
    int westOrientation = 3;

    // convert preview data to CV Mat and start processing
    public void setOriginalMat(byte[] data, int width, int height){
        Mat yuvMat = new Mat(height+height/2, width, CvType.CV_8UC1);
        Mat rgba = new Mat(height, width, CvType.CV_8UC4);
        if(data.length>0){
            yuvMat.put(0, 0, data);
        }
        Imgproc.cvtColor(yuvMat, rgba, Imgproc.COLOR_YUV420sp2RGB);
        originalMat = rgba;
        debugMat = originalMat.clone();



        find(originalMat);
    }

    // getdebugMat is used by MainActivity for displaying purpose
    public Mat getdebugMat()
    {
        return debugMat;
    }

    // calculate distance between two points
    private double distance(Point p1, Point p2){
        double d = Math.sqrt(Math.pow((p1.x-p2.x), 2)+Math.pow((p1.y-p2.y), 2));
        return d;
    }

    // calculate distance between a point and a line
    private double distancePointToLine(Point linePoint1, Point linePoint2, Point inputPoint){
        double p1,p2,p3,distance;

        p1 = -((linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x));
        p2 = 1.0;
        p3 = (((linePoint2.y - linePoint1.y) /(linePoint2.x - linePoint1.x)) * linePoint1.x) - linePoint1.y;


        distance = (p1 * inputPoint.x + (p2 * inputPoint.y) + p3) / Math.sqrt((p1 * p1) + (p2 * p2));
        return distance;
    }

    //calculate slope of line that goes through p1 and p2. Alignement=0 flag for special case.
    //return SlopeObject
    SlopeObject createSlopeObject(Point p1, Point p2)
    {
        double distanceX,distanceY,slope;
        int alignement;
        distanceX = p2.x - p1.x;
        distanceY = p2.y - p1.y;

        if ( distanceY != 0)
        {
            alignement = 1;
            slope = distanceY / distanceX;
        }
        else
        {
            alignement = 0;
            slope = 0.0;
        }
        SlopeObject slopeObject = new SlopeObject();
        slopeObject.setAlignement(alignement);
        slopeObject.setSlope(slope);
        return slopeObject;
    }

    // checks distance between testPoint and referencePoint.
    // If distance is farther than MaxDistance return updated cornerPoint. Else return untouched cornerPoint
    FarthestPoint checkFarthestPoint(Point testPoint, Point referencePoint, double maxDistance, Point cornerPoint)
    {
        double tmpDistance;
        tmpDistance = distance(testPoint, referencePoint);

        if(tmpDistance > maxDistance)
        {
            maxDistance = tmpDistance;
            cornerPoint = testPoint;

        }
        FarthestPoint farthestPoint = new FarthestPoint();
        farthestPoint.setDistance(maxDistance);
        farthestPoint.setP(cornerPoint);
        return farthestPoint;
    }

    //rearrange the four Marker Corners to match the given qr-code orientation
    MatOfPoint2f rearrangeMarkerCorners(int orientation, MatOfPoint2f inputCornerPoints)
    {
        MatOfPoint2f outputCornerPoints = new MatOfPoint2f();
        Point[] tmpInputCornerPoints = inputCornerPoints.toArray();
        Point[] tmpOutputCornerPoints = inputCornerPoints.toArray();
        Point p1 = new Point(0,0);
        Point p2 = new Point(0,0);
        Point p3 = new Point(0,0);
        Point p4 = new Point(0,0);
        if(orientation == northOrientation)
        {
            p1 = tmpInputCornerPoints[0];
            p2 = tmpInputCornerPoints[1];
            p3 = tmpInputCornerPoints[2];
            p4 = tmpInputCornerPoints[3];
        }
        else if (orientation == eastOrientation)
        {
            p1 = tmpInputCornerPoints[1];
            p2 = tmpInputCornerPoints[2];
            p3 = tmpInputCornerPoints[3];
            p4 = tmpInputCornerPoints[0];
        }
        else if (orientation == southOrientation)
        {
            p1 = tmpInputCornerPoints[2];
            p2 = tmpInputCornerPoints[3];
            p3 = tmpInputCornerPoints[0];
            p4 = tmpInputCornerPoints[1];
        }
        else if (orientation == westOrientation)
        {
            p1 = tmpInputCornerPoints[3];
            p2 = tmpInputCornerPoints[0];
            p3 = tmpInputCornerPoints[1];
            p4 = tmpInputCornerPoints[2];
        }

        tmpOutputCornerPoints[0] = p1;
        tmpOutputCornerPoints[1] = p2;
        tmpOutputCornerPoints[2] = p3;
        tmpOutputCornerPoints[3] = p4;
        outputCornerPoints.fromArray(tmpOutputCornerPoints);
        return outputCornerPoints;
    }

    //get the crossingPoint of two lines
    CrossingPoint getCrossingPoint(Point inputPointA1, Point inputPointA2, Point inputPointB1, Point inputPointB2)
    {
        CrossingPoint crossingPoint = new CrossingPoint();
        Point crossPoint = new Point();
        Point p1 = inputPointA1;
        Point p2 = inputPointB1;
        Point p3 = new Point(inputPointA2.x-inputPointA1.x,inputPointA2.y-inputPointA1.y);
        Point p4 = new Point(inputPointB2.x-inputPointB1.x,inputPointB2.y-inputPointB1.y);

        if(cross(p3,p4) == 0) {
            crossingPoint.setCrossPoint(crossPoint);
            crossingPoint.setCrosses(false);
            return crossingPoint;
        }
        else {

            Point z = new Point(p2.x - p1.x, p2.y - p1.y);
            double t = cross(z, p4) / cross(p3, p4);

            crossPoint.x= p1.x + (t*p3.x);
            crossPoint.y= p1.y + (t*p3.y);

            crossingPoint.setCrossPoint(crossPoint);
            crossingPoint.setCrosses(true);
            return crossingPoint;
        }
    }

    double cross(Point p1,Point p2)
    {
        return p1.x*p2.y - p1.y*p2.x;
    }

    // create the four marker corners for a single Marker contour object
    MatOfPoint2f createMarkerCorners(List<MatOfPoint> contours, int c_id, double slope)
    {
        Rect box;
        box = Imgproc.boundingRect( contours.get(c_id));

        Point marker1 = new Point();
        Point marker2 = new Point();
        Point marker3 = new Point();
        Point marker4 = new Point();
        Point p1;
        Point p2 = new Point();
        Point p3;
        Point p4 = new Point();
        Point p12 = new Point();
        Point p23 = new Point();
        Point p34 = new Point();
        Point p41 = new Point();

        p1 =  box.tl();
        p2.x = box.br().x;
        p2.y = box.tl().y;
        p3 = box.br();
        p4.x = box.tl().x;
        p4.y = box.br().y;

        p12.x = (p1.x + p2.x) / 2;
        p12.y = p1.y;

        p23.x = p2.x;
        p23.y = (p2.y + p3.y) / 2;

        p34.x = (p3.x + p4.x) / 2;
        p34.y = p3.y;

        p41.x = p4.x;
        p41.y = (p4.y + p1.y) / 2;

        double[] maxDistances = new double[4];
        maxDistances[0]=0.0;
        maxDistances[1]=0.0;
        maxDistances[2]=0.0;
        maxDistances[3]=0.0;

        double caPointToLineDistance = 0.0;
        double bdPointToLineDistance = 0.0;

        MatOfPoint contour = contours.get(c_id);
        Point[] contourTmp = contour.toArray();

        if (slope > 5 || slope < -5 )
        {
            for( int i = 0; i < contourTmp.length; i++ )
            {
                caPointToLineDistance = distancePointToLine(p3, p1, contourTmp[i]);
                bdPointToLineDistance = distancePointToLine(p2, p4, contourTmp[i]);
                FarthestPoint farthestPoint;

                if((caPointToLineDistance >= 0.0) && (bdPointToLineDistance > 0.0))
                {
                    farthestPoint = checkFarthestPoint(contourTmp[i], p12, maxDistances[1], marker2);
                    maxDistances[1] = farthestPoint.getDistance();
                    marker2 = farthestPoint.getP();
                }
                else if((caPointToLineDistance > 0.0) && (bdPointToLineDistance <= 0.0))
                {
                    farthestPoint = checkFarthestPoint(contourTmp[i], p23, maxDistances[2], marker3);
                    maxDistances[2] = farthestPoint.getDistance();
                    marker3 = farthestPoint.getP();
                }
                else if((caPointToLineDistance <= 0.0) && (bdPointToLineDistance < 0.0))
                {
                    farthestPoint = checkFarthestPoint(contourTmp[i], p34, maxDistances[3], marker4);
                    maxDistances[3] = farthestPoint.getDistance();
                    marker4 = farthestPoint.getP();
                }
                else if((caPointToLineDistance < 0.0) && (bdPointToLineDistance >= 0.0))
                {
                    farthestPoint = checkFarthestPoint(contourTmp[i], p41, maxDistances[0], marker1);
                    maxDistances[0] = farthestPoint.getDistance();
                    marker1 = farthestPoint.getP();
                }
                else
                    continue;
            }
        }
        else
        {
            double halfx = (p1.x + p2.x) / 2;
            double halfy = (p1.y + p4.y) / 2;
            FarthestPoint farthestPoint;

            for( int i = 0; i < contourTmp.length; i++ )
            {
                if((contourTmp[i].x < halfx) && (contourTmp[i].y <= halfy))
                {
                    farthestPoint = checkFarthestPoint(contourTmp[i], p3, maxDistances[2], marker1);
                    maxDistances[2] = farthestPoint.getDistance();
                    marker1 = farthestPoint.getP();
                }
                else if((contourTmp[i].x >= halfx) && (contourTmp[i].y < halfy))
                {
                    farthestPoint = checkFarthestPoint(contourTmp[i], p4, maxDistances[3], marker2);
                    maxDistances[3] = farthestPoint.getDistance();
                    marker2 = farthestPoint.getP();
                }
                else if((contourTmp[i].x > halfx) && (contourTmp[i].y >= halfy))
                {
                    farthestPoint = checkFarthestPoint(contourTmp[i], p1, maxDistances[0], marker3);
                    maxDistances[0] = farthestPoint.getDistance();
                    marker3 = farthestPoint.getP();
                }
                else if((contourTmp[i].x <= halfx) && (contourTmp[i].y > halfy))
                {
                    farthestPoint = checkFarthestPoint(contourTmp[i], p2, maxDistances[1], marker4);
                    maxDistances[1] = farthestPoint.getDistance();
                    marker4 = farthestPoint.getP();
                }
            }
        }

        Point[] transition = new Point[4];
        transition[0] = marker1;
        transition[1] = marker2;
        transition[2] = marker3;
        transition[3] = marker4;
        MatOfPoint2f markerCornerMat = new MatOfPoint2f();
        markerCornerMat.fromArray(transition);
        return markerCornerMat;
    }


    //process Image
    private void find(Mat src)
    {
        Mat srcMatCopy = src.clone();
        Mat srcMatGray = new Mat(src.size(), CvType.makeType(src.depth(), 1));
        Mat srcMatEdges = new Mat(src.size(), CvType.makeType(src.depth(), 1));

        Mat qrMat = Mat.zeros(100,100,CvType.CV_8UC3);
        Mat qrMatUnprocessed = Mat.zeros(100,100,CvType.CV_8UC3);
        Mat qrMatGray = Mat.zeros(100, 100, CvType.CV_8UC1);
        Mat qrMatThresholded = Mat.zeros(100, 100, CvType.CV_8UC1);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        int markerCount;
        int A = -1;
        int B = -1;
        int C = -1;
        int topMarkerContourId = -1;
        int rightMarkerContourId = -1;
        int bottomMarkerContourId = -1;
        int marker1 = -1;
        int marker2 = -1;
        int topMarker = -1;
        double AB;
        double BC;
        double CA;
        double dist,slope;

        int align = -1;
        int orientation = -1;
        markerCount = 0;

        //Grayscale conversion + Canny edge detection + Contour detection with hierarchy tree retrieval
        Imgproc.cvtColor(src, srcMatGray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.Canny(srcMatGray, srcMatEdges, 100, 200, 3, true);
        Imgproc.findContours(srcMatEdges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //calculate contour mass centers. Not very elegant approach since cv::Moments are not implemented in OpenCv4Android 3.0.0
        List<Point> massCenter= new ArrayList<>();
        for(MatOfPoint c : contours){
            Rect box;
            box = Imgproc.boundingRect(c);
            Point p = new Point(box.x + box.width/2,box.y + box.height/2);
            massCenter.add(p);
        }

        //transform hierarchy mat in java friendly int[][]
        int[][] hierInt = new int[contours.size()][4];
        for(int x=0; x<hierarchy.cols();x++){
            double[] vec = hierarchy.get(0,x);
            hierInt[x][0]=(int)vec[0];
            hierInt[x][1]=(int)vec[1];
            hierInt[x][2]=(int)vec[2];
            hierInt[x][3]=(int)vec[3];
        }

        //search for potential marker candidates
        for( int i = 0; i < contours.size(); i++ )
        {
            int hierarchyIndex=i;
            int childCounter=0;


            while(hierInt[hierarchyIndex][2] != -1)
            {
                hierarchyIndex = hierInt[hierarchyIndex][2] ;
                childCounter = childCounter+1;
            }
            if(hierInt[hierarchyIndex][2] != -1)
                childCounter = childCounter+1;

            if (childCounter >= 5)
            {
                if (markerCount == 0)		A = i;
                else if  (markerCount == 1)	B = i;
                else if  (markerCount == 2)	C = i;
                markerCount = markerCount + 1 ;
            }
        }

        //found the three necessary markers
        if (markerCount >= 3)
        {

            //longest distance can be found between right and bottom marker. So top marker is not on the longest line
            AB = distance(massCenter.get(A),massCenter.get(B));
            BC = distance(massCenter.get(B),massCenter.get(C));
            CA = distance(massCenter.get(C),massCenter.get(A));

            if ( AB > BC && AB > CA )
            {
                topMarker = C; marker1=A; marker2=B;
            }
            else if ( CA > AB && CA > BC )
            {
                topMarker = B; marker1=A; marker2=C;
            }
            else if ( BC > AB && BC > CA )
            {
                topMarker = A;  marker1=B; marker2=C;
            }

            topMarkerContourId = topMarker;

            dist = distancePointToLine(massCenter.get(marker1), massCenter.get(marker2), massCenter.get(topMarker));
            SlopeObject slopeObject = createSlopeObject(massCenter.get(marker1), massCenter.get(marker2));
            slope = slopeObject.getSlope();
            align = slopeObject.getAlignement();

            //slope of line marker1 to marker2  and  traveldistance of topMarker to said line, indicates qrCode-orientation
            if (align == 0)
            {
                bottomMarkerContourId = marker1;
                rightMarkerContourId = marker2;
                orientation = northOrientation;
            }
            else if (slope < 0 && dist < 0 )
            {
                bottomMarkerContourId = marker1;
                rightMarkerContourId = marker2;
                orientation = northOrientation;
            }
            else if (slope > 0 && dist < 0 )
            {
                rightMarkerContourId = marker1;
                bottomMarkerContourId = marker2;
                orientation = eastOrientation;
            }
            else if (slope < 0 && dist > 0 )
            {
                rightMarkerContourId = marker1;
                bottomMarkerContourId = marker2;
                orientation = southOrientation;
            }

            else if (slope > 0 && dist > 0 )
            {
                bottomMarkerContourId = marker1;
                rightMarkerContourId = marker2;
                orientation = westOrientation;
            }



            //make sure found Markers are valid and big enough
            if( topMarkerContourId < contours.size() && rightMarkerContourId < contours.size() && bottomMarkerContourId < contours.size() && Imgproc.contourArea(contours.get(topMarkerContourId)) > 10 && Imgproc.contourArea(contours.get(rightMarkerContourId)) > 10 && Imgproc.contourArea(contours.get(bottomMarkerContourId)) > 10 )
            {

                MatOfPoint2f topMarkerMat = new MatOfPoint2f();
                MatOfPoint2f rightMarkerMat = new MatOfPoint2f();
                MatOfPoint2f bottomMarkerMat = new MatOfPoint2f();
                MatOfPoint2f tmpTopMarkerMat = new MatOfPoint2f();
                MatOfPoint2f tmpRightMarkerMat = new MatOfPoint2f();
                MatOfPoint2f tmpBottomMarkerMat = new MatOfPoint2f();
                Point crossPoint;

                Point[] src2 = new Point[4];
                Point[] dst = new Point[4];


                Mat warpPerspectiveMatrix;

                tmpTopMarkerMat = createMarkerCorners(contours, topMarkerContourId, slope);
                tmpRightMarkerMat = createMarkerCorners(contours, rightMarkerContourId, slope);
                tmpBottomMarkerMat = createMarkerCorners(contours, bottomMarkerContourId, slope);

                topMarkerMat = rearrangeMarkerCorners(orientation, tmpTopMarkerMat);
                rightMarkerMat = rearrangeMarkerCorners(orientation, tmpRightMarkerMat);
                bottomMarkerMat = rearrangeMarkerCorners(orientation, tmpBottomMarkerMat);

                Point[]topMarkerPointArray = topMarkerMat.toArray();
                Point[]rightMarkerPointArray = rightMarkerMat.toArray();
                Point[]bottomMarkerPointArray = bottomMarkerMat.toArray();

                //get missing fourth point for getPerspectiveTransform()
                CrossingPoint crossingPoint = getCrossingPoint(rightMarkerPointArray[1], rightMarkerPointArray[2], bottomMarkerPointArray[3], bottomMarkerPointArray[2]);
                crossPoint = crossingPoint.getCrossPoint();

                //setup source and destination for getPerspectiveTransform()
                src2[0] = topMarkerPointArray[0];
                src2[1] = rightMarkerPointArray[1];
                src2[2] = crossPoint;
                src2[3] = bottomMarkerPointArray[3];

                dst[0] = new Point(0,0);
                dst[1] = new Point(qrMat.cols(),0);
                dst[2] = new Point(qrMat.cols(), qrMat.rows());
                dst[3] = new Point(0, qrMat.rows());

                if (src2.length == 4 && dst.length == 4 )
                {
                    MatOfPoint2f src3 = new MatOfPoint2f();
                    src3.fromArray(src2);
                    MatOfPoint2f dst2 = new MatOfPoint2f();
                    dst2.fromArray(dst);

                    warpPerspectiveMatrix = Imgproc.getPerspectiveTransform(src3, dst2);
                    Imgproc.warpPerspective(srcMatCopy, qrMatUnprocessed, warpPerspectiveMatrix, new Size(qrMat.cols(), qrMat.rows()));
                    Core.copyMakeBorder(qrMatUnprocessed, qrMat, 10, 10, 10, 10, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));

                    Imgproc.cvtColor(qrMat, qrMatGray, Imgproc.COLOR_RGB2GRAY);
                    Imgproc.threshold(qrMatGray, qrMatThresholded, 127, 255, Imgproc.THRESH_BINARY);
                }


                /*
                // Debug Options
                Imgproc.drawContours(copy, contours, topMarkerContourId, new Scalar(255, 200, 0), 2);
                Imgproc.drawContours(copy, contours, rightMarkerContourId, new Scalar(0, 0, 255), 2);
                Imgproc.drawContours( copy, contours, bottomMarkerContourId , new Scalar(255,0,100), 2);
                */

                //debugMat is the displayed mat
                debugMat = qrMatThresholded.clone();


            }
        }



    }
}
