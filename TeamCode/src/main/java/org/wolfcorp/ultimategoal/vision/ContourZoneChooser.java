package org.wolfcorp.ultimategoal.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourZoneChooser extends OpenCvPipeline implements ZoneChooser {
    long[] stats = new long[3];

    public boolean ran = false;
    public boolean empty = false;

    protected Target target = Target.UNSET;

    @Override
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        // TODO: implement
    }

    @Override
    public void stop() {
        // TODO: implement
    }

    @Override
    public Mat processFrame(Mat input) {
        // TODO: Adjust code to new season
        ran = true;
        empty = false;

        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        // TODO: implement
        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
            target = Target.A;
            empty = true;
            return input;
        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(20, 100, 25); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 170, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // find contours the create bounding rectangles
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image.
        // The boxes are drawn into the result matrix
        Mat result = input.clone();
        int width = input.width();
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side
        for (Rect rect : boundRect) {
            // Apply an area threshold first
            if (rect.width * rect.height >= 2000) {
                if (rect.x < left_x)
                    left = true;
                if (rect.x + rect.width > right_x)
                    right = true;

                // draw red bounding rectangles
                Imgproc.rectangle(result, rect, new Scalar(211, 47, 47));
            }
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone
//        if (!left) location = Location.LEFT;
//        else if (!right) location = Location.RIGHT;
//        else location = Location.NOT_FOUND;

        // record statistics as the algorithm can become unstable
        updateStatistics();
        return result; // return the mat with rectangles drawn
    }

    // get location based on statistics
    @Override
    public Target getTarget() {
        empty = false;
        int maxIdx = 0;
        for (int i = 1; i != stats.length; i++) {
            if (stats[maxIdx] > stats[i])
                maxIdx = i;
        }
//        switch (maxIdx) {
//            case 0: return Location.LEFT;
//            case 1: return Location.RIGHT;
//            default: return Location.NOT_FOUND;
//        }
        return Target.UNSET;
    }

    // set all stats elements to 0
    public void resetStatistics() {
        for (double i : stats) {
            i = 0;
        }
    }

    // increase stats count based on current location
    private void updateStatistics() {
//        switch (location) {
//            case LEFT:
//                stats[0]++;
//                break;
//            case RIGHT:
//                stats[1]++;
//                break;
//            case NOT_FOUND:
//                stats[2]++;
//        }
    }
}

