package org.wolfcorp.ultimategoal.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class DimOpenCVZoneChooser extends OpenCvPipeline implements ZoneChooser {

    public static final double ZONE_C_THRESH = 80.0;
    public static final double ZONE_B_THRESH = 15.0;

    protected OpenCvCamera cam;
    protected Telemetry telemetry;
    protected Rect ringROI;
    protected Rect fieldROI;
    protected Mat mat = new Mat();
    protected Mat ring = new Mat();
    protected Mat field = new Mat();
    protected Target target = Target.UNSET;

    public DimOpenCVZoneChooser() {
        // TODO: shrink the rectangle
        ringROI = new Rect(new Point(213, 170), new Point(267, 210));
        fieldROI = new Rect(new Point(80, 140), new Point(239, 220));
    }


    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        telemetry.addData("cols", mat.cols());
        telemetry.addData("rows", mat.rows());
        telemetry.update();
        Mat ring = mat.submat(ringROI);
        Mat field = mat.submat(fieldROI);
        double[] a = avgColor(ring);
        double[] b = avgColor(field);
        telemetry.addData("cols new", mat.cols());
        telemetry.addData("rows new", mat.rows());
        telemetry.update();

        double diff = 0;
        for (int i = 0; i < a.length; i++)
            diff += Math.abs(a[i] - b[i]);
        telemetry.addData("Difference", diff);

        // TODO: tune percentage thresholds
        Scalar resultColor;
        if (diff > ZONE_C_THRESH) {
            target = Target.C;
            resultColor = new Scalar(255, 0, 0);
        }
        else if (diff > ZONE_B_THRESH) {
            target = Target.B;
            resultColor = new Scalar(0, 255, 0);
        }
        else {
            target = Target.A;
            resultColor = new Scalar(0, 0, 255);
        }

        Imgproc.rectangle(input, ringROI, resultColor);
        Imgproc.rectangle(input, fieldROI, new Scalar(0, 0, 0));
        telemetry.addData("Target Zone", target.name());
        telemetry.update();

        return input;
    }

    double[] avgColor(Mat mat) {
        Scalar sum = Core.sumElems(mat);
        for (int i = 0; i < sum.val.length; i++) {
            sum.val[i] /= mat.rows() * mat.cols();
        }
        return sum.val.clone();
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cam.setPipeline(this);
        cam.openCameraDeviceAsync(
                () -> cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );
    }

    public Target getTarget() {
        return target;
    }

    public void stop() {
        cam.stopStreaming();
    }
}
