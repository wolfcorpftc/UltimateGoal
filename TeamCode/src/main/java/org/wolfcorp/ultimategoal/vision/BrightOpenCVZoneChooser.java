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

public class BrightOpenCVZoneChooser extends OpenCvPipeline implements ZoneChooser {

    public static final double THRESH = 8.0;

    protected OpenCvCamera cam;
    protected Telemetry telemetry;
    protected Rect topROI, bottomROI, ringROI;
    protected Mat mat = new Mat();
    protected Mat top = new Mat();
    protected Mat bottom = new Mat();
    protected Target target = Target.UNSET;

    public BrightOpenCVZoneChooser() {
        topROI = new Rect(new Point(223, 120), new Point(287, 145));
        bottomROI = new Rect(new Point(223, 145), new Point(287, 170));
        ringROI = new Rect(new Point(213, 175), new Point(267, 215));
    }


    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        // Note that OpenCV HSV hue ranges from [0, 179], not [0, 359]
        // Hue: [0, 179]
        // Sat: [0, 255]
        // Val: [0, 255]
        // ring color range
        Scalar lowHSV = new Scalar(15 / 2.0, 100, 100);
        Scalar highHSV = new Scalar(45 / 2.0, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat); // filter color; result is black-and-white
        // extract regions where the rings are
        top = mat.submat(topROI);
        bottom = mat.submat(bottomROI);
        // calculate percentage of ring color on the image
        double topPercentage = Math.round(
                Core.sumElems(top).val[0] / topROI.area() / 255 * 100);
        double bottomPercentage = Math.round(
                Core.sumElems(bottom).val[0] / bottomROI.area() / 255 * 100);
        //mat.copyTo(input); // use grayscale for debugging
        mat.release(); // use color imaged for book

        Scalar resultColor = new Scalar(255, 0, 255);
        if (topPercentage > THRESH) {
            target = Target.C;
            Imgproc.rectangle(input, topROI, resultColor);
        }
        else if (bottomPercentage > THRESH) {
            target = Target.B;
            Imgproc.rectangle(input, bottomROI, resultColor);
        }
        else {
            target = Target.A;
            Imgproc.rectangle(input, ringROI, resultColor);
        }

        telemetry.addData("Top", topPercentage + "%");
        telemetry.addData("Bottom", bottomPercentage + "%");
        telemetry.addData("Threshold", THRESH + "%");
        telemetry.addData("Target Zone", target.name());
        telemetry.update();

        return input;
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
