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

    public static final double ZONE_C_THRESH = 80.0;
    public static final double ZONE_B_THRESH = 15.0;

    protected OpenCvCamera cam;
    protected Telemetry telemetry;
    protected Rect ringROI;
    protected Mat mat = new Mat();
    protected Mat ring = new Mat();
    protected Target target = Target.UNSET;

    public BrightOpenCVZoneChooser() {
        ringROI = new Rect(new Point(275, 120), new Point(329, 160));
    }


    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        // Note that OpenCV HSV hue ranges from [0, 179], not [0, 359]
        // TODO: tune color
        Scalar lowHSV = new Scalar(30 / 2.0, 150, 153);
        Scalar highHSV = new Scalar(44 / 2.0, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        ring = mat.submat(ringROI);
        double percentage = Math.round(Core.sumElems(ring).val[0] / ringROI.area() / 255 * 100);
        telemetry.addData("Raw value", (int) Core.sumElems(ring).val[0]);
        telemetry.addData("Percentage", Math.round(percentage * 100) + "%");

        // TODO: tune percentage thresholds
        Scalar resultColor;
        if (percentage > ZONE_C_THRESH) {
            target = Target.C;
            resultColor = new Scalar(255, 0, 0);
        }
        else if (percentage > ZONE_B_THRESH) {
            target = Target.B;
            resultColor = new Scalar(0, 255, 0);
        }
        else {
            target = Target.A;
            resultColor = new Scalar(0, 0, 255);
        }

        Imgproc.rectangle(input, ringROI, resultColor);
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
