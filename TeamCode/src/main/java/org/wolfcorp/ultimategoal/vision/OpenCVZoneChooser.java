package org.wolfcorp.ultimategoal.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
@Disabled
public class OpenCVZoneChooser extends OpenCvPipeline implements ZoneChooser {

    protected OpenCvCamera cam;
    protected Telemetry telemetry;
    protected Rect ROI;
    protected Mat mat = new Mat();
    protected Target target = Target.UNSET;

    public OpenCVZoneChooser(StartingPosition sp) {
        // TODO: tune the rectangles
        switch (sp) {
            case BL: ROI = new Rect(new Point(60, 35), new Point(120, 75)); break;
            case BR: ROI = new Rect(new Point(60, 35), new Point(120, 75)); break;
            case RL: ROI = new Rect(new Point(60, 35), new Point(120, 75)); break;
            case RR: ROI = new Rect(new Point(60, 35), new Point(120, 75)); break;
        }
    }

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        // TODO: tune color
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat ringArea = mat.submat(ROI);
        double percentage = Math.round(Core.sumElems(ringArea).val[0] / ROI.area() / 255 * 100);
        ringArea.release();

        telemetry.addData("Raw value", (int) Core.sumElems(ringArea).val[0]);
        telemetry.addData("Percentage", Math.round(percentage * 100) + "%");

        // TODO: tune percentage thresholds
        Scalar resultColor;
        if (percentage > 80) {
            target = Target.C;
            resultColor = new Scalar(255, 0, 0);
        }
        else if (percentage > 15) {
            target = Target.B;
            resultColor = new Scalar(0, 255, 0);
        }
        else {
            target = Target.A;
            resultColor = new Scalar(0, 0, 255);
        }
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, ROI, resultColor);
        telemetry.addData("Target Zone", target.name());
        telemetry.update();

        return mat;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(
                        OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
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
