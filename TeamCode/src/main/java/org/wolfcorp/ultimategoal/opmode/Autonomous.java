package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.wolfcorp.ultimategoal.vision.StartingPosition;
import org.wolfcorp.ultimategoal.vision.Target;
import org.wolfcorp.ultimategoal.vision.TargetZoneChooser;


// TODO: add actual autonomous opmodes
public abstract class Autonomous extends OpMode {
    protected OpenCvCamera cam;
    protected TargetZoneChooser chooser;

    // configration for specific starting positions / plans
    // TODO: figure out the four initialPoses
    protected Pose2d initialPose = new Pose2d();
    protected StartingPosition sp = StartingPosition.UNSET;

    protected void initVision() {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(
                        OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        chooser = new TargetZoneChooser(sp, telemetry);
        cam.setPipeline(chooser);
        cam.openCameraDeviceAsync(
                () -> cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );
    }

    public abstract Vector2d getTargetPos(Target t);

    @Override
    public void runOpMode() throws InterruptedException {
        initVision();
        waitForStart();
        Vector2d zonePos = getTargetPos(chooser.getTarget());
        drive.follow(drive
                .from(initialPose)
                .splineTo(zonePos, initialPose.getHeading())
                .build());
        // TODO: drop goal
        // TODO: maybe reset by banging against wall
        // TODO: power shot one-by-one
        // TODO: depending on vision result, pick up starter stack
        //  and aim for the high goal
        // TODO: park
    }
}
