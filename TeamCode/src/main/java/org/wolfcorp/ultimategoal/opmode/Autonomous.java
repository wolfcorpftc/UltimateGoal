package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.wolfcorp.ultimategoal.vision.StartingPosition;
import org.wolfcorp.ultimategoal.vision.TargetZoneChooser;


// TODO: add actual autonomous opmodes
public abstract class Autonomous extends OpMode {
    protected OpenCvCamera cam;
    protected TargetZoneChooser chooser;

    // configration for specific starting positions / plans
    // TODO: figure out the four initialPoses
    protected Pose2d initialPose = new Pose2d();
    protected StartingPosition sp = StartingPosition.UNSET;
    protected boolean doPushGoal = true;

    @Override
    public void runOpMode() throws InterruptedException {
        if (doPushGoal) {
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
        waitForStart();
        if (doPushGoal) {
            Vector2d zonePos = new Vector2d();
            // TODO: initialize zonePos to actual wobble goal coords; check sp
            switch (chooser.getTarget()) {
                default: telemetry.addLine("Target not set, defaulting to A");
                case A: break;
                case B: break;
                case C: break;
            }
            // TODO: improve path planning
            drive.follow(drive
                    .from(initialPose)
                    .splineTo(zonePos, initialPose.getHeading())
                    .build());
        }
    }
}
