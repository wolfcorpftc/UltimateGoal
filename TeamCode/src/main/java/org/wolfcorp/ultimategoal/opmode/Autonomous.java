package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.wolfcorp.ultimategoal.vision.StartingPosition;
import org.wolfcorp.ultimategoal.vision.TargetZoneChooser;


public abstract class Autonomous extends OpMode {
    protected Pose2d initialPose = new Pose2d();
    protected StartingPosition sp = StartingPosition.UNSET;
    protected boolean doPushGoal = true;
    protected OpenCvCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {
        if (doPushGoal) {
            int cameraMonitorViewId = hardwareMap.appContext
                    .getResources().getIdentifier("cameraMonitorViewId",
                            "id", hardwareMap.appContext.getPackageName());
            cam = OpenCvCameraFactory.getInstance()
                    .createInternalCamera(
                            OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            TargetZoneChooser chooser = new TargetZoneChooser(sp, telemetry);
            cam.setPipeline(chooser);
            cam.openCameraDeviceAsync(
                    () -> cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
            );
        }
        waitForStart();
        drive.follow(drive.from(initialPose).build());
    }
}
