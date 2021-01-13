package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ultimategoal.drive.Drivetrain;
import org.wolfcorp.ultimategoal.vision.StartingPosition;
import org.wolfcorp.ultimategoal.vision.TFODZoneChooser;
import org.wolfcorp.ultimategoal.vision.Target;
import org.wolfcorp.ultimategoal.vision.ZoneChooser;

// TODO: add actual autonomous opmodes
@Autonomous(name="Vision Testing")
public class VisionTesting extends LinearOpMode {
    protected ZoneChooser chooser;

    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap);
        // To change the vision algorithm, change the constructor used
        chooser = new TFODZoneChooser();
        chooser.init(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("target", chooser.getTarget());
            telemetry.update();
        }
        chooser.stop();
    }
}
