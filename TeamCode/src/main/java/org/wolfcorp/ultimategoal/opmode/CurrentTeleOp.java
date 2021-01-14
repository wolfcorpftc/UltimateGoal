package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;

@Config
@TeleOp(name="Current TeleOp", group = "drive")
public class CurrentTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap);
        Scorer scorer = new Scorer(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message", "Hello Driver");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Drivetrain
            drive.drive(-gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x * 0.9);

            // Drivetrain speeds
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                drive.speedMultiplier = 1;
            } else if (gamepad1.left_bumper) {
                drive.speedMultiplier = 0.4;
            } else if (gamepad1.right_bumper) {
                drive.speedMultiplier = 0.5;
            } else {
                drive.speedMultiplier = 0.85;
            }

            // Reversing scoring mechanisms
            scorer.reverse(gamepad1.right_bumper, 200);

            // Intake
            scorer.intakeToggle(gamepad1.a && !gamepad1.start, 1, 200);

            // Outtake
            scorer.outtakeToggle(gamepad1.y, 0.4, 200);

            // Wobble Goal
            scorer.wobbleGripper(gamepad1.dpad_down, 200);
            scorer.wobbleArm(gamepad1.left_bumper);

            // TODO: add driver assist (auto-shoot using odom)
            // TODO: add button for manually resetting encoders after banging against wall
            //  (odometry gets more and more inaccurate over time)

            drive.update(); // odometry update
        }
    }
}
