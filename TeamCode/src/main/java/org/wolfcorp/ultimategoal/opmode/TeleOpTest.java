package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="Current TeleOp", group = "drive")
public class TeleOpTest extends OpMode {
    @Override
    public void runOpMode() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message", "Hello Driver");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Drivetrain
            if(gamepad1.y) {
                drive.setMotorPowers(0.15);
            } else if(gamepad1.a && !gamepad1.start){
                drive.setMotorPowers(-0.3);
            } else {
                drive.drive(-gamepad1.right_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_stick_x * 0.9);
            }

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

            // TODO: add intake / outtake
            // TODO: add driver assist (auto-shoot using odom)
            // TODO: add button for manually resetting encoders after banging against wall
            //  (odometry gets more and more inaccurate over time)

            drive.update(); // odometry update
        }
    }
}
