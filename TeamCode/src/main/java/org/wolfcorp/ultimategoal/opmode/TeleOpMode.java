package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;

@TeleOp(name="Current TeleOp", group = "drive")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Drivetrain drive = new Drivetrain(hardwareMap);
        Scorer scorer = new Scorer(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message", "Hello Driver");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        scorer.openRelease();

        while (opModeIsActive()) {

            // Drivetrain
            drive.drive(-gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x * 0.8, 0.4, gamepad1.right_bumper);

            // Reversing scoring mechanisms
            scorer.reverse(gamepad1.b, 200);

            // Intake
            scorer.toggleIntake(gamepad1.a && !gamepad1.start, 1, 200);

            // Outtake
            scorer.toggleStopper(gamepad1.dpad_left, gamepad1.x, 200);
            scorer.toggleOuttake(gamepad1.y || gamepad2.y, gamepad1.left_stick_button || gamepad2.a, 200);

            // Wobble Goal
            scorer.wobbleGripper(gamepad1.left_bumper || gamepad2.left_bumper, 200);
            scorer.wobbleArm(gamepad1.dpad_down || gamepad2.dpad_down, gamepad1.dpad_up || gamepad2.dpad_up);

            // TODO: add driver assist (auto-shoot using odom)
            // TODO: add button for manually resetting encoders after banging against wall
            //  (odometry gets more and more inaccurate over time)

            //TelemetryPacket packet = new TelemetryPacket();
            //packet.put("vel", scorer.outtake.getVelocity());
            //packet.put("shoot", scorer.stopper.getPosition() < 0.34 ? 2200 : 1500);

            //dashboard.sendTelemetryPacket(packet);

            drive.update(); // odometry update
        }
    }
}
