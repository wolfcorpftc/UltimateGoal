package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ultimategoal.robot.Scorer;

@Config
@TeleOp(name="Demo TeleOp", group = "drive")
public class DemoMode extends LinearOpMode {
    Scorer scorer;

    boolean ledOn;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        scorer = new Scorer(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message", "Hello Driver");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        scorer.openRelease();

        timer.reset();
        while (opModeIsActive()) {
            // Reversing scoring mechanisms
            scorer.reverse(gamepad1.b, 200);

            // Intake
            scorer.toggleIntake(gamepad1.a && !gamepad1.start, 1, 200);
            scorer.toggleIntakeRelease(gamepad1.dpad_right || gamepad2.right_bumper);

            // Outtake
            scorer.toggleStopper(gamepad1.dpad_left, gamepad1.x, 200);
            scorer.toggleOuttake(gamepad1.y || gamepad2.y, gamepad2.a && !gamepad2.start, 200);

            // Wobble Goal
            scorer.wobbleGripper(gamepad1.left_bumper || gamepad2.left_bumper, 200);
            scorer.wobbleArm(gamepad1.dpad_down || gamepad2.dpad_down, gamepad1.dpad_up || gamepad2.dpad_up);
            scorer.moveFlipper(gamepad2.b && !gamepad2.start, gamepad2.x);

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                scorer.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
            else if (gamepad1.left_stick_button) {
                scorer.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
            }
            else if (gamepad1.right_stick_button) {
                scorer.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
            }
        }
    }
}
