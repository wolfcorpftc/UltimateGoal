/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.wolfcorp.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Lesson Program", group="Linear Opmode")
@Disabled
public class LessonProgram extends LinearOpMode {

    DcMotor left;
    DcMotor right;
    DcMotor leftF;
    DcMotor rightF;


    @Override
    public void runOpMode() {

        left = hardwareMap.dcMotor.get("LB");
        right = hardwareMap.dcMotor.get("RB");
        leftF = hardwareMap.dcMotor.get("LF");
        rightF = hardwareMap.dcMotor.get("RF");

        left.setDirection(DcMotor.Direction.REVERSE);
        leftF.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
/*            // forward
            left.setPower(0.5);
            right.setPower(0.5);
            leftF.setPower(0.5);
            rightF.setPower(0.5);
            sleep(1500);

            left.setPower(0);
            right.setPower(0);
            leftF.setPower(0);
            rightF.setPower(0);
            sleep(5000);

            // backward
            left.setPower(-0.5);
            right.setPower(-0.5);
            leftF.setPower(-0.5);
            rightF.setPower(-0.5);
            sleep(1500);

            left.setPower(0);
            right.setPower(0);
            leftF.setPower(0);
            rightF.setPower(0);
            sleep(5000);

            // turn left
            left.setPower(-0.5);
            right.setPower(0.5);
            leftF.setPower(-0.5);
            rightF.setPower(0.5);
            sleep(1700);

            left.setPower(0);
            right.setPower(0);
            leftF.setPower(0);
            rightF.setPower(0);
            sleep(5000);

            // turn right
            left.setPower(0.5);
            right.setPower(-0.5);
            leftF.setPower(0.5);
            rightF.setPower(-0.5);
            sleep(1700);

            left.setPower(0);
            right.setPower(0);
            leftF.setPower(0);
            rightF.setPower(0);
            sleep(5000);

            // slow turn right
            left.setPower(0.5);
            right.setPower(0.2);
            leftF.setPower(0.5);
            rightF.setPower(0.2);
            sleep(2500);

            left.setPower(0);
            right.setPower(0);
            leftF.setPower(0);
            rightF.setPower(0);
            sleep(5000);

            // slow turn left
            left.setPower(-0.5);
            right.setPower(-0.2);
            leftF.setPower(-0.5);
            rightF.setPower(-0.2);
            sleep(1700);

            left.setPower(0);
            right.setPower(0);
            leftF.setPower(0);
            rightF.setPower(0);
            sleep(5000);*/

            // square
            for (int i = 0; i < 4; i++) {
                left.setPower(0.5);
                right.setPower(0.5);
                leftF.setPower(0.5);
                rightF.setPower(0.5);
                sleep(1750);

                right.setPower(-0.5);
                rightF.setPower(-0.5);
                sleep(930);
            }

            left.setPower(0);
            right.setPower(0);
            leftF.setPower(0);
            rightF.setPower(0);
            sleep(100000);
        }
    }
}
