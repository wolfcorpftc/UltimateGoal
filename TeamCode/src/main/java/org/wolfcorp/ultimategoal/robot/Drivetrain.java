package org.wolfcorp.ultimategoal.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.wolfcorp.ultimategoal.util.AxesSigns;
import org.wolfcorp.ultimategoal.util.BNO055IMUUtil;
import org.wolfcorp.ultimategoal.util.DashboardUtil;
import org.wolfcorp.ultimategoal.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.wolfcorp.ultimategoal.robot.DriveConstants.BASE_CONSTRAINTS;
import static org.wolfcorp.ultimategoal.robot.DriveConstants.MOTOR_VELO_PID;
import static org.wolfcorp.ultimategoal.robot.DriveConstants.RUN_USING_ENCODER;
import static org.wolfcorp.ultimategoal.robot.DriveConstants.TRACK_WIDTH;
import static org.wolfcorp.ultimategoal.robot.DriveConstants.encoderTicksToInches;
import static org.wolfcorp.ultimategoal.robot.DriveConstants.kA;
import static org.wolfcorp.ultimategoal.robot.DriveConstants.kStatic;
import static org.wolfcorp.ultimategoal.robot.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Drivetrain extends MecanumDrive {

    public static double TKP = 20;
    public static double TKI = 0;
    public static double TKD = 1;

    public static double HKP = 5;
    public static double HKI = 0;
    public static double HKD = 0;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(TKP, TKI, TKD);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(HKP, HKI, HKD);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;
    public static boolean DRAW_PATH_HISTORY = true;

    public double speedMultiplier = 1;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Drivetrain.Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;
    private ArrayList<Path> pathHistory;

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private List<DcMotorEx> motors;
    public static BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    public Drivetrain(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public Drivetrain(HardwareMap hardwareMap, boolean resetIMU) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Drivetrain.Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();
        pathHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        if (imu == null || resetIMU || 1==1) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set up odometry
        //setLocalizer(new ThreeWheelTrackingLocalizer(hardwareMap));
    }

    public TrajectoryBuilder from(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, true, constraints);
    }

    public TrajectoryBuilder from(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder from(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public TrajectoryBuilder from(Pose2d startPose, double maxVel, double maxAccel) {
        DriveConstraints tempConstraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        tempConstraints.maxVel = maxVel;
        tempConstraints.maxAccel = maxAccel;
        return new TrajectoryBuilder(startPose, tempConstraints);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        turnStart = clock.seconds();
        mode = Drivetrain.Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Drivetrain.Mode.FOLLOW_TRAJECTORY;
        if (DRAW_PATH_HISTORY)
            pathHistory.add(trajectory.getPath());
    }

    public void follow(Trajectory trajectory) {
        followAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(
                        lastPoseOnTurn.getX(),
                        lastPoseOnTurn.getY(),
                        targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = Drivetrain.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                if (DRAW_PATH_HISTORY) {
                    for (Path path : pathHistory)
                        DashboardUtil.drawSampledPath(fieldOverlay, path);
                }
                else {
                    Trajectory trajectory = follower.getTrajectory();
                    DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                    double t = follower.elapsedTime();
                    DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                    fieldOverlay.setStroke("#3F51B5");
                    DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                }

                if (!follower.isFollowing()) {
                    mode = Drivetrain.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        if (!DRAW_PATH_HISTORY) {
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, currentPose);
        }

        //dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Drivetrain.Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double lf, double lb, double rb, double rf) {
        leftBack.setPower(lb);
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    public void setMotorPowers(double v) {
        this.setMotorPowers(v, v, v, v);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++) {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }

    public void drive(double x, double y, double rotation, double slowModeSpeed, boolean slowModeCondition) {

        double[] wheelSpeeds = new double[4];

        /*wheelSpeeds[0] = x * (slowModeCondition ? smX : 1) + y * (slowModeCondition ? smY : 1) + rotation * (slowModeCondition ? smH : 1); // LF
        wheelSpeeds[1] = x * (slowModeCondition ? smX : 1) - y * (slowModeCondition ? smY : 1) - rotation * (slowModeCondition ? smH : 1); // RF
        wheelSpeeds[2] = x * (slowModeCondition ? smX : 1) - y * (slowModeCondition ? smY : 1) + rotation * (slowModeCondition ? smH : 1); // LB
        wheelSpeeds[3] = x * (slowModeCondition ? smX : 1) + y * (slowModeCondition ? smY : 1) - rotation * (slowModeCondition ? smH : 1); // RB*/

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = x - y - rotation;
        wheelSpeeds[2] = x - y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);

        for (int i = 0; i < 4; i++) {
            if (wheelSpeeds[i] > -0.25 && wheelSpeeds[i] < 0) {
                wheelSpeeds[i] = -0.25;
            } else if (wheelSpeeds[i] < 0.25 && wheelSpeeds[i] > 0) {
                wheelSpeeds[i] = 0.25;
            }
            wheelSpeeds[i] = Math.pow(wheelSpeeds[i], 3);
        }
        normalize(wheelSpeeds);

        for (int i = 0; i < 4; i++)
            wheelSpeeds[i] *= speedMultiplier * (slowModeCondition ? slowModeSpeed : 1);

        setMotorPowers(wheelSpeeds[0], wheelSpeeds[2], wheelSpeeds[3], wheelSpeeds[1]);
    }

    public void resetAngle(){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        System.out.println(angle);
        if(Math.abs(angle)<0.5){
            setMotorPowers(0, 0, 0, 0);
            return;
        }
        if(angle<0){
            angle=Math.max(0.1,Math.min(1,Math.abs(angle)/50));
            setMotorPowers(-angle, -angle, angle, angle);
        }
        else{
            angle=Math.max(0.1,Math.min(1,Math.abs(angle)/50));
            setMotorPowers(angle, angle, -angle, -angle);
        }
    }
    public void turnTo(double degree){
        double angle = degree + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        System.out.println(angle);
        if(Math.abs(angle)<0.5){
            setMotorPowers(0, 0, 0, 0);
            return;
        }
        if(angle<0){
            angle=Math.max(0.1,Math.min(1,Math.abs(angle)/50));
            setMotorPowers(-angle, -angle, angle, angle);
        }
        else{
            angle=Math.max(0.1,Math.min(1,Math.abs(angle)/50));
            setMotorPowers(angle, angle, -angle, -angle);
        }
    }

    public void turnForward(boolean condition) {
        double angle = Math.toRadians(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
        if (condition) turn(angle);
        System.out.println();
        System.out.println("angle" + angle);
    }

    public double[] aim(){
        Pose2d position = getPoseEstimate(); // current position
        // distance from the goal
        double x = 72-position.getX();
        double y = 36-position.getY();
        // solve for the desired pose using trig
        double distance = Math.sqrt(x*x+y*y);
        double degree = Math.toDegrees(Math.atan(-y/x));
        return new double[]{distance,degree};
    }

    public void setDriveTargetPos(int lf, int rf,
                                  int lb, int rb) {
        leftFront.setTargetPosition(lf);
        rightFront.setTargetPosition(rf);
        leftBack.setTargetPosition(lb);
        rightBack.setTargetPosition(rb);
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position (unless timeout has been reached)
     *  2) Driver stops the opmode running.
     */
    public void drive(double speed,
                      double leftInches, double rightInches,
                      double leftBackInches, double rightBackInches,
                      double timeoutSec) {
        int leftTarget;
        int rightTarget;
        int leftBackTarget;
        int rightBackTarget;

        leftTarget = leftFront.getCurrentPosition() + (int) (leftInches * DriveConstants.TICKS_PER_INCH);
        rightTarget = rightFront.getCurrentPosition() + (int) (rightInches * DriveConstants.TICKS_PER_INCH);
        leftBackTarget = leftBack.getCurrentPosition() + (int) (leftBackInches * DriveConstants.TICKS_PER_INCH);
        rightBackTarget = rightBack.getCurrentPosition() + (int) (rightBackInches * DriveConstants.TICKS_PER_INCH);

        setDriveTargetPos(
                leftTarget,
                rightTarget,
                leftBackTarget,
                rightBackTarget
        );

        DcMotor.RunMode originalMode = leftFront.getMode();
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime timer = new ElapsedTime();
        setMotorPowers(Math.abs(speed));

        PIDCoefficients coeffs = new PIDCoefficients(0.05, 0.05, 0.005);
        PIDFController controller = new PIDFController(coeffs);
        while ((timeoutSec <= 0 || timer.seconds() < timeoutSec)
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy()) {
            setMotorPowers(Math.abs(speed) + controller.update(leftFront.getCurrentPosition()));
        }

        setMotorPowers(0);
        setMode(originalMode);
    }

    /** Same params but no timeout */
    public void drive(double speed,
                      double leftInches, double rightInches,
                      double leftBackInches, double rightBackInches) {
        drive(speed, leftInches, rightInches, leftBackInches, rightBackInches, -1);
    }

    /** Same params but no timeout nor back motor args */
    public void drive(double speed, double leftInches, double rightInches, double timeoutSec) {
        drive(speed, leftInches, rightInches, leftInches, rightInches, timeoutSec);
    }

    public void forward(double speed, double distance, double timeoutSec) {
        double converted = distance;
        drive(speed, -converted, -converted, timeoutSec);
    }

    public void forward(double speed, double distance) {
        forward(speed, distance, -1);
    }

    public void backward(double speed, double distance, double timeoutSec) {
        drive(speed, distance, distance, timeoutSec);
    }

    public void backward(double speed, double distance) {
        backward(speed, distance, -1);
    }

    public void turnLeft(double speed, double degrees) {
        drive(speed, degrees, -degrees, degrees, -degrees);
    }

    public void turnRight(double speed, double degrees) {
        drive(speed, -degrees, degrees, -degrees, degrees);
    }

    public void sidestepRight(double speed, double distance) {
        drive(speed, distance, -distance, -distance, distance);
    }

    public void sidestepLeft(double speed, double distance) {
        drive(speed, -distance, distance, distance, -distance);
    }
}
