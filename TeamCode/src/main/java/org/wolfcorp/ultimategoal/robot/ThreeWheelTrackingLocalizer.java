package org.wolfcorp.ultimategoal.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.wolfcorp.ultimategoal.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class ThreeWheelTrackingLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 360*4;
    public static double WHEEL_RADIUS = 0.748; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.0; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 7.0; // in; offset of the lateral wheel

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder backEncoder;

    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    public ThreeWheelTrackingLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(-1.25, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(-1.25, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(-FORWARD_OFFSET, -1, Math.toRadians(-90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LF"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RF"));


        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(backEncoder.getCurrentPosition())
        );
}

    public List<Integer> getWheelTicks(){
        return Arrays.asList(
                leftEncoder.getCurrentPosition(),
                rightEncoder.getCurrentPosition(),
                backEncoder.getCurrentPosition()
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()),
                encoderTicksToInches(rightEncoder.getRawVelocity()),
                encoderTicksToInches(backEncoder.getRawVelocity())
        );
    }
}
