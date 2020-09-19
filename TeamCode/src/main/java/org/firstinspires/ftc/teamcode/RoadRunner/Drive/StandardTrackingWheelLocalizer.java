package org.firstinspires.ftc.teamcode.RoadRunner.Drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Util.RevEncoder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

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
public class StandardTrackingWheelLocalizer implements Localizer {
    public static double TICKS_PER_REV = 2048;
    public static double WHEEL_RADIUS = 58/25.4/2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double TRACK_WIDTH = (166.762+192.762)/2/25.4; // in; distance between the left and right wheels (found in cad, might need tuning)
    public static double TURN_ADJUSTMENT_FACTOR = 235.222/25.4; // Adustment to compensate for rotations on the strafe wheel

    public static double OFFSET = 63.81/25.4; //Distance from center of forward odo wheels to robot, used to translate odo centric position to robot centric

    public double leftStartEncoderPosition;
    public double rightStartEncoderPosition;

    public double leftEncoderPosition;
    public double rightEncoderPosition;
    public double strafeEncoderPosition;

    public double lastLeftEncoderPosition;
    public double lastRightEncoderPosition;
    public double lastStrafeEncoderPosition;

    public double leftEncoderDelta;
    public double rightEncoderDelta;
    public double strafeEncoderDelta;

    public double currentRobotX;
    public double currentRobotY;
    public double currentRobotTheta;

    public double lastRobotTheta;
    public double deltaRobotTheta;

    public double compensationEstimate;

    public double relativeY;
    public double relativeX;

    public double movementRadius;
    public double strafeRadius;

    private RevEncoder leftEncoder, rightEncoder, strafeEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        leftEncoder = new RevEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new RevEncoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        strafeEncoder = new RevEncoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(strafeEncoder.getCurrentPosition())
        );
    }

    @NonNull
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(strafeEncoder.getCorrectedVelocity())
        );
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(currentRobotX-Math.cos(currentRobotTheta)*OFFSET, currentRobotY-Math.sin(currentRobotTheta)*OFFSET, currentRobotTheta);
    }

    @Override
    public void update() {
        leftEncoderPosition = encoderTicksToInches(leftEncoder.getCurrentPosition());
        rightEncoderPosition = encoderTicksToInches(rightEncoder.getCurrentPosition());
        strafeEncoderPosition = encoderTicksToInches(strafeEncoder.getCurrentPosition());

        leftEncoderDelta = leftEncoderPosition-lastLeftEncoderPosition;
        rightEncoderDelta = rightEncoderPosition-lastRightEncoderPosition;
        strafeEncoderDelta = strafeEncoderPosition-lastStrafeEncoderPosition;

        //use absolute for robot heading
        currentRobotTheta = AngleWrap(((leftEncoderPosition-leftStartEncoderPosition)-(rightEncoderPosition-rightStartEncoderPosition))/TRACK_WIDTH);
        deltaRobotTheta = AngleWrap(currentRobotTheta - lastRobotTheta);

        relativeY=(leftEncoderDelta+rightEncoderDelta)/2;
        compensationEstimate = deltaRobotTheta * TURN_ADJUSTMENT_FACTOR;
        relativeX = strafeEncoderPosition-compensationEstimate;

        if (Math.abs(deltaRobotTheta) > 0) {
            movementRadius = (leftEncoderDelta+rightEncoderDelta)/(2*deltaRobotTheta);
            strafeRadius = compensationEstimate/deltaRobotTheta;

            relativeY = (movementRadius * Math.sin(deltaRobotTheta)) - (strafeRadius * (1 - Math.cos(deltaRobotTheta)));
            relativeX = movementRadius * (1 - Math.cos(deltaRobotTheta)) + (strafeRadius * Math.sin(deltaRobotTheta));
        }

        currentRobotX += (Math.cos(lastRobotTheta) * relativeY) + (Math.sin(lastRobotTheta) * relativeX);
        currentRobotY += (Math.sin(lastRobotTheta) * relativeY) - (Math.cos(lastRobotTheta) * relativeX);

        lastLeftEncoderPosition=leftEncoderPosition;
        lastRightEncoderPosition=rightEncoderPosition;
        lastStrafeEncoderPosition=strafeEncoderPosition;
        lastRobotTheta=currentRobotTheta;
    }


    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        currentRobotX = pose2d.getX();
        currentRobotY = pose2d.getY();
        currentRobotTheta = pose2d.getHeading();

        leftStartEncoderPosition = encoderTicksToInches(leftEncoder.getCurrentPosition());
        rightStartEncoderPosition = encoderTicksToInches(rightEncoder.getCurrentPosition());

        lastLeftEncoderPosition = leftStartEncoderPosition;
        lastRightEncoderPosition = rightStartEncoderPosition;
        lastStrafeEncoderPosition = encoderTicksToInches(strafeEncoder.getCurrentPosition());
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    public static double AngleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2.0*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2.0*Math.PI;
        }
        return angle;
    }
}
