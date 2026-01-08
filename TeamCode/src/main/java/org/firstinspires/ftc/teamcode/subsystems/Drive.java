package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

import java.util.ArrayList;
import java.util.List;

public class Drive extends SubsystemBase
{
    private final MotorEx frontLeft, frontRight, backLeft, backRight;

    private double speed = RobotConstants.Drive.Speed.DEFAULT;
    private DriveMode currentMode = DriveMode.RAW_ROBOT_CENTRIC;

    public Drive(MotorEx[] driveMotors)
    {
        frontLeft = driveMotors[0];
        frontRight = driveMotors[1];
        backLeft = driveMotors[2];
        backRight = driveMotors[3];
    }

    public DriveMode getMode()
    {
        return currentMode;
    }

    public boolean motorEnabled(DriveMotor motor)
    {
        switch (motor)
        {
            case FRONT_LEFT:
                return frontLeft != null;
            case FRONT_RIGHT:
                return frontRight != null;
            case BACK_LEFT:
                return backLeft != null;
            case BACK_RIGHT:
                return backRight != null;
            default:
                return false;
        }
    }

    public boolean motorEnabled(MotorEx motor)
    {
        return motor != null;
    }

    public List<DriveMotor> enabledMotors()
    {
        List<DriveMotor> enabledMotors = new ArrayList<>();

        if (motorEnabled(frontLeft)) enabledMotors.add(DriveMotor.FRONT_LEFT);
        if (motorEnabled(frontRight)) enabledMotors.add(DriveMotor.FRONT_RIGHT);
        if (motorEnabled(backLeft)) enabledMotors.add(DriveMotor.BACK_LEFT);
        if (motorEnabled(backRight)) enabledMotors.add(DriveMotor.BACK_RIGHT);

        return enabledMotors;
    }

    public void toggleDriveMode()
    {
        switch (currentMode)
        {
            case FIELD_CENTRIC:
                currentMode = DriveMode.ROBOT_CENTRIC_HYBRID;
                break;
            case ROBOT_CENTRIC_HYBRID:
                currentMode = DriveMode.RAW_ROBOT_CENTRIC;
                break;
            case RAW_ROBOT_CENTRIC:
                currentMode = DriveMode.FIELD_CENTRIC;
                break;
        }
    }

    public void drive(double lateral, double axial, double yaw, Angle driverHeading)
    {
        double rotX = lateral;
        double rotY = axial;
        double rx = yaw;

        double botHeading = driverHeading.toUnit(AngleUnit.RADIANS).measure; // ensure angle is always in radians

        switch (currentMode)
        {
            case FIELD_CENTRIC:
                rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
                rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
                break;

            case ROBOT_CENTRIC_HYBRID:
                // Agar.io / Slither.io style: Robot faces the direction the joystick points,
                // and drives forward based on joystick magnitude.
                double magnitude = Math.hypot(axial, lateral);
                if (magnitude > RobotConstants.Drive.HybridMode.DEADBAND)
                {
                    // Target heading = direction joystick is pointing (field-relative)
                    // atan2(lateral, axial) gives angle where axial=forward, lateral=right
                    double targetHeading = Math.atan2(lateral, axial);

                    // Calculate shortest angle error to target heading
                    double error = AngleUnit.normalizeRadians(targetHeading - botHeading);

                    // Auto-turn to face target direction (unless driver is manually turning)
                    if (Math.abs(yaw) < 0.05)
                    {
                        rx = Range.clip(error * RobotConstants.Drive.HybridMode.TURN_P, -1.0, 1.0);
                    }

                    // Drive forward (robot-centric) based on joystick magnitude
                    // Since robot is turning to face the joystick direction, driving forward
                    // will move the robot in that direction
                    rotY = magnitude;  // Forward
                    rotX = 0;          // No strafing
                }
                else
                {
                    // Below deadband: no movement, allow rotation in place
                    rotX = 0;
                    rotY = 0;
                }
                break;

            case RAW_ROBOT_CENTRIC:
                // Pass through
                break;
        }

        rotX *= 1.1; // Counteract imperfect strafing

        // Mecanum Math
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        double powerScale = speed;

        safeSetPower(frontLeft, frontLeftPower * powerScale);
        safeSetPower(frontRight, frontRightPower * powerScale);
        safeSetPower(backLeft, backLeftPower * powerScale);
        safeSetPower(backRight, backRightPower * powerScale);
    }

    // Inside Drive.java

    public void setDrivePowers(double forward, double strafe, double turn) {
        // Standard Mecanum Math
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        double fl = (forward + strafe + turn) / denominator;
        double bl = (forward - strafe + turn) / denominator;
        double fr = (forward - strafe - turn) / denominator;
        double br = (forward + strafe - turn) / denominator;

        // Apply to hardware
        safeSetPower(frontLeft, fl);
        safeSetPower(backLeft, bl);
        safeSetPower(frontRight, fr);
        safeSetPower(backRight, br);
    }


    public void setDrivePowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower)
    {
        safeSetPower(frontLeft, frontLeftPower);
        safeSetPower(frontRight, frontRightPower);
        safeSetPower(backLeft, backLeftPower);
        safeSetPower(backRight, backRightPower);
    }

    public void safeSetPower(MotorEx motor, double power)
    {
        if (!motorEnabled(motor))
        {
            return;
        }

        motor.set(power);
    }

    public void stop()
    {
        setDrivePowers(0, 0, 0, 0);
    }

    public void increaseSpeed()
    {
        speed = Math.min(RobotConstants.Drive.Speed.MAXIMUM, speed + RobotConstants.Drive.Speed.CHANGE);
    }

    public void decreaseSpeed()
    {
        speed = Math.max(RobotConstants.Drive.Speed.MINIMUM, speed - RobotConstants.Drive.Speed.CHANGE);
    }

    public double getSpeed()
    {
        return speed;
    }

    public enum DriveMode
    {
        FIELD_CENTRIC,
        ROBOT_CENTRIC_HYBRID,
        RAW_ROBOT_CENTRIC
    }

    public enum DriveMotor
    {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }
}