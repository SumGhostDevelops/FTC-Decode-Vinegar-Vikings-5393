package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class Drive extends SubsystemBase
{
    private final MotorEx frontLeft, frontRight, backLeft, backRight;
    private final MotorEx[] motors;

    private double speed = RobotConstants.Drive.Speed.DEFAULT;
    private DriveMode currentMode = DriveMode.FIELD_CENTRIC;

    public Drive(MotorEx[] driveMotors)
    {
        this.frontLeft = driveMotors[0];
        this.frontRight = driveMotors[1];
        this.backLeft = driveMotors[2];
        this.backRight = driveMotors[3];
        this.motors = driveMotors;
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

    /**
     * Main drive method.
     * @param lateral Left/Right strafe (Left stick X)
     * @param axial Forward/Back (Left stick Y) - Note: Up should be positive in your command
     * @param yaw Turn (Right stick X)
     * @param driverHeading The robot's heading relative to the driver's forward
     */
    public void drive(double lateral, double axial, double yaw, Angle driverHeading)
    {
        double rotX = lateral;
        double rotY = axial;
        double rx = yaw;

        // Ensure angle is always in radians for Math functions
        double botHeading = driverHeading.getRadians();

        switch (currentMode)
        {
            case FIELD_CENTRIC:
                // Standard Field Centric Math
                rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
                rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

                // Counteract imperfect strafing if needed
                rotX *= 1.1;
                break;

            case ROBOT_CENTRIC_HYBRID:
                double magnitude = Math.hypot(axial, lateral);
                if (magnitude > RobotConstants.Drive.HybridMode.DEADBAND)
                {
                    // Calculate target heading from joystick
                    double targetHeading = Math.atan2(lateral, axial);
                    double error = AngleUnit.normalizeRadians(targetHeading - botHeading);

                    // Auto-turn to face target if not manually turning
                    if (Math.abs(yaw) < 0.05)
                    {
                        rx = Range.clip(error * RobotConstants.Drive.HybridMode.TURN_P, -1.0, 1.0);
                    }

                    // Drive forward in the new direction
                    rotY = magnitude;
                    rotX = 0;
                }
                else
                {
                    rotX = 0;
                    rotY = 0;
                }
                break;

            case RAW_ROBOT_CENTRIC:
                rotX *= 1.1; // Counteract imperfect strafing
                break;
        }

        // Standard Mecanum Kinematics
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double fl = (rotY + rotX + rx) / denominator;
        double bl = (rotY - rotX + rx) / denominator;
        double fr = (rotY - rotX - rx) / denominator;
        double br = (rotY + rotX - rx) / denominator;

        setDrivePowers(fl * speed, fr * speed, bl * speed, br * speed);
    }

    public void setDrivePowers(double fl, double fr, double bl, double br)
    {
        safeSetPower(frontLeft, fl);
        safeSetPower(frontRight, fr);
        safeSetPower(backLeft, bl);
        safeSetPower(backRight, br);
    }

    private void safeSetPower(MotorEx motor, double power)
    {
        if (motor != null)
        {
            motor.set(power);
        }
    }

    public void stop()
    {
        setDrivePowers(0, 0, 0, 0);
    }

    public List<MotorEx> getEnabledMotors()
    {
        return Arrays.stream(motors)
                .filter(Objects::nonNull)
                .collect(Collectors.toList());
    }

    public enum DriveMode
    {
        FIELD_CENTRIC,
        ROBOT_CENTRIC_HYBRID,
        RAW_ROBOT_CENTRIC
    }
}