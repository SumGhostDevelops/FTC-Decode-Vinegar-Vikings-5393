package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.modules.odometry.Odometry;

public class Drive
{
    private final Telemetry telemetry;
    private final RobotHardware hw;
    private final Odometry localization;

    private DriveMode currentMode = DriveMode.FIELD_CENTRIC;

    public Drive(RobotHardware hw, Odometry localization, Telemetry telemetry)
    {
        this.hw = hw;
        this.localization = localization;
        this.telemetry = telemetry;
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
        telemetry.addData("Drive Mode", currentMode.toString());
    }

    public DriveMode getMode()
    {
        return currentMode;
    }

    public void drive(double axial, double lateral, double yaw)
    {
        double botHeading = localization.getHeading(AngleUnit.RADIANS, Odometry.AngleType.SIGNED);
        double rotX = lateral;
        double rotY = axial;
        double rx = yaw;

        switch (currentMode)
        {
            case FIELD_CENTRIC:
                rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
                rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
                break;

            case ROBOT_CENTRIC_HYBRID:
                // Agar.io / Slither.io style: Robot faces the direction the joystick points,
                // and drives forward based on joystick magnitude.
                double magnitude = Math.hypot(axial, lateral);
                if (magnitude > RobotConstants.HYBRID_MODE_DEADBAND)
                {
                    // Target heading = direction joystick is pointing (field-relative)
                    // atan2(lateral, axial) gives angle where axial=forward, lateral=right
                    double targetHeading = Math.atan2(lateral, axial);

                    // Calculate shortest angle error to target heading
                    double error = AngleUnit.normalizeRadians(targetHeading - botHeading);

                    // Auto-turn to face target direction (unless driver is manually turning)
                    if (Math.abs(yaw) < 0.05)
                    {
                        rx = Range.clip(error * RobotConstants.HYBRID_MODE_TURN_P, -1.0, 1.0);
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

        double powerScale = RobotConstants.DRIVE_SPEED_MULTIPLIER;
        hw.frontLeft.setPower(frontLeftPower * powerScale);
        hw.backLeft.setPower(backLeftPower * powerScale);
        hw.frontRight.setPower(frontRightPower * powerScale);
        hw.backRight.setPower(backRightPower * powerScale);
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
        hw.frontLeft.setPower(fl);
        hw.backLeft.setPower(bl);
        hw.frontRight.setPower(fr);
        hw.backRight.setPower(br);
    }

    /**
     *
     * @param leftFront
     * @param leftBack
     * @param rightFront
     * @param rightBack
     */
    public void setDrivePowers(double leftFront, double leftBack, double rightFront, double rightBack)
    {
        hw.frontLeft.setPower(leftFront);
        hw.backLeft.setPower(leftBack);
        hw.frontRight.setPower(rightFront);
        hw.backRight.setPower(rightBack);
    }

    public void stop()
    {
        setDrivePowers(0, 0, 0, 0);
    }

    public enum DriveMode
    {
        FIELD_CENTRIC,
        ROBOT_CENTRIC_HYBRID,
        RAW_ROBOT_CENTRIC
    }
}