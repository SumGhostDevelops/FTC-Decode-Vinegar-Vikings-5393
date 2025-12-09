package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;

public class Drive
{
    private final RobotHardware robot;
    private final Localization localization;
    private DriveMode currentMode = DriveMode.FIELD_CENTRIC;

    public Drive(RobotHardware robot, Localization localization)
    {
        this.robot = robot;
        this.localization = localization;
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
        robot.telemetry.addData("Drive Mode", currentMode.toString());
    }

    public DriveMode getMode()
    {
        return currentMode;
    }

    public void drive(double axial, double lateral, double yaw)
    {
        double botHeading = Math.toRadians(localization.getHeading());
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
                // Hybrid Logic: Point robot front towards input vector
                double magnitude = Math.hypot(axial, lateral);
                if (magnitude > RobotConstants.HYBRID_MODE_DEADBAND)
                {
                    // Angle of the joystick input
                    double moveAngle = Math.atan2(lateral, axial);

                    // Calculate error between move angle and current robot heading
                    // Note: Inputs need to be rotated by heading to be field relative first
                    // if we want "stick points North, robot turns North".
                    // Assuming axial/lateral are Field Centric inputs here:

                    double error = AngleUnit.normalizeRadians(moveAngle - botHeading);

                    // Apply P control to turn robot
                    if (Math.abs(yaw) < 0.05)
                    { // Only auto-turn if driver isn't manually turning
                        rx = Range.clip(error * RobotConstants.HYBRID_MODE_TURN_P, -0.6, 0.6);
                    }
                }
                // Drive vectors remain relative to field input for movement,
                // but since we are rotating the bot to face it, we might want simple robot centric drive
                // usually Hybrid means: Push stick up, robot turns North and drives North.

                // Standard Field Centric translation to ensure movement matches stick direction
                // regardless of current rotation
                rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
                rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
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
        robot.leftFront.setPower(frontLeftPower * powerScale);
        robot.leftBack.setPower(backLeftPower * powerScale);
        robot.rightFront.setPower(frontRightPower * powerScale);
        robot.rightBack.setPower(backRightPower * powerScale);
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
        robot.leftFront.setPower(fl);
        robot.leftBack.setPower(bl);
        robot.rightFront.setPower(fr);
        robot.rightBack.setPower(br);
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
        robot.leftFront.setPower(leftFront);
        robot.leftBack.setPower(leftBack);
        robot.rightFront.setPower(rightFront);
        robot.rightBack.setPower(rightBack);
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