package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public class LoaderIntake
{
    private final RobotHardware robot;

    public LoaderIntake(RobotHardware robot)
    {
        this.robot = robot;
    }

    public void intakeForward()
    {
        if (robot.intakeMotor != null) robot.intakeMotor.setPower(RobotConstants.INTAKE_POWER_FWD);
    }

    public void intakeReverse()
    {
        if (robot.intakeMotor != null) robot.intakeMotor.setPower(RobotConstants.INTAKE_POWER_REV);
    }

    public void stopIntake()
    {
        if (robot.intakeMotor != null) robot.intakeMotor.setPower(0);
    }

    public void loadBall()
    {
        robot.loaderServo.setPower(RobotConstants.LOADER_POWER_LOAD);
    }

    public void stopLoader()
    {
        robot.loaderServo.setPower(RobotConstants.LOADER_POWER_IDLE);
    }

    // Placeholder ball detection
    public boolean isBallLoaded()
    {
        // In future, check distance sensor or color sensor here
        return false;
    }
}