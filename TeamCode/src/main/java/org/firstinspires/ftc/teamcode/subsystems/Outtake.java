package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public class Outtake extends Actuator
{
    private final RobotHardware robot;
    private double targetRPM = RobotConstants.OUTTAKE_TARGET_RPM;

    public Outtake(RobotHardware robot)
    {
        super(robot.outtakeMotor);
        this.robot = robot;
    }

    public double getRPM()
    {
        return tpsToRpm(robot.outtakeMotor.getVelocity()); // Adjust unit as needed
    }

    public double getTargetRPM()
    {
        return targetRPM;
    }

    public void setRPM(double rpm)
    {
        if (isResetting())
        {
            robot.telemetry.log().add("Ignoring a set RPM command because the motor is currently in a state of resetting.");
            return;
        }

        if (rpm > 6000)
        {
            robot.telemetry.log().add("RPM was attempted to be set over the 6000 RPM limit (" + rpm + "). Capped to 6000.");
            rpm = 6000;
        }

        status = Status.FORWARD_ENABLED;

        targetRPM = rpm;
        robot.outtakeMotor.setVelocity(rpmToTps(targetRPM));
    }

    public boolean isReadyToLaunch()
    {
        return Math.abs(getRPM() - targetRPM) < RobotConstants.OUTTAKE_RPM_TOLERANCE && !isResetting();
    }

    // tps = Ticks per Second; rpm = Revolutions per Minute
    private double tpsToRpm(double ticks)
    {
        return (ticks * 60.0) / RobotConstants.OUTTAKE_PPR;
    }

    private double rpmToTps(double rpm)
    {
        return (rpm * RobotConstants.OUTTAKE_PPR) / 60.0;
    }
}