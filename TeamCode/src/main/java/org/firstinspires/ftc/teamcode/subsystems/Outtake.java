package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
        return ticksToRpm(robot.outtakeMotor.getVelocity()); // Adjust unit as needed
    }

    public double getTargetRPM()
    {
        return targetRPM;
    }

    public void setRPM(double rpm)
    {
        status = Status.FORWARD_ENABLED;

        targetRPM = rpm;
        robot.outtakeMotor.setVelocity(targetRPM);
    }

    public boolean isReadyToLaunch()
    {
        return Math.abs(getRPM() - targetRPM) < RobotConstants.OUTTAKE_RPM_TOLERANCE;
    }

    private double ticksToRpm(double ticks)
    {
        return (ticks * 60.0) / RobotConstants.OUTTAKE_PPR;
    }

    private double rpmToTicks(double rpm)
    {
        return (rpm * RobotConstants.OUTTAKE_PPR) / 60.0;
    }
}