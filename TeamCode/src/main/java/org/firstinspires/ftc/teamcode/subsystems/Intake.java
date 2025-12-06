package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public class Intake extends Actuator
{
    private final RobotHardware robot;

    public Intake(RobotHardware robot)
    {
        super(robot.intakeMotor);
        this.robot = robot;
    }
}