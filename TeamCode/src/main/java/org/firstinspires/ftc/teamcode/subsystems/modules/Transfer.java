package org.firstinspires.ftc.teamcode.subsystems.modules;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public class Transfer extends Actuator
{
    public Transfer(RobotHardware robot)
    {
        super(robot.transferServo);
    }
}