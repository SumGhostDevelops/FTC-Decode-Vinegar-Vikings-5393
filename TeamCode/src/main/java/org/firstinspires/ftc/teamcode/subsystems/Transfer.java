package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer extends Actuator
{
    public Transfer(RobotHardware robot)
    {
        super(robot.transferServo);
    }
}