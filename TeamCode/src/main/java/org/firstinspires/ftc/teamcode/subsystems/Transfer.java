package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public class Transfer extends SubsystemBase
{
    private final ServoEx transferServo;

    public Transfer(ServoEx transferServo)
    {
        this.transferServo = transferServo;
    }
}