package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.motors.MotorExPlusGroup;

public class Outtake extends SubsystemBase
{
    MotorExPlusGroup outtakeGroup;
    public Outtake(MotorExPlusGroup outtakeGroup)
    {
        this.outtakeGroup = outtakeGroup;
    }
}