package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.util.MotorExPlusGroup;

public class Outtake extends SubsystemBase
{
    MotorExPlusGroup outtakeGroup;
    public Outtake(MotorExPlusGroup outtakeGroup)
    {
        this.outtakeGroup = outtakeGroup;
    }
}