package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class Outtake extends SubsystemBase
{
    MotorGroup outtakeGroup;
    public Outtake(MotorGroup outtakeGroup)
    {
        this.outtakeGroup = outtakeGroup;
    }
}