package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake extends SubsystemBase
{
    private final MotorEx intakeMotor;

    public Intake(MotorEx intakeMotor)
    {
        this.intakeMotor = intakeMotor;
    }
}