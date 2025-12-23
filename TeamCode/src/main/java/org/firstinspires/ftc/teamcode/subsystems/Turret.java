package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Turret extends SubsystemBase
{
    private final MotorEx turretMotor;

    public Turret(MotorEx turretMotor)
    {
        this.turretMotor = turretMotor;
    }

    public void aim(double angle)
    {

    }

    public void reset()
    {
        aim(0);
    }
}