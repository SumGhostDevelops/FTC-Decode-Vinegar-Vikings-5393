package org.firstinspires.ftc.teamcode.controls.commands.turret;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.Angle;

public class Aim0 extends CommandBase
{
    private final Turret turret;

    public Aim0(Turret turret)
    {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize()
    {
        turret.aimRelative(new Angle(0, AngleUnit.DEGREES));
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}