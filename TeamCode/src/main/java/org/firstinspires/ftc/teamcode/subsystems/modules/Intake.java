package org.firstinspires.ftc.teamcode.subsystems.modules;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public class Intake extends Actuator
{

    public Intake(RobotHardware hw)
    {
        super(hw.intakeMotor);
    }
}