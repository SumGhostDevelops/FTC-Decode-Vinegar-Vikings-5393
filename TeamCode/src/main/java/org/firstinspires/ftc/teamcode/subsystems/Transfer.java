package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;

public class Transfer extends SubsystemBase
{
    private final ServoEx transfer;

    private final double openAngle = RobotConstants.Transfer.OPEN_ANGLE;
    private final double closeIntakeAngle = RobotConstants.Transfer.CLOSE_INTAKE_ANGLE;

    public Transfer(ServoEx transfer)
    {
        this.transfer = transfer;
    }

    /**
     * releases balls into the outtake
     */
    public void open()
    {
        setAngle(openAngle);
    }

    /**
     * closes the intake (closes forward)
     */
    public void close()
    {
        setAngle(closeIntakeAngle);
    }

    public void setAngle(double angle)
    {
        transfer.set(angle);
    }

    /**
     * @return the last requested input/set position call
     */
    public double getPosition()
    {
        return transfer.get();
    }

    /**
     * @return the raw position of the servo between 0 and 1
     */
    public double getRawPosition()
    {
        return transfer.getRawPosition();
    }
}