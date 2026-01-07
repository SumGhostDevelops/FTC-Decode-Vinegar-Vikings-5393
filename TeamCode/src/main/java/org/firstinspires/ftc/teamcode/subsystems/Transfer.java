package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;

public class Transfer extends SubsystemBase
{

    private final ServoEx transfer;
    private boolean leftSide = false;

    public Transfer(ServoEx transfer)
    {
        this.transfer = transfer;
    }

    public void open()
    {
        transfer.set(RobotConstants.Transfer.OPEN_ANGLE);
    }

    public void close()
    {
        transfer.set(RobotConstants.Transfer.CLOSED_ANGLE);
        leftSide = true;
    }

    public void flip()
    {
        if (leftSide)
        {
            transfer.set(-90);
            leftSide = false;
        }
        else
        {
            transfer.set(180);
            leftSide = true;
        }
    }
}
