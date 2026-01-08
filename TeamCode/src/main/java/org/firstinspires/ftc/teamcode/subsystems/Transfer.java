package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;

public class Transfer extends SubsystemBase
{

    private final ServoEx transfer;
    private boolean leftSide = false;

    public enum CloseType
    {
        /**
         * Moves the transfer servo in a manner that balls are allowed to enter the ball pit: backward, for transferring
         */
        TRANSFER,
        /**
         * Moves the transfer in a manner that balls are not allowed to enter the ball pit: forward, for intaking
         */
        INTAKE
    }
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
        transfer.set(RobotConstants.Transfer.CLOSED_INTAKE_ANGLE);
        leftSide = true;
    }

    public void close(CloseType type)
    {
        switch (type)
        {
            case INTAKE:
                transfer.set(RobotConstants.Transfer.CLOSED_INTAKE_ANGLE);
                leftSide = true;
                break;
            case TRANSFER:
                transfer.set(RobotConstants.Transfer.CLOSED_TRANSFER_ANGLE);
                leftSide = false;
                break;
        }
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
