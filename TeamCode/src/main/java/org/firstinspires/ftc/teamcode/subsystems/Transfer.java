package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;

public class Transfer extends SubsystemBase
{

    private final ServoEx transfer;

    public enum CloseType
    {
        /**
         * Moves the transfer servo in a manner that balls are allowed to enter the ball pit: backward, for transferring
         */
        BACKWARD,
        SHOOT,
        /**
         * Moves the transfer in a manner that balls are not allowed to enter the ball pit: forward, for intaking
         */
        FORWARD
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
    }

    public void close(CloseType type)
    {
        switch (type)
        {
            case FORWARD:
                transfer.set(RobotConstants.Transfer.CLOSED_INTAKE_ANGLE);
                break;
            case BACKWARD:
                transfer.set(RobotConstants.Transfer.CLOSED_FULL_TRANSFER_ANGLE);
                break;
            case SHOOT:
                transfer.set(RobotConstants.Transfer.CLOSED_SHOOTING_TRANSFER_ANGLE);
                break;
        }
    }
}
