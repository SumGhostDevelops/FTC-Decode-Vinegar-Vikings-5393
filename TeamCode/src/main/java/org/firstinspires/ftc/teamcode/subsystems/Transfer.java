package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;

import java.util.function.DoubleSupplier;

public class Transfer extends SubsystemBase
{
    private final ServoEx transfer;

    private final DoubleSupplier openAngle = () -> RobotConstants.Transfer.OPEN_ANGLE;
    private final DoubleSupplier closeIntakeAngle = () -> RobotConstants.Transfer.CLOSE_INTAKE_ANGLE;

    private final double minAngle;
    private final double maxAngle;

    private final double angleTolerance;

    public Transfer(ServoEx transfer)
    {
        this(transfer, 0, 360);
    }

    public Transfer(ServoEx transfer, double minAngle, double maxAngle)
    {
        this(transfer, minAngle, maxAngle, 1);
    }

    public Transfer(ServoEx transfer, double minAngle, double maxAngle, double angleTolerance)
    {
        if (maxAngle < minAngle)
        {
            throw new IllegalArgumentException("Minimum angle should be less than maximum angle!");
        }

        if (minAngle < 0)
        {
            throw new IllegalArgumentException("Minimum angle should be greater than or equal to 0!");
        }

        if (angleTolerance < 0)
        {
            throw new IllegalArgumentException("Angle tolerance should be greater than or equal to 0!");
        }

        this.transfer = transfer;

        this.minAngle = minAngle;
        this.maxAngle = maxAngle;

        this.angleTolerance = angleTolerance;

        close();
    }

    /**
     * releases balls into the outtake
     */
    public void open()
    {
        setAngle(openAngle.getAsDouble());
    }

    /**
     * closes the intake (closes forward)
     */
    public void close()
    {
        setAngle(closeIntakeAngle.getAsDouble());
    }

    public void setAngle(double angle)
    {
        transfer.set(angle);
    }

    /**
     * @return the angle of the servo
     */
    public double getAngle()
    {
        return transfer.getRawPosition() * (maxAngle - minAngle) + minAngle;
    }

    /**
     * @return the raw position of the servo between 0 and 1
     */
    public double getRawPosition()
    {
        return transfer.getRawPosition();
    }

    public boolean isAtAngle()
    {
        double lastRequestedAngle = transfer.get();

        return Math.abs(getAngle() - lastRequestedAngle) < angleTolerance;
    }
}