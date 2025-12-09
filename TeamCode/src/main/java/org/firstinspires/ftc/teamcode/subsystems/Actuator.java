package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.BasicEnum;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public abstract class Actuator
{
    protected final ElapsedTime timer = new ElapsedTime();
    private final ActuatorType type;

    private DcMotorEx motor = null;
    private CRServo servo = null;

    protected Status status = Status.IDLE;
    protected double targetTime = 0.0; // Stores the duration requested for Timed actions

    protected enum Status
    {
        FORWARD_ENABLED,
        REVERSE_ENABLED,
        FORWARD_TIMED,
        REVERSE_TIMED,
        RESETTING,
        IDLE
    }

    private enum ActuatorType
    {
        MOTOR,
        SERVO
    }

    protected Actuator(DcMotorEx motor)
    {
        this.motor = motor;
        this.type = ActuatorType.MOTOR;
    }

    protected Actuator(CRServo servo)
    {
        this.servo = servo;
        this.type = ActuatorType.SERVO;
    }

    public void stop()
    {
        if (isResetting())
        {
            return;
        }

        switch (type)
        {
            case MOTOR:
                motor.setPower(0);
                break;
            case SERVO:
                servo.setPower(0);
                break;
        }

        status = Status.IDLE;
    }

    private void setPowerHelper(double power)
    {
        switch (type)
        {
            case MOTOR:
                motor.setPower(power);
                break;
            case SERVO:
                servo.setPower(power);
                break;
        }
    }

    public void setPower(double power)
    {
        if (isResetting())
        {
            return;
        }

        if (power > 0)
        {
            status = Status.FORWARD_ENABLED;
        }
        else
        {
            status = Status.REVERSE_ENABLED;
        }

        setPowerHelper(power);
    }

    public void setDuration(double duration, double power)
    {
        if (isResetting())
        {
            return;
        }

        if (power > 0)
        {
            status = Status.FORWARD_TIMED;
        }
        else
        {
            status = Status.REVERSE_TIMED;
        }

        targetTime = duration;
        timer.reset();

        setPowerHelper(power);
    }

    public void reset()
    {
        status = Status.RESETTING;
        setPowerHelper(-0.5);
    }

    public void stopResetting()
    {
        status = Status.IDLE;
        stop();
    }

    public boolean isIdle()
    {
        return status == Status.IDLE;
    }

    public boolean isResetting()
    {
        return status == Status.RESETTING;
    }

    public void update()
    {
        switch (status)
        {
            case FORWARD_TIMED, REVERSE_TIMED:
                if (timer.seconds() >= targetTime)
                {
                    switch (type)
                    {
                        case MOTOR:
                            motor.setPower(0);
                            break;
                        case SERVO:
                            servo.setPower(0);
                            break;
                    }
                }
                status = Status.IDLE;
                break;

            case FORWARD_ENABLED, REVERSE_ENABLED, IDLE:
                break;
        }
    }
}