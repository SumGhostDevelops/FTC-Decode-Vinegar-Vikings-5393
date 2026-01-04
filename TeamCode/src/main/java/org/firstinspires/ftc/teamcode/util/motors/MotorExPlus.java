package org.firstinspires.ftc.teamcode.util.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.RobotMath;

public class MotorExPlus extends MotorEx
{
    // Basically the same thing as MotorEx but adds built-in functionality for interfacing it w/ RPM

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public MotorExPlus(@NonNull HardwareMap hMap, String id)
    {
        super(hMap, id);
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public MotorExPlus(@NonNull HardwareMap hMap, String id, @NonNull Motor.GoBILDA gobildaType)
    {
        super(hMap, id, gobildaType);
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public MotorExPlus(@NonNull HardwareMap hMap, String id, double cpr, double rpm)
    {
        super(hMap, id, cpr, rpm);
    }

    private double targetRPM = 0;

    /**
     * Number of recent samples the {@link MotorAccelerationTracker} maintains when estimating
     * velocity/acceleration. A window size of 7 was chosen empirically as a
     * balance between responsiveness (smaller window) and noise smoothing
     * (larger window) for typical FTC motor telemetry rates.
     */
    private static final int TRACKER_WINDOW_SIZE = 7;

    private double currentAccel = 0.0;
    private final MotorAccelerationTracker tracker = new MotorAccelerationTracker(TRACKER_WINDOW_SIZE);
    private final ElapsedTime accelTimer = new ElapsedTime();

    private double rpmTolerance = 50;
    private double accelTolerance = 150;

    public double getRpmTolerance()
    {
        return rpmTolerance;
    }

    public void setRpmTolerance(double rpmTolerance)
    {
        if (rpmTolerance < 0)
        {
            throw new IllegalArgumentException("rpmTolerance must be non-negative");
        }
        this.rpmTolerance = rpmTolerance;
    }

    public double getAccelTolerance()
    {
        return accelTolerance;
    }

    public void setAccelTolerance(double accelTolerance)
    {
        if (accelTolerance < 0)
        {
            throw new IllegalArgumentException("accelTolerance must be non-negative");
        }
        this.accelTolerance = accelTolerance;
    }

    public void setRPM(double rpm)
    {
        setVelocity(RobotMath.Motor.rpmToTps(rpm, super.getCPR()));
    }

    // Bypass the SolversLib PIDF velocity controller whatever cus it sucks

    /**
     * @inheritDoc
     */
    public void setVelocity(double velocity)
    {
        targetRPM = RobotMath.Motor.tpsToRpm(velocity, super.getCPR());
        super.motorEx.setVelocity(velocity);
    }

    public double getVelocity()
    {
        return RobotMath.Motor.rpmToTps(getRPM(), super.getCPR());
    }

    public double getRPM()
    {
        return RobotMath.Motor.tpsToRpm(super.getCorrectedVelocity(), super.getCPR());
    }

    public double getAcceleration()
    {
        return currentAccel;
    }

    public double getRPMAcceleration()
    {
        return RobotMath.Motor.tps2ToRpm2(getAcceleration(), super.getCPR());
    }

    /**
     * Updates the acceleration tracker by fetching the latest velocity
     */
    public void updateAcceleration()
    {
        currentAccel = tracker.updateAndGetAcceleration(accelTimer.seconds(), getRPM());
    }

    /**
     * @return If the motor's velocity is at the target (within a tolerance) and its acceleration is at zero (within a tolerance)
     */
    public boolean isStable()
    {
        return Math.abs(getRPM() - targetRPM) <= rpmTolerance && Math.abs(currentAccel) <= accelTolerance;
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f)
    {
        super.motorEx.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setRunMode(RunMode runMode)
    {
        switch (runMode)
        {
            case RawPower:
            {
                setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            }
            case VelocityControl:
            {
                setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
            case PositionControl:
            {
                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            }
        }
    }

    public void setRunMode(DcMotor.RunMode runMode)
    {
        if (super.motorEx.getMode() == runMode)
        {
            return;
        }

        super.motorEx.setMode(runMode);
    }
}