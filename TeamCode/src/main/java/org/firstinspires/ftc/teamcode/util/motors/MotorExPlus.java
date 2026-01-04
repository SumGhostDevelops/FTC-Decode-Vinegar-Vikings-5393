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
    public MotorExPlus(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public MotorExPlus(@NonNull HardwareMap hMap, String id, @NonNull Motor.GoBILDA gobildaType) {
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
    public MotorExPlus(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }

    private double currentAccel = 0.0;
    private final Tracker tracker = new Tracker(7);
    private final ElapsedTime accelTimer = new ElapsedTime();

    public void setRPM(int rpm)
    {
        setVelocity(RobotMath.Motor.rpmToTps(rpm, super.getCPR()));
    }

    // Bypass the SolversLib PIDF velocity controller whatever cus it sucks
    /**
     * @inheritDoc
     */
    public void setVelocity(double velocity)
    {
        super.motorEx.setVelocity(velocity);
    }

    public double getVelocity()
    {
        double currentTicks = super.getCorrectedVelocity();
        currentAccel = tracker.updateAndGetAcceleration(accelTimer.seconds(), currentTicks);

        return currentTicks;
    }

    public double getRPM()
    {
        return RobotMath.Motor.tpsToRpm(getVelocity(), super.getCPR());
    }

    public double getAcceleration()
    {
        return currentAccel;
    }

    public double getRPMAcceleration()
    {
        return RobotMath.Motor.tps2ToRpm2(getAcceleration(), super.getCPR());
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
            }
        }
    }

    public void setRunMode(DcMotor.RunMode runMode)
    {
        super.motorEx.setMode(runMode);
    }
}