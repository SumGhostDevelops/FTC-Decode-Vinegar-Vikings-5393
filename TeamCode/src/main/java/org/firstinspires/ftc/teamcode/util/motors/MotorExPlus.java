package org.firstinspires.ftc.teamcode.util.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
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


    public void setRPM(double rpm)
    {
        super.setVelocity(RobotMath.Motor.rpmToTps(rpm, super.getCPR()));
    }

    public double getRPM()
    {
        return RobotMath.Motor.tpsToRpm(super.getCorrectedVelocity(), super.getCPR());
    }

    public double getRPMAcceleration()
    {
        return RobotMath.Motor.tps2ToRpm2(super.getAcceleration(), super.getCPR());
    }
}