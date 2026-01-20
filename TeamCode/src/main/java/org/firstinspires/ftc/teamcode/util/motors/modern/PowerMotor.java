package org.firstinspires.ftc.teamcode.util.motors.modern;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.RobotMath;

public class PowerMotor
{
    protected final MotorEx motorEx;
    protected VoltageSensor battery;
    private double voltageCompensation = 13.0;

    protected boolean stopped = false;

    public PowerMotor(MotorEx motorEx)
    {
        this.motorEx = motorEx;

        motorEx.setRunMode(Motor.RunMode.RawPower);
    }

    public PowerMotor setMotorDirection(Motor.Direction direction)
    {
        motorEx.setInverted(direction == Motor.Direction.REVERSE);

        return this;
    }

    public PowerMotor setEncoderDirection(Motor.Direction direction)
    {
        motorEx.encoder.setDirection(direction);

        return this;
    }

    public PowerMotor setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior)
    {
        motorEx.setZeroPowerBehavior(behavior);

        return this;
    }

    /**
     *
     * @param distancePerPulse The desired distance per pulse (in units per tick)
     * @return
     * @see <a href="https://www.chiefdelphi.com/t/encoder-distance-per-pulse/156742/4">What is Encoder Distance per Pulse?</a>
     */
    public PowerMotor setDistancePerPulse(double distancePerPulse)
    {
        motorEx.setDistancePerPulse(distancePerPulse);

        return this;
    }

    /**
     * Typically for velocity; in terms of revolutions completed
     * @param inputGearRatio
     * @param outputGearRatio
     * @return
     */
    public PowerMotor setDistancePerPulse(double inputGearRatio, double outputGearRatio)
    {
        return setDistancePerPulse((1.0 / motorEx.getCPR()) * (inputGearRatio / outputGearRatio));
    }

    /**
     * Typically for position; in terms of displacement in degrees/radians
     * @param inputGearRatio
     * @param outputGearRatio
     * @param inRadians
     * @return
     */
    public PowerMotor setDistancePerPulse(double inputGearRatio, double outputGearRatio, boolean inRadians)
    {
        double revolution = inRadians ? Math.PI * 2 : 360.0;

        return setDistancePerPulse((revolution / motorEx.getCPR()) * (inputGearRatio / outputGearRatio));
    }

    protected PowerMotor setVoltageCompensation(double volts)
    {
        voltageCompensation = volts;

        return this;
    }

    public double getRPM()
    {
        return tpsToRpm(motorEx.getCorrectedVelocity());
    }

    public double getRPMAcceleration()
    {
        return tps2ToRpm2(motorEx.getAcceleration());
    }

    /**
     * ticks of the motor
     * @return
     */
    public int getPosition()
    {
        return motorEx.getCurrentPosition();
    }

    /**
     * distance travelled
     * @return
     */
    public double getDistance()
    {
        return motorEx.getDistance();
    }

    public double getVoltageScale()
    {
        if (battery == null)
        {
            return 1.0;
        }

        return voltageCompensation / battery.getVoltage();
    }

    public void update()
    {
        motorEx.encoder.getRawVelocity();
    }

    public void setPower(double power)
    {
        stopped = false;
        motorEx.set(RobotMath.clamp(power, -1, 1));
    }

    public void stopMotor()
    {
        if (stopped) return;

        motorEx.stopMotor();
        stopped = true;
    }

    public void resetEncoder()
    {
        motorEx.resetEncoder();
    }

    protected double tpsToRpm(double tps)
    {
        double ppr = motorEx.getCPR();

        if (ppr == 0.0) return 0.0;

        return (tps / ppr) * 60.0;
    }

    protected double rpmToTps(double rpm)
    {
        return (rpm * motorEx.getCPR()) / 60.0;
    }

    protected double tps2ToRpm2(double tps2)
    {
        double factor = 60.0 / motorEx.getCPR();
        return tps2 * factor * factor;
    }
}
