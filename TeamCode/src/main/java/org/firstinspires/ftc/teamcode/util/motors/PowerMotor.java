package org.firstinspires.ftc.teamcode.util.motors;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * The PowerMotor class provides an abstraction for controlling a motor with additional
 * functionalities such as voltage compensation, encoder direction, and distance per pulse.
 */
public class PowerMotor
{
    // The motor object that this class wraps around
    protected final MotorEx motorEx;

    // Voltage sensor for battery monitoring
    protected VoltageSensor battery;

    // Voltage compensation value
    private double voltageCompensation = 12.0;

    // Flag to indicate if the motor is stopped
    protected boolean stopped = false;

    // Filter factor: 0.0 = infinite smoothing (no change), 1.0 = no smoothing (raw data)
    // Start with 0.8. If still noisy, lower it. If too laggy, raise it.
    private static final double accelFilterFactor = RobotConstants.General.MOTOR_ACCELERATION_FILTER_FACTOR;
    private double lastFilteredAccel = 0.0;

    /**
     * Constructor to initialize the PowerMotor with a MotorEx instance.
     * Sets the motor to run in raw power mode.
     *
     * @param motorEx The MotorEx instance to be controlled.
     */
    public PowerMotor(MotorEx motorEx)
    {
        this.motorEx = motorEx;

        motorEx.setRunMode(Motor.RunMode.RawPower);
        motorEx.resetEncoder();
    }

    /**
     * Constructor to initialize the PowerMotor with a MotorEx instance and a VoltageSensor.
     *
     * @param motorEx The MotorEx instance to be controlled.
     * @param battery The VoltageSensor for monitoring battery voltage.
     */
    public PowerMotor(MotorEx motorEx, VoltageSensor battery)
    {
        this(motorEx);
        this.battery = battery;
    }

    /**
     * Sets the direction of the motor.
     *
     * @param direction The desired motor direction (FORWARD or REVERSE).
     * @return The current PowerMotor instance for method chaining.
     */
    public PowerMotor setMotorDirection(Motor.Direction direction)
    {
        motorEx.setInverted(direction == Motor.Direction.REVERSE);

        return this;
    }

    /**
     * Sets the direction of the encoder.
     *
     * @param direction The desired encoder direction (FORWARD or REVERSE).
     * @return The current PowerMotor instance for method chaining.
     */
    public PowerMotor setEncoderDirection(Motor.Direction direction)
    {
        motorEx.encoder.setDirection(direction);

        return this;
    }

    /**
     * Sets the zero power behavior of the motor.
     *
     * @param behavior The desired zero power behavior (e.g., BRAKE or FLOAT).
     * @return The current PowerMotor instance for method chaining.
     */
    public PowerMotor setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior)
    {
        motorEx.setZeroPowerBehavior(behavior);

        return this;
    }

    /**
     * Sets the distance per pulse for the motor's encoder.
     *
     * @param distancePerPulse The desired distance per pulse (in units per tick).
     * @return The current PowerMotor instance for method chaining.
     * @see <a href="https://www.chiefdelphi.com/t/encoder-distance-per-pulse/156742/4">What is Encoder Distance per Pulse?</a>
     */
    public PowerMotor setDistancePerPulse(double distancePerPulse)
    {
        motorEx.setDistancePerPulse(distancePerPulse);

        return this;
    }

    /**
     * Sets the distance per pulse based on gear ratios, typically for velocity calculations.
     *
     * @param inputGearRatio  The input gear ratio.
     * @param outputGearRatio The output gear ratio.
     * @return The current PowerMotor instance for method chaining.
     */
    public PowerMotor setDistancePerPulse(double inputGearRatio, double outputGearRatio)
    {
        return setDistancePerPulse((1.0 / motorEx.getCPR()) * (inputGearRatio / outputGearRatio));
    }

    /**
     * Sets the distance per pulse based on gear ratios and whether the measurement is in radians.
     * Typically used for position calculations.
     *
     * @param inputGearRatio  The input gear ratio.
     * @param outputGearRatio The output gear ratio.
     * @param angleUnit       If the measurement should be in degrees or radians.
     * @return The current PowerMotor instance for method chaining.
     */
    public PowerMotor setDistancePerPulse(double inputGearRatio, double outputGearRatio, AngleUnit angleUnit)
    {
        double revolution = (angleUnit == AngleUnit.RADIANS) ? Math.PI * 2 : 360.0;

        return setDistancePerPulse((revolution / motorEx.getCPR()) * (inputGearRatio / outputGearRatio));
    }

    /**
     * Sets the voltage compensation value for the motor.
     *
     * @param volts The desired voltage compensation value.
     * @return The current PowerMotor instance for method chaining.
     */
    public PowerMotor setVoltageCompensation(double volts)
    {
        voltageCompensation = volts;

        return this;
    }

    /**
     * Gets the current RPM (Revolutions Per Minute) of the motor.
     *
     * @return The current RPM of the motor.
     */
    public double getRPM()
    {
        return tpsToRpm(motorEx.getCorrectedVelocity()); // consider changing this to motorEx.motorEx.getVelocity()
    }

    /**
     * Gets the current acceleration in RPM^2 of the motor.
     *
     * @return The current acceleration in RPM^2.
     */
    public double getRPMAcceleration()
    {
        double rawAccel = tps2ToRpm2(motorEx.getAcceleration());

        // Low-Pass Filter Implementation (EMA)
        lastFilteredAccel = (accelFilterFactor * rawAccel)
                + ((1.0 - accelFilterFactor) * lastFilteredAccel);

        return lastFilteredAccel;
    }

    /**
     * Gets the current position of the motor in ticks.
     *
     * @return The current position in ticks.
     */
    public int getPosition()
    {
        return motorEx.getCurrentPosition();
    }

    /**
     * Gets the distance traveled by the motor.
     *
     * @return The distance traveled.
     */
    public double getDistance()
    {
        return motorEx.getDistance();
    }

    /**
     * Calculates the voltage scale factor based on the current battery voltage.
     * Ensures the effective voltage does not drop below 10.0V to prevent large multipliers.
     *
     * @return The voltage scale factor.
     */
    public double getVoltageScale()
    {
        if (battery == null) return 1.0;

        // Get voltage, but don't let it drop below 10.0V in the math
        // This prevents massive multipliers if the sensor glitches
        double effectiveVoltage = Math.max(battery.getVoltage(), 10.0);

        return voltageCompensation / effectiveVoltage;
    }

    /**
     * Updates the motor's encoder velocity.
     */
    public void update()
    {
        motorEx.encoder.getRawVelocity();
    }

    /**
     * Sets the power of the motor, applying voltage compensation if necessary.
     *
     * @param power The desired power level (-1 to 1).
     */
    public void setPower(double power)
    {
        stopped = false;
        motorEx.set(MathUtil.clamp(power * getVoltageScale(), -1, 1));
    }

    /**
     * Stops the motor by setting its power to zero.
     * Ensures the motor is not stopped multiple times unnecessarily.
     */
    public void stopMotor()
    {
        if (stopped) return;

        motorEx.stopMotor();
        stopped = true;
    }

    /**
     * Resets the motor's encoder to zero.
     */
    public void resetEncoder()
    {
        motorEx.resetEncoder();
    }

    /**
     * Converts ticks per second (tps) to revolutions per minute (RPM).
     *
     * @param tps The ticks per second.
     * @return The equivalent RPM.
     */
    protected double tpsToRpm(double tps)
    {
        double ppr = motorEx.getCPR();

        if (ppr == 0.0) return 0.0;

        return (tps / ppr) * 60.0;
    }

    /**
     * Converts revolutions per minute (RPM) to ticks per second (tps).
     *
     * @param rpm The revolutions per minute.
     * @return The equivalent ticks per second.
     */
    protected double rpmToTps(double rpm)
    {
        return (rpm * motorEx.getCPR()) / 60.0;
    }

    /**
     * Converts the second derivative of ticks per second (tps^2) to RPM^2.
     *
     * @param tps2 The second derivative of ticks per second.
     * @return The equivalent RPM^2.
     */
    protected double tps2ToRpm2(double tps2)
    {
        double factor = 60.0 / motorEx.getCPR();
        return tps2 * factor * factor;
    }
}
