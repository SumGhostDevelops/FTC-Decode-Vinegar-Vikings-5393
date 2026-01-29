package org.firstinspires.ftc.teamcode.util.motors.modern;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.controller.BangBangController;
import org.firstinspires.ftc.teamcode.util.controller.TakeBackHalfController;

/**
 * The VelocityMotor class extends the PowerMotor class to provide additional
 * functionality for controlling motor velocity using various control algorithms.
 */
public class VelocityMotor extends PowerMotor
{
    // The type of velocity controller being used (e.g., PIDF, BangBang, TakeBackHalf)
    private VelocityController controllerType;

    // The controller instance used for velocity control
    private PIDFController controller;

    // PIDF coefficients for the controller
    private PIDFCoefficients coefficients = new PIDFCoefficients(1, 1, 1, 1);

    // Target RPM for the motor
    private double targetRPM = 0.0;

    // Tolerance for RPM caching
    private double rpmCachingTolerance = 25;

    /**
     * Constructor to initialize the VelocityMotor with a MotorEx instance and a VoltageSensor.
     * Sets the default controller type to PIDF and the zero power behavior to FLOAT.
     *
     * @param motorEx The MotorEx instance to be controlled.
     * @param battery The VoltageSensor for monitoring battery voltage.
     */
    public VelocityMotor(MotorEx motorEx, VoltageSensor battery)
    {
        super(motorEx, battery);

        motorEx.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        setControllerType(VelocityController.PIDF);
    }

    /**
     * Sets the type of velocity controller to be used (e.g., PIDF, BangBang, TakeBackHalf).
     *
     * @param velocityController The desired velocity controller type.
     * @return The current VelocityMotor instance for method chaining.
     */
    public VelocityMotor setControllerType(VelocityController velocityController)
    {
        this.controllerType = velocityController;

        switch (velocityController)
        {
            case PIDF:
                controller = new PIDFController(coefficients);
                break;
            case BangBang:
                controller = new BangBangController(coefficients.p, coefficients.f);
                break;
            case TakeBackHalf:
                controller = new TakeBackHalfController(coefficients.i);
        }

        return this;
    }

    /**
     * Sets the PIDF coefficients for the velocity controller.
     *
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param kf Feedforward gain.
     * @return The current VelocityMotor instance for method chaining.
     */
    public VelocityMotor setPIDF(double kp, double ki, double kd, double kf)
    {
        return setPIDF(new PIDFCoefficients(kp, ki, kd, kf));
    }

    /**
     * Sets the PIDF coefficients for the velocity controller.
     *
     * @param coefficients The PIDFCoefficients object containing the PIDF values.
     * @return The current VelocityMotor instance for method chaining.
     */
    public VelocityMotor setPIDF(PIDFCoefficients coefficients)
    {
        this.coefficients = coefficients;
        controller.setCoefficients(coefficients);

        return this;
    }

    /**
     * Sets the tolerance for the velocity controller.
     *
     * @param rpmTolerance   The tolerance for RPM.
     * @param accelTolerance The tolerance for acceleration.
     * @return The current VelocityMotor instance for method chaining.
     */
    public VelocityMotor setTolerance(double rpmTolerance, double accelTolerance)
    {
        controller.setTolerance(rpmTolerance, accelTolerance);

        return this;
    }

    /**
     * Overriding the zero power behavior is not allowed for VelocityMotor.
     *
     * @param behavior The zero power behavior (ignored).
     * @return The current VelocityMotor instance for method chaining.
     */
    public VelocityMotor setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior)
    {
        return this;
    }

    /**
     * @param volts The desired voltage compensation value.
     * @return
     */
    @Override
    public VelocityMotor setVoltageCompensation(double volts)
    {
        super.setVoltageCompensation(volts);

        return this;
    }

    /**
     * Sets the target RPM as a scale of the motor's maximum RPM.
     *
     * @param scale The scale (0 to 1) of the motor's maximum RPM.
     */
    public void setScale(double scale)
    {
        scale = MathUtil.clamp(scale, 0, 1);

        setTargetRPM(scale * motorEx.getMaxRPM());
    }

    /**
     * Gets the current target RPM of the motor.
     *
     * @return The target RPM.
     */
    public double getTargetRPM()
    {
        return targetRPM;
    }

    /**
     * Sets the target RPM for the motor.
     *
     * @param rpm The desired target RPM.
     */
    public void setTargetRPM(double rpm)
    {
        this.targetRPM = MathUtil.clamp(rpm, 0, motorEx.getMaxRPM());
    }

    /**
     * Gets the RPM caching tolerance.
     *
     * @return The RPM caching tolerance.
     */
    public double getRpmCachingTolerance()
    {
        return rpmCachingTolerance;
    }

    /**
     * Sets the RPM caching tolerance.
     *
     * @param rpm The desired RPM caching tolerance.
     * @return The current VelocityMotor instance for method chaining.
     */
    public VelocityMotor setRpmCachingTolerance(double rpm)
    {
        rpmCachingTolerance = MathUtil.clamp(25, 0, Double.POSITIVE_INFINITY);

        return this;
    }

    /**
     * Checks if the motor is at the set point (target RPM).
     *
     * @return True if the motor is at the set point, false otherwise.
     */
    public boolean atSetPoint()
    {
        return controller.atSetPoint();
    }

    /**
     * Updates the motor's velocity control loop.
     * If the target RPM is zero or less, the motor is stopped.
     * Otherwise, the controller calculates the output power to achieve the target RPM.
     */
    @Override
    public void update()
    {
        super.update();

        if (targetRPM <= 0)
        {
            stopMotor();
            return;
        }

        double currentRPM = getRPM();
        double output = 0;

        // Calculate base output
        switch (controllerType)
        {
            case PIDF:
            case TakeBackHalf:
                output = controller.calculate(currentRPM, targetRPM);
                break;
            case BangBang:
                output = (currentRPM < targetRPM) ? 1.0 : 0.0;
                break;
        }

        // Send to motor (clamped in PowerMotor.setPower)
        setPower(output);
    }

    /**
     * Stops the motor and resets the target RPM to zero.
     */
    @Override
    public void stopMotor()
    {
        super.stopMotor();
        targetRPM = 0.0;
    }

    /**
     * Enum representing the types of velocity controllers available.
     */
    public enum VelocityController
    {PIDF, BangBang, TakeBackHalf}
}