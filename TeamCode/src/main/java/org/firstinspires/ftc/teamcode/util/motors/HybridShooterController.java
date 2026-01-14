package org.firstinspires.ftc.teamcode.util.motors;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class HybridShooterController
{

    // Tuning Parameters
    public static double kS = 0.0;  // Static Friction (Base power to move)
    public static double kV = 0.0002; // Velocity Constant (Power per RPM)

    // Gain Scheduling: aggressive when far, gentle when close
    public static double kP_Far = 0.005, kP_Near = 0.002;
    public static double kI = 0.0001;
    public static double kD = 0.0001;

    // Thresholds
    public static double NEAR_THRESHOLD_RPM = 100; // Switch to "Near" gains within this range
    public static double I_ZONE_RPM = 50;          // Only use Integral within this range
    public static double RECOVERY_TRIGGER_RPM = 250; // Error size that triggers "Recovery Mode"

    // Recovery Boost (The "Kick")
    public static double BOOST_POWER = 0.2;     // Extra power added during recovery
    public static double BOOST_DURATION = 0.15; // Seconds to hold the boost

    // Internal State
    private double integralSum = 0;
    private double lastError = 0;
    private boolean isRecovering = false;
    private ElapsedTime recoveryTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();

    private VoltageSensor voltageSensor;

    public HybridShooterController(VoltageSensor voltageSensor)
    {
        this.voltageSensor = voltageSensor;
    }

    public double calculate(double currentRPM, double targetRPM)
    {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt == 0) dt = 0.001; // Safety

        double error = targetRPM - currentRPM;

        // 1. VOLTAGE COMPENSATION
        // Scale the Feedforward based on current battery voltage relative to nominal 12V
        double currentVoltage = voltageSensor.getVoltage();
        double voltageScale = 12.0 / Math.max(currentVoltage, 10.0); // prevent div/0

        // kS (static) + kV (velocity) * target
        double feedforward = (kS + (kV * targetRPM)) * voltageScale;

        // 2. GAIN SCHEDULING
        double currentP = (Math.abs(error) > NEAR_THRESHOLD_RPM) ? kP_Far : kP_Near;

        // 3. INTEGRAL ZONING (I-Zone)
        if (Math.abs(error) < I_ZONE_RPM)
        {
            integralSum += error * dt;
        }
        else
        {
            integralSum = 0; // Reset I if we are too far away (prevents windup)
        }

        // 4. DERIVATIVE
        double derivative = (error - lastError) / dt;
        lastError = error;

        // 5. RECOVERY LOGIC (The "Kick")
        // If error suddenly spikes (shot fired), trigger recovery mode
        if (!isRecovering && error > RECOVERY_TRIGGER_RPM)
        {
            isRecovering = true;
            recoveryTimer.reset();
        }

        double boostTerm = 0;
        if (isRecovering)
        {
            if (recoveryTimer.seconds() < BOOST_DURATION)
            {
                // Apply boost
                boostTerm = BOOST_POWER;
            }
            else
            {
                // Time up, stop recovering
                isRecovering = false;
            }
        }

        // 6. SUMMATION
        double pid = (currentP * error) + (kI * integralSum) + (kD * derivative);
        double output = feedforward + pid + boostTerm;

        // 7. ASYMMETRIC BRAKING (Optional Inspiration)
        // If we are overshooting (error is negative), we might want to cut power harder
        // or let it coast depending on the motor.
        if (output < 0)
        {
            output *= 0.5; // Soften the braking so we don't jitter
        }

        return Math.max(-1.0, Math.min(1.0, output));
    }
}