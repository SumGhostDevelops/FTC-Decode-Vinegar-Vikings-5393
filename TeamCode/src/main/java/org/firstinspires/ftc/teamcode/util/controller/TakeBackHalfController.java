package org.firstinspires.ftc.teamcode.util.controller;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

/**
 * Take Back Half (TBH) controller implementation extending PIDFController.
 *
 * TBH is a simple, effective controller for velocity control that:
 * - Accumulates output directly based on error (not time-integrated)
 * - Averages output with previous "take back half" value on zero-crossing
 * - Naturally converges to the correct output without overshoot
 *
 * Usage:
 * - gain: Controls how quickly output accumulates (typical range: 0.0001 -
 * 0.001)
 *
 * The algorithm works by continuously accumulating (gain * error) until the
 * error crosses zero, then averaging the current output with the previous
 * zero-crossing value to rapidly converge on the correct output.
 */
public class TakeBackHalfController extends PIDFController
{
    // The TBH value saved at last zero-crossing
    private double tbhVal = 0;

    // Accumulated output from TBH algorithm
    private double output = 0;

    // Track if this is the first iteration after reset
    private boolean firstIteration = true;

    /**
     * Creates a TakeBackHalfController.
     *
     * @param gain
     *            The TBH gain - controls accumulation rate (stored in kI).
     *            Typical range: 0.0001 to 0.001 for FTC motors.
     */
    public TakeBackHalfController(double gain)
    {
        this(gain, 0);
    }

    /**
     * Creates a TakeBackHalfController with Feedforward.
     *
     * @param gain
     *            The TBH gain - controls accumulation rate (stored in kI).
     * @param f
     *            The Feedforward gain (stored in kF).
     */
    public TakeBackHalfController(double gain, double f)
    {
        super(0, gain, 0, f);
    }

    @Override
    public void reset()
    {
        super.reset();
        tbhVal = 0;
        output = 0;
        firstIteration = true;
    }

    @Override
    protected double calculateOutput(double pv)
    {
        // 1. Update timing and error state
        prevErrorVal = errorVal_p;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        errorVal_p = setPoint - pv;
        measuredValue = pv;

        // 2. Feedforward (The "Floor")
        // Tune this to reach ~80% of your target RPM.
        double ffTerm = kF * setPoint;

        // 3. Proportional (The "Snap")
        // This provides the immediate correction when speed drops.
        // Use a small kP so it doesn't oscillate.
        double pTerm = kP * errorVal_p;

        // 4. Integral Accumulation (TBH Internal)
        // This adds the I-gain per loop cycle.
        output += kI * errorVal_p;

        // 5. Dynamic Asymmetric Clamp (Anti-Windup)
        // Limits the 'output' so (output + ffTerm) never exceeds 1.0.
        // This allows the motor to actually drop power the moment TBH triggers.
        double maxIntegral = 1.0 - ffTerm;
        output = MathUtils.clamp(output, -1.0, maxIntegral);

        // 6. Zero-Crossing Detection (Take Back Half)
        if (!firstIteration && (errorVal_p * prevErrorVal < 0))
        {
            output = 0.5 * (output + tbhVal);
            tbhVal = output;

            // Safety re-clamp
            output = MathUtils.clamp(output, -1.0, maxIntegral);
        }

        firstIteration = false;

        // 7. Final Return
        // Clamp the final sum to valid motor ranges [0.0, 1.0].
        return MathUtils.clamp(output + ffTerm + pTerm, 0.0, 1.0);
    }
}