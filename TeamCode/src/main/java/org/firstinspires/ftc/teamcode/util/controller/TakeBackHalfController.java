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
        // Store previous error for zero-crossing detection
        prevErrorVal = errorVal_p;

        // Update timing
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0)
            lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        measuredValue = pv;

        // --- STEP 1: Calculate Feedforward First ---
        // We need this value to know how much room is left for the integral
        double ffTerm = kF * setPoint;

        // --- STEP 2: Accumulate Output ---
        output += kI * errorVal_p;

        // --- STEP 3: DYNAMIC ANTI-WINDUP (The Critical Fix) ---
        // Calculate how much "headroom" is left after Feedforward.
        // e.g. If FF is 0.9, maxIntegral is 0.1.
        double maxIntegral = 1.0 - ffTerm;

        // Clamp the internal 'output' variable immediately.
        // We allow the low end to go to -1.0 to permit "braking" logic internally,
        // even if your final return limits it to 0.
        output = MathUtils.clamp(output, -1.0, maxIntegral);

        // --- STEP 4: Take Back Half ---
        // Now TBH operates on a value that is essentially "Real Power," not "Virtual Math."
        if (!firstIteration && (errorVal_p * prevErrorVal < 0))
        {
            output = 0.5 * (output + tbhVal);
            tbhVal = output;

            // Re-clamp in case the TBH math pushed it slightly out of bounds (rare but safer)
            output = MathUtils.clamp(output, -1.0, maxIntegral);
        }

        firstIteration = false;

        // --- STEP 5: Return ---
        // Final check to ensure we only send positive voltage (0.0 to 1.0)
        return MathUtils.clamp(output + ffTerm, 0.0, 1.0);
    }
}