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

        // Update timing (kept for parent class potential usage, e.g. logging or future
        // expansion)
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0)
            lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        // Current error (errorVal_p) is already calculated in super.calculate()
        // so we don't need to re-calculate it here.
        measuredValue = pv;

        // 1. Accumulate output directly (traditional TBH - loop dependent)
        output += kI * errorVal_p;

        // 2. Take Back Half on zero-crossing
        // When error changes sign, average current output with saved TBH value
        // Skip on first iteration to avoid false zero-crossing detection
        if (!firstIteration && (errorVal_p * prevErrorVal < 0))
        {
            output = 0.5 * (output + tbhVal);
            tbhVal = output;
        }

        firstIteration = false;

        // 3. Return output with Feedforward, clamped to valid motor power range
        return MathUtils.clamp(output + (kF * setPoint), 0.0, 1.0);
    }
}