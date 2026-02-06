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
 * - gain: Controls how quickly output accumulates (typical range: 0.0001 - 0.001)
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
        super(0, gain, 0, 0);
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
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        // Calculate current error
        errorVal_p = setPoint - pv;
        measuredValue = pv;

        // 1. Accumulate output directly (classic TBH - NOT time-integrated)
        // This provides much faster response than period-based integration
        output += kI * errorVal_p;

        // 2. Clamp output to valid motor power range
        output = MathUtils.clamp(output, 0.0, 1.0);

        // 3. Take Back Half on zero-crossing
        // When error changes sign, average current output with saved TBH value
        // Skip on first iteration to avoid false zero-crossing detection
        if (!firstIteration && (errorVal_p * prevErrorVal < 0))
        {
            output = 0.5 * (output + tbhVal);
            tbhVal = output;
        }

        firstIteration = false;

        return output;
    }
}