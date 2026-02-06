package org.firstinspires.ftc.teamcode.util.controller;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

/**
 * Take Back Half (TBH) controller implementation extending PIDFController.
 *
 * TBH is a simple, effective controller for velocity control that:
 * - Accumulates output directly based on error (not time-integrated)
 * - Averages output with previous "take back half" value on zero-crossing
 * - Uses feedforward for faster initial response
 *
 * Usage:
 * - gain: Controls how quickly output accumulates (typical range: 0.0001 - 0.001)
 * - kF: Feedforward coefficient, normalized (e.g., 1.0 / maxRPM)
 *
 * The feedforward provides immediate power proportional to target, while the
 * TBH mechanism fine-tunes to eliminate steady-state error.
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
     * @param kF
     *            The feedforward coefficient, should be normalized.
     *            For velocity control: kF ≈ 1.0 / maxRPM
     */
    public TakeBackHalfController(double gain, double kF)
    {
        super(0, gain, 0, kF);
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

        // 1. Calculate normalized feedforward
        // kF should be pre-normalized (e.g., 1/maxRPM), so kF * setPoint gives [0, 1]
        double ffOutput = kF * setPoint;

        // 2. Accumulate output directly (classic TBH - NOT time-integrated)
        // This provides much faster response than period-based integration
        output += kI * errorVal_p;

        // 3. Clamp output to valid range, accounting for feedforward
        // Allow negative values for braking when overshooting
        double maxOutput = 1.0 - ffOutput;
        double minOutput = -ffOutput; // Allow output to cancel FF completely for braking
        output = MathUtils.clamp(output, minOutput, maxOutput);

        // 4. Take Back Half on zero-crossing
        // When error changes sign, average current output with saved TBH value
        // Skip on first iteration to avoid false zero-crossing detection
        if (!firstIteration && (errorVal_p * prevErrorVal < 0))
        {
            output = 0.5 * (output + tbhVal);
            tbhVal = output;
        }

        firstIteration = false;

        // 5. Combine feedforward and TBH output
        double totalOutput = output + ffOutput;

        // Final clamp to ensure output is in valid range [0, 1] for motor power
        return MathUtils.clamp(totalOutput, 0.0, 1.0);
    }
}