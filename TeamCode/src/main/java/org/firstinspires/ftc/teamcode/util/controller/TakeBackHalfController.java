package org.firstinspires.ftc.teamcode.util.controller;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

/**
 * Take Back Half (TBH) implementation extending PIDFController.
 * kI = The integration gain.
 * totalError = The accumulated output.
 */
public class TakeBackHalfController extends PIDFController
{

    private double tbhVal = 0;

    /**
     * @param gain
     *            The integral gain (kI)
     * @param kF
     *            The feedforward term
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
    }

    @Override
    protected double calculateOutput(double pv)
    {
        prevErrorVal = errorVal_p;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        errorVal_p = setPoint - pv;
        measuredValue = pv;

        // 1. Calculate Feedforward separately
        double ffOutput = kF * setPoint;

        // 2. Integrate only if running
        if (Math.abs(period) > 1E-6)
        {
            totalError += kI * errorVal_p * period;
        }

        // 3. ASYMMETRIC CLAMP (The Fix)
        // Prevent the sum (FF + I) from exceeding 1.0 (anti-windup).
        // But allow I to go fully negative (-1.0) so it can brake if needed.
        double maxIntegral = 1.0 - ffOutput;
        totalError = MathUtils.clamp(totalError, -1.0, maxIntegral);

        // 4. Zero Crossing (TBH)
        // Use multiplication for safer sign check
        if (errorVal_p * prevErrorVal < 0)
        {
            totalError = 0.5 * (totalError + tbhVal);
            tbhVal = totalError;
        }

        return totalError + ffOutput;
    }
}