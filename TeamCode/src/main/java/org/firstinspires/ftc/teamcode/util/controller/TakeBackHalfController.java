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
    protected double calculateOutput(double pv) {
        prevErrorVal = errorVal_p;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        errorVal_p = setPoint - pv;
        measuredValue = pv;

        // Calculate the Feedforward component separately
        double ffOutput = kF * setPoint;

        // TBH Integration
        // We only integrate if the output isn't already maxed out (Anti-Windup)
        if (period > 1E-6) {
            totalError += kI * errorVal_p * period;
        }

        // FIX 1: Dynamic Clamping
        // Clamp the integral so that (Integral + FF) never exceeds [-1, 1].
        // This ensures TBH cuts the *real* power, not a theoretical value.
        double maxIntegral = 1.0 - Math.abs(ffOutput);
        totalError = MathUtils.clamp(totalError, -maxIntegral, maxIntegral);

        // FIX 2: Safer Zero Crossing
        // Use multiplication to detect sign change. This prevents double-triggering
        // if the error lands exactly on 0.0.
        if (errorVal_p * prevErrorVal < 0) {
            totalError = 0.5 * (totalError + tbhVal);
            tbhVal = totalError;
        }

        return totalError + ffOutput;
    }
}