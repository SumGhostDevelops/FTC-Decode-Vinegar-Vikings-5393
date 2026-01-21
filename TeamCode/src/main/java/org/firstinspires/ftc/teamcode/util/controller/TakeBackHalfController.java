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
     * // kP = 0, kI = gain, kD = 0, kF = 0
     * @param gain
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

        if (Math.abs(period) > 1E-6)
        {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        }

        // TBH Integration into totalError
        totalError += kI * errorVal_p * period;

        // Clamp totalError to motor limits [-1, 1]
        totalError = MathUtils.clamp(totalError, -1.0, 1.0);

        // Zero Crossing Detection
        if (Math.signum(errorVal_p) != Math.signum(prevErrorVal))
        {
            totalError = 0.5 * (totalError + tbhVal);
            tbhVal = totalError;
        }

        return totalError;
    }
}