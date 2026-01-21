package org.firstinspires.ftc.teamcode.util.controller;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;

/**
 * Bang-Bang implementation extending PIDFController.
 * kP = High Output (Power applied when below setpoint)
 * kF = Low Output (Power applied when at or above setpoint)
 */
public class BangBangController extends PIDFController
{

    /**
     * // kP = high, kI = 0, kD = 0, kF = low
     * @param highOutput
     * @param lowOutput
     */
    public BangBangController(double highOutput, double lowOutput)
    {
        super(highOutput, 0, 0, lowOutput);
    }

    @Override
    protected double calculateOutput(double pv)
    {
        // Use super to update errorVal_p, period, and lastTimeStamp
        // We ignore the return value of super.calculateOutput because it's linear PID
        updateState(pv);

        // Bang-Bang logic: If current < target (positive error), use kP. Else use kF.
        return (errorVal_p > 0) ? kP : kF;
    }

    /**
     * Updates internal error states without performing PID math.
     */
    private void updateState(double pv)
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
    }
}