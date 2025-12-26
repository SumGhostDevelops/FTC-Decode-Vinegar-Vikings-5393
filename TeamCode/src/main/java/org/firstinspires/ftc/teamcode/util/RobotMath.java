package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class RobotMath
{

    public static class Motor
    {
        public double tpsToRpm(double tps, double ppr)
        {
            return (tps / ppr) * 60.0;
        }

        public double rpmToTps(double rpm, double ppr)
        {
            return (rpm * ppr) / 60.0;
        }

        public double tps2ToRpm2(double tps2, double ppr)
        {
            // Square the conversion factor (60/ppr)
            double factor = 60.0 / ppr;
            return tps2 * (factor * factor);
        }

        public double rpm2ToTps2(double rpm2, double ppr)
        {
            // Square the inverse conversion factor (ppr/60)
            double factor = ppr / 60.0;
            return rpm2 * (factor * factor);
        }
    }

    public static class Outtake
    {
        private final InterpLUT lut = new InterpLUT();
        private boolean lutInited = false;

        public void initLUT()
        {
            /* example stuff
            //Adding each val with a key
            lut.add(1.1, 0.2);
            lut.add(2.7, .5);
            lut.add(3.6, 0.75);
            lut.add(4.1, 0.9);
            lut.add(5, 1);
            //generating final equation
            lut.createLUT();
             */
        }

        public double rpmLUT(double distance)
        {
            if (!lutInited)
            {
                initLUT();
            }

            return lut.get(distance);
        }
    }
}