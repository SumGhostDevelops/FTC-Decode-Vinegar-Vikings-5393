package org.firstinspires.ftc.teamcode.util.motors;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class VelocityMotorGroup
{
    public final VelocityMotor[] group;

    public VelocityMotorGroup(VelocityMotor leader)
    {
        group = new VelocityMotor[1];
        group[0] = leader;
    }

    public VelocityMotorGroup(VelocityMotor leader, VelocityMotor... followers)
    {
        group = new VelocityMotor[followers.length + 1];
        group[0] = leader;

        System.arraycopy(followers, 0, group, 1, followers.length);
    }

    public VelocityMotorGroup setControllerType(VelocityMotor.VelocityController velocityController)
    {
        for (VelocityMotor motor : group)
        {
            motor.setControllerType(velocityController);
        }

        return this;
    }

    public VelocityMotorGroup setPIDF(double kp, double ki, double kd, double kf)
    {
        for (VelocityMotor motor : group)
        {
            motor.setPIDF(kp, ki, kd, kf);
        }

        return this;
    }

    public VelocityMotorGroup setPIDF(PIDFCoefficients coefficients)
    {
        for (VelocityMotor motor : group)
        {
            motor.setPIDF(coefficients);
        }

        return this;
    }

    public VelocityMotorGroup setTolerance(double rpmTolerance, double accelTolerance)
    {
        for (VelocityMotor motor : group)
        {
            motor.setTolerance(rpmTolerance, accelTolerance);
        }

        return this;
    }

    public VelocityMotorGroup setScale(double scale)
    {
        for (VelocityMotor motor : group)
        {
            motor.setScale(scale);
        }

        return this;
    }

    public VelocityMotorGroup setVoltageCompensation(double volts)
    {
        for (VelocityMotor motor : group)
        {
            motor.setVoltageCompensation(volts);
        }

        return this;
    }

    public double getTargetRPM()
    {
        return group[0].getTargetRPM();
    }

    public void setTargetRPM(double rpm)
    {
        for (VelocityMotor motor : group)
        {
            motor.setTargetRPM(rpm);
        }
    }

    public double getRpmCachingTolerance()
    {
        return group[0].getRpmCachingTolerance();
    }

    public VelocityMotorGroup setRpmCachingTolerance(double rpm)
    {
        for (VelocityMotor motor : group)
        {
            motor.setRpmCachingTolerance(rpm);
        }

        return this;
    }

    public boolean atSetPoint()
    {
        /*
        for (VelocityMotor motor : group)
        {
            if (!motor.atSetPoint())
            {
                return false;
            }
        }

        return true;
         */

        return group[0].atSetPoint();
    }

    public double getRPM()
    {
        return group[0].getRPM();
    }

    public double[] getRPMs()
    {
        double[] rpms = new double[group.length];

        for (int i = 0; i < group.length; i++)
        {
            rpms[i] = group[i].getRPM();
        }

        return rpms;
    }

    public double getRPMAcceleration()
    {
        return group[0].getRPMAcceleration();
    }

    public void update()
    {
        for (VelocityMotor motor : group)
        {
            motor.update();
        }
    }

    public void stopMotor()
    {
        for (VelocityMotor motor : group)
        {
            motor.stopMotor();
        }
    }
}
