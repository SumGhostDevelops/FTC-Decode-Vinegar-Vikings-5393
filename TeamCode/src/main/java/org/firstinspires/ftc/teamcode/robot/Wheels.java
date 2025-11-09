package org.firstinspires.ftc.teamcode.robot;

public class Wheels
{
    // TODO: Consider changing how this class works because it operates more as of a value storer that needs to be manually applied

    public double leftFront;
    public double leftBack;
    public double rightFront;
    public double rightBack;

    public Wheels()
    {
        this.leftFront = 0;
        this.leftBack = 0;
        this.rightFront = 0;
        this.rightBack = 0;
    }

    public Wheels(double leftFront, double leftBack, double rightFront, double rightBack)
    {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }

    public void setAllPower(double power)
    {
        leftFront = power;
        leftBack = power;
        rightFront = power;
        rightBack = power;
    }

    public boolean isNonZeroPower()
    {
        return leftFront != 0 || leftBack != 0 || rightFront != 0 || rightBack != 0;
    }
}