package org.firstinspires.ftc.teamcode.robot;

public class Wheels
{
    // TODO: Consider changing how this class works because it operates more as of a value storer that needs to be manually applied

    private double leftFront;
    private double leftBack;
    private double rightFront;
    private double rightBack;

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

    public double getLeftFrontPower() {
        return leftFront;
    }

    public double getLeftBackPower() {
        return leftBack;
    }

    public double getRightFrontPower() {
        return rightFront;
    }

    public double getRightBackPower() {
        return rightBack;
    }

    public void setLeftFrontPower(double leftFront) {
        this.leftFront = leftFront;
    }

    public void setLeftBackPower(double leftBack) {
        this.leftBack = leftBack;
    }

    public void setRightFrontPower(double rightFront) {
        this.rightFront = rightFront;
    }

    public void setRightBackPower(double rightBack) {
        this.rightBack = rightBack;
    }

    public void setAllWheelsPower(double power)
    {
        leftFront = power;
        leftBack = power;
        rightFront = power;
        rightBack = power;
    }
}