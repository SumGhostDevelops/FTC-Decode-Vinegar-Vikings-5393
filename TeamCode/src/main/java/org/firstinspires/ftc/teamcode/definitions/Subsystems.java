package org.firstinspires.ftc.teamcode.definitions;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.odometry.OdometryControlHub;

public class Subsystems
{
    public final Drive drive;
    public final Intake intake;
    public final Transfer transfer;
    public final Outtake outtake;
    public final Turret turret;

    public OdometryControlHub odometry;

    public Subsystems(RobotHardware hw)
    {
        this.drive = new Drive(hw.getDriveArray());
        this.intake = new Intake(hw.intake);
        this.transfer = new Transfer(hw.transfer);
        this.outtake = new Outtake(hw.outtake);
        this.turret = new Turret(hw.turret, RobotConstants.Turret.FORWARD_ANGLE, true);
        this.odometry = new OdometryControlHub(hw.webcam, hw.imu, hw.dwFwd, hw.dwStrf);
    }
}
