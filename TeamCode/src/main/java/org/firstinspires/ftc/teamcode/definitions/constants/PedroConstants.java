package org.firstinspires.ftc.teamcode.definitions.constants;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PedroConstants {
    // get mass later
    static double mass = 11.3398; // kg, this is the mass of the robot without any game elements

    public static FollowerConstants followerConstants = new FollowerConstants().mass(mass)
            .forwardZeroPowerAcceleration(-347.24285869156);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotConstants.Drive.WHEEL_NAMES.FRONT_RIGHT)
            .rightRearMotorName(RobotConstants.Drive.WHEEL_NAMES.BACK_RIGHT)
            .leftRearMotorName(RobotConstants.Drive.WHEEL_NAMES.BACK_LEFT)
            .leftFrontMotorName(RobotConstants.Drive.WHEEL_NAMES.FRONT_LEFT)
            // typical sign layout: left motors reversed, right motors forward
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(222.64501842436727)
            .yVelocity(168.03253138683195);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName(RobotConstants.Drive.WHEEL_NAMES.BACK_LEFT)
            .strafeEncoder_HardwareMapName(RobotConstants.Drive.WHEEL_NAMES.BACK_RIGHT)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardPodY(RobotConstants.Odometry.Deadwheels.Forward.OFFSET.toUnit(DistanceUnit.INCH).magnitude)
            // <- corrected to use Strafe offset
            .strafePodX(RobotConstants.Odometry.Deadwheels.Strafe.OFFSET.toUnit(DistanceUnit.INCH).magnitude)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
