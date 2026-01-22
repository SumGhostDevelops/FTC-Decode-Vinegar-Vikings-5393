package org.firstinspires.ftc.teamcode.definitions.constants;


import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
            .forwardZeroPowerAcceleration(-70.6038)
            .lateralZeroPowerAcceleration(-130.811)
            .translationalPIDFCoefficients(new PIDFCoefficients(.13,0,.0001,.019))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,.01,.001))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.025,0,.00001,.6,.01));


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
            .xVelocity(-31.40130638956564)
            .yVelocity(-61.18830882220879);
    //drive
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1
    );

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName(RobotConstants.Drive.WHEEL_NAMES.BACK_LEFT)
            .strafeEncoder_HardwareMapName(RobotConstants.Drive.WHEEL_NAMES.BACK_RIGHT)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)

                        .forwardPodY(RobotConstants.Odometry.Deadwheels.Forward.OFFSET.toUnit(DistanceUnit.INCH).magnitude)
            // <- corrected to use Strafe offset
            .strafePodX(RobotConstants.Odometry.Deadwheels.Strafe.OFFSET.toUnit(DistanceUnit.INCH).magnitude)
            .forwardTicksToInches(0.00052844)
         .strafeTicksToInches((0.00042844))
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
