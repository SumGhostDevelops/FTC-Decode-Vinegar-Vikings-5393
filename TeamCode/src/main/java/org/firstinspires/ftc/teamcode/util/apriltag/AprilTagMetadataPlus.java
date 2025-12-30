package org.firstinspires.ftc.teamcode.util.apriltag;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

/**
 * A better version of {@link AprilTagMetadata} (cause {@link AprilTagMetadataPlus#fieldPosition} is made infinitely more accessible LOL!)
 */
public class AprilTagMetadataPlus
{
    public final int id;
    public final String name;
    public final double tagSize;
    public final FieldCoordinate fieldPosition;
    public final Quaternion fieldOrientation;

    public AprilTagMetadataPlus(int id, String name, double tagSize, FieldCoordinate fieldPosition, Quaternion fieldOrientation)
    {
        this.id = id;
        this.name = name;
        this.tagSize = tagSize;
        this.fieldPosition = fieldPosition;
        this.fieldOrientation = fieldOrientation;
    }

    public static AprilTagMetadataPlus fromRegular(AprilTagMetadata oldMetadata)
    {
        VectorF fPos = oldMetadata.fieldPosition;
        DistanceUnit posU = oldMetadata.distanceUnit;
        Distance posX = new Distance(fPos.get(0), posU);
        Distance posY = new Distance(fPos.get(1), posU);

        FieldCoordinate fieldPosition = new FieldCoordinate(posX, posY, FieldCoordinate.CoordinateSystem.FTC_STD); // ftc sdk, so they probably use ftc standard

        return new AprilTagMetadataPlus(oldMetadata.id, oldMetadata.name, oldMetadata.tagsize, fieldPosition, oldMetadata.fieldOrientation);
    }
}