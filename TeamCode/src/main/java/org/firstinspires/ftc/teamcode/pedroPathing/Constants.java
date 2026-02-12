package org.firstinspires.ftc.teamcode.pedroPathing;

<<<<<<< Updated upstream
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(10.84086)
        .forwardZeroPowerAcceleration(-34.09)
            //32.531 -> 33.18
            //33.18 -> 43.32
        .lateralZeroPowerAcceleration(-57.719)
            //45.557 -> 52.3
            //52.3 -> -53.33
        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.04))

        .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, .03))

        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.001,0.6,0.035))
        .centripetalScaling(0.0003);
    //.000025 -> .001
    //.001 -> .0003




    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1)    ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()

            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.665)
            .yVelocity(52.489)

            ;
            //86.9 -> 89.6
            //89.6 -> 72.94
            //.yVelocity(56.39);

            //76.42 -> 75.919
            //75.1 -> 56.39
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7)
            //0
            //4.5 -> -4.5
            .strafePodX(-6)
            //.67
            //7.5 -> -6.25
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



}


=======
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}
>>>>>>> Stashed changes
