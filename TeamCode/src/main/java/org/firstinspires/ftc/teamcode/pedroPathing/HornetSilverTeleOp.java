
package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp(name = "HornetSilverTeleOp")
public class HornetSilverTeleOp extends LinearOpMode {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotorEx intake;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo hoodAngle;
    private Servo gate;

    double shooterVelocity = 1800;
    boolean gateOpen = false;
    boolean startVelocity = true;
    double openPos = 0.37;
    double closePos = 0.63;

    @Override
    public void runOpMode() {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");

        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        gate.setPosition(closePos);
        hoodAngle.setPosition(0.52);
        imu.resetYaw();
        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            // Put run blocks here.

            imu.resetYaw();
            while (opModeIsActive()) {
                // Put loop blocks here.
                if(startVelocity){
                    rightFlywheel.setVelocity(shooterVelocity);
                    leftFlywheel.setVelocity(shooterVelocity);
                    startVelocity = !startVelocity;
                }


                double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = -gamepad1.left_stick_x*1.1;
                double rx = -gamepad1.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);


                // if (gamepad1.b) {

                // }

                if (gamepad1.right_trigger > 0.2) {
                    gate.setPosition(openPos);

                }
                else {
                    gate.setPosition(closePos);

                }



                if (gamepad1.left_bumper) {
                    intake.setVelocity(1000);
                }
                else if (gamepad1.right_bumper) {
                    intake.setVelocity(-1000);
                }
                else {
                    intake.setPower(0);
                }

                if (gamepad1.dpad_down) {
                    shooterVelocity-=50;
                    rightFlywheel.setVelocity(shooterVelocity);
                    leftFlywheel.setVelocity(shooterVelocity);
                    sleep(300);
                }
                if (gamepad1.dpad_up) {
                    shooterVelocity+=50;
                    rightFlywheel.setVelocity(shooterVelocity);
                    leftFlywheel.setVelocity(shooterVelocity);
                    sleep(300);
                }
                if (gamepad1.dpad_left) {
                    hoodAngle.setPosition(hoodAngle.getPosition()-.01);
                    sleep(300);
                }
                if (gamepad1.dpad_right) {
                    hoodAngle.setPosition(hoodAngle.getPosition()+.01);
                    sleep(300);
                }
                if (gamepad1.a) {
                }

                if (gamepad1.y) {
                    // rightFlywheel.setVelocity(1650);
                    // leftFlywheel.setVelocity(1650);
                }

                if (gamepad1.b) {
                    // rightFlywheel.setVelocity(1500);
                    // leftFlywheel.setVelocity(1500);
                }

                telemetry.addData("Right Velocity: ", rightFlywheel.getVelocity());
                telemetry.addData("Left Velocity: ", leftFlywheel.getVelocity());
                telemetry.addData("Gate Pos: ", gate.getPosition());
                telemetry.update();

            }
        }
    }
}