package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "blueCloseFinal")
public class blueCloseFinal extends LinearOpMode {



    private IMU imu;
    /**
     * Now let's put both of these programs together in the penultimate IMU sample program.
     * The goal for this program is to have the bot go out 4000 ticks, rotate 180 degrees,
     * then come back to the same position that it started.
     *
     * The one additional item that we will be adding into this program is creating a couple
     * methods. We will create one for forward/backward movement and one for rotating.
     *
     */
    double headingAngle;
    double headingAngle360;

    ImuOrientationOnRobot orientationOnRobot;
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotorEx intake;
    private Servo hoodAngle;
    private Servo gate;

    double shooterVelocity = 1850;
    boolean gateOpen = false;
    boolean startVelocity = true;

    @Override

    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double openPos = 0.37;
        double closePos = 0.63;

        gate.setPosition(closePos);



        imu = hardwareMap.get(IMU.class, "imu");
        orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        waitForStart();

        if (opModeIsActive()) {
            intake.setVelocity(-1000);
            if (startVelocity)
            {
                rightFlywheel.setVelocity(shooterVelocity);
                leftFlywheel.setVelocity(shooterVelocity);
                startVelocity = !startVelocity;
            }
            hoodAngle.setPosition(0.54);
            orientation = imu.getRobotYawPitchRollAngles();
            angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            headingAngle = Double.parseDouble(JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2)) - 20;
            telemetry.addData("Yaw (Z)", headingAngle + " Deg. (Heading)");
            telemetry.addData("shooter velocity",rightFlywheel.getVelocity());
            telemetry.update();


            /**
             * In the previous IMU sample programs, the bulk of our program was taking place right here
             * in the "if (opModeIsActive())" method. All you will see here now are method calls, since
             * the bulk of our programming will be done in those methods.
             *
             * The calls are fairly simple -- we are just sending down the crucial information to the
             * method to make sure the bot does exactly what we'd like it to do. Head down the the methods
             * to see what they are actually doing.
             *
             * Essentially, all this opMode is doing is have the bot go in a small box pattern. It will go
             * forward, then rotate 90 degrees clockwise continuously until it returns to its starting
             * position. It will do this twice.
             *
             * Feel free to edit the program to make the bot go further, faster, or turn at different
             * angles.
             */

            //shoot at the front
            // move("backward", 2000, 0, 1500);
            // sleep(200);
            // gate.setPosition(0.3);
            // sleep(1500);
            // gate.setPosition(0.58);

            // rotate("left",45, 1250);
            // sleep(200);
            // move("forward",1300,45,1500);
            // sleep(200);
            // move("backward",1250,45,1500);
            // sleep(200);
            // rotate("right", 10, 1250);
            // sleep(200);
            // gate.setPosition(0.3);
            // sleep(1500);
            // gate.setPosition(0.58);

            // rotate("left",45, 1250);
            // sleep(200);
            // move("left",1100,45,1500);
            // sleep(200);
            // move("forward",1500,45,1500);
            // sleep(200);
            // move("backward",1450,45,1500);
            // sleep(200);
            // move("right",1100 ,0 ,1500);
            // sleep(200);
            // rotate("right", 10, 1250);
            // sleep(200);
            // gate.setPosition(0.3);
            // sleep(1500);

            // gate.setPosition(0.58);
            // rotate("left",45, 1250);
            // sleep(200);
            // move("left",2300,45,1500);
            // sleep(200);
            // move("forward",1500,45,1500);
            // sleep(200);
            // move("backward",1450,45,1500);
            // sleep(200);
            // move("right",2150 ,0 ,1500);
            // sleep(200);
            // rotate("right", 10, 1250);
            // sleep(200);
            // gate.setPosition(0.3);
            // sleep(1500);


            // gate.setPosition(0.58);
            // move("left",1000 ,0 ,1500);

            //strafe
            move("backward", 2000, 0, 1500);
            sleep(200);
            gate.setPosition(openPos);
            sleep(1500);
            gate.setPosition(closePos);

            rotate("left",45, 1250);
            sleep(400);
            move("forward",1300,45,1500);
            sleep(400);
            move("backward",1150,45,1500);
            sleep(400);
            rotate("right", 10, 1250);
            sleep(400);
            gate.setPosition(openPos);
            sleep(1500);
            gate.setPosition(closePos);

            rotate("left",45, 1250);
            sleep(200);
            move("left",1100,45,1500);
            sleep(200);
            move("forward",1350,45,1500);
            sleep(200);
            rotate("right",0, 1250);
            sleep(200);
            move("right",1800,0,1500);
            sleep(200);
            gate.setPosition(openPos);
            sleep(1500);
            gate.setPosition(closePos);

            rotate("left",45, 1250);
            sleep(200);
            move("left",2350,45,1500);
            sleep(200);
            move("forward",1300,45,1500);
            sleep(200);
            move("right", 1050, 45, 1250);
            sleep(200);
            rotate("right",0, 1250);
            sleep(200);
            move("right",1800,0,1500);
            sleep(200);
            gate.setPosition(openPos);
            sleep(1500);
            gate.setPosition(closePos);
            move("left",1100 ,0 ,1500);

            telemetry.addData("done",true);
            telemetry.update();

        }
    }
    /**
     * Below are two methods created for going forward/backward and rotating. Basically all
     * that was done was copy/pasting the existing code for each movement, then adding some
     * slight changes/parameters that can be used in the method calls above.
     *
     */
    public void move(String direction, int target, double targetAngle, double velocity){
        // Instead of creating two different methods, we added a String parameter where we tell
        // the method the direction, so it can go forward (positive ticks) or backward (negative
        // ticks).

        if(direction == "forward"){
            frontLeftMotor.setTargetPosition(target);
            frontRightMotor.setTargetPosition(target);
            backLeftMotor.setTargetPosition(target);
            backRightMotor.setTargetPosition(target);
        }
        else if(direction == "backward"){
            frontLeftMotor.setTargetPosition(-target);
            frontRightMotor.setTargetPosition(-target);
            backLeftMotor.setTargetPosition(-target);
            backRightMotor.setTargetPosition(-target);
        }
        else if (direction == "left")
        {
            frontLeftMotor.setTargetPosition(-target);
            frontRightMotor.setTargetPosition(target);
            backLeftMotor.setTargetPosition(target);
            backRightMotor.setTargetPosition(-target);
        }
        else if (direction == "right")
        {
            frontLeftMotor.setTargetPosition(target);
            frontRightMotor.setTargetPosition(-target);
            backLeftMotor.setTargetPosition(-target);
            backRightMotor.setTargetPosition(target);
        }

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Below I've created a simple variable that will eventually be used to have particular motors run
        // slower (80% of what the others are running at)
        double slower = 0.8;

        while (opModeIsActive() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            orientation = imu.getRobotYawPitchRollAngles();
            headingAngle = Double.parseDouble(JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
            telemetry.addData("Yaw (Z)", headingAngle + " Deg. (Heading)");
            telemetry.update();

            // The 'if' statements below may seem a little complicated, but it is simply making sure
            // the the robot corrects by turning the right way for 'forward' 'backward' "left' and 'right'
            // directions.
            if((headingAngle > targetAngle+1 && direction == "forward") || (headingAngle < targetAngle-1 && direction == "backward")){
                frontLeftMotor.setVelocity(velocity);
                frontRightMotor.setVelocity(velocity*slower);
                backLeftMotor.setVelocity(velocity);
                backRightMotor.setVelocity(velocity*slower);
            }
            else if((headingAngle < targetAngle-1 && direction == "forward") || (headingAngle > targetAngle+1 && direction == "backward")){
                frontLeftMotor.setVelocity(velocity*slower);
                frontRightMotor.setVelocity(velocity);
                backLeftMotor.setVelocity(velocity*slower);
                backRightMotor.setVelocity(velocity);
            }
            else if((headingAngle > targetAngle+1 && direction == "left") || (headingAngle < targetAngle-1 && direction == "right")){
                frontLeftMotor.setVelocity(velocity*slower);
                frontRightMotor.setVelocity(velocity*slower);
                backLeftMotor.setVelocity(velocity);
                backRightMotor.setVelocity(velocity);
            }
            else if((headingAngle < targetAngle-1 && direction == "left") || (headingAngle > targetAngle+1 && direction == "right")){
                frontLeftMotor.setVelocity(velocity);
                frontRightMotor.setVelocity(velocity);
                backLeftMotor.setVelocity(velocity*slower);
                backRightMotor.setVelocity(velocity*slower);
            }
            else{
                frontLeftMotor.setVelocity(velocity);
                frontRightMotor.setVelocity(velocity);
                backLeftMotor.setVelocity(velocity);
                backRightMotor.setVelocity(velocity);
            }
        }
        // When you complete a directional movement, always reset the encoders

        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void rotate(String direction, int targetAngle, double velocity){
        /**
         * You will notice here that we use the absolute value of the target angle since at 180
         * degrees, the positive and negative flip -- for example if you rotate right past 180
         * degrees, it will go -178, -179, -180, 179, 178, 177.. and continue on. Using absolute
         * value mitigates that.
         *
         * The 'while' loop is important here. Because we are not using a target position for our
         * motors to go to, we need to use our IMU's reading and compare to our target angle.
         * If we are within 1.5 degrees of our target angle, the while loop will stop.
         */


        while (opModeIsActive() && Math.abs(headingAngle-targetAngle) > 1.5) {
            orientation = imu.getRobotYawPitchRollAngles();
            headingAngle = Double.parseDouble(JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));

            if(direction == "right"){
                frontLeftMotor.setVelocity(velocity);
                frontRightMotor.setVelocity(-velocity);
                backLeftMotor.setVelocity(velocity);
                backRightMotor.setVelocity(-velocity);
            }
            else if (direction == "left"){
                frontLeftMotor.setVelocity(-velocity);
                frontRightMotor.setVelocity(velocity);
                backLeftMotor.setVelocity(-velocity);
                backRightMotor.setVelocity(velocity);
            }
            telemetry.addData("Yaw (Z)", headingAngle + " Deg. (Heading)");
            telemetry.addData("Power", frontLeftMotor.getPower());
            telemetry.addData("shooter velocity", rightFlywheel.getVelocity());
            telemetry.update();
        }
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


}

