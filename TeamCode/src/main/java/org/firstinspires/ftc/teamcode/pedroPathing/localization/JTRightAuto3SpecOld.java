package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTRightAuto3SpecOld extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor arm;
    private DcMotor armOther;
    private Servo claw;

    JArm armPID;

//    private SparkFunOTOS otos;

    private SparkFunOTOS.Pose2D pose;


    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armOther = hardwareMap.get(DcMotor.class,"armOther");

//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        claw = hardwareMap.get(Servo.class, "claw");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

//        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0);

        claw.scaleRange(0.51, 1);

        JTracking tracker = new JTracking(this, hardwareMap);
        /* it's easier to set the back wall to be x = 0, but if we want to reuse positions for
        left and right autos, we have to use specific y values for the starting position. */
        pose = new SparkFunOTOS.Pose2D(0, -24+tracker.robotWidth/2, 0);
        tracker.setPosition(pose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
        // 1st specimen
            claw.setPosition(1);
            arm.setTargetPosition(4180);
            sleep(900);
            tracker.moveTo(25.85, -6.5, 0, 0.1, 0.5, 0.6); // Robot moves to the high chamber
            claw.setPosition(1);
            tracker.setMotorsMecanum(0.3, 0, 0);
            arm.setTargetPosition(2000);
            sleep(800);
            arm.setTargetPosition(3250);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
            sleep(200);

        // human player 1st cycle
            // move to 1
            tracker.moveTo(22, -6.5, 0, 1, 0.5, 0.7);
            arm.setTargetPosition(1785);
            tracker.moveTo(26, -36, 0, 1, 0.5, 0.9);
            tracker.moveTo(50, -36, 0, 1, 0.5,0.9);
            tracker.moveTo(50, -49, 0, 1, 0.5,0.8);
            // 1
            tracker.moveTo(10, -49, 0, 1, 0.5,1.0);
            tracker.moveTo(50, -49, 0, 1, 0.5,1.0);
            tracker.moveTo(50, -58.5, 0, 1, 0.5,0.9);
            // 2
            tracker.moveTo(10, -58.5, 0, 1, 0.5,1.0);
            tracker.setMotorsMecanum(0.15, 0, 0);
            // align with right wall
            tracker.setMotorsMecanum(0, 0.4, 0);
            sleep(500);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(pose.x, -72+tracker.robotWidth/2, 0));
            sleep(150);

            // turn to get 1
            tracker.moveTo(28, -42, 180, 1, 0.5, 0.6);
            tracker.moveTo(7.2, -42, 180, 0.1, 0.5, 0.6);
            claw.setPosition(1);
            sleep(500);
            // back away
            tracker.moveTo(10, -42, 180, 1, 0.5, 0.9);

        // 2nd specimen
            arm.setTargetPosition(4180);
            tracker.moveTo(17, -3.5, 0, 1, 0.5, 0.9);
            tracker.moveTo(25.7, -3.5, 0, 0.1, 0.5, 0.6);
            tracker.setMotorsMecanum(0.2, 0, 0);
            arm.setTargetPosition(2000);
            sleep(800);
            arm.setTargetPosition(3250);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
            sleep(200);

        // human player 2nd cycle
            // get 2
            tracker.moveTo(22, -0.75, 0, 1, 0.5, 0.7);
            arm.setTargetPosition(1785);
            tracker.moveTo(28, -42, 180, 1, 0.5, 0.9);
            tracker.moveTo(7.2, -42, 180, 0.1, 0.5, 0.6);
            claw.setPosition(1);
            sleep(500);
            // back away
            tracker.moveTo(10, -42, 180, 1, 0.5, 0.9);

        // 3rd specimen
            arm.setTargetPosition(4180);
            tracker.moveTo(17, -0.5, 0, 1, 0.5, 0.9);
            tracker.moveTo(25.7, -0.5, 0, 0.1, 0.5, 0.6);
            tracker.setMotorsMecanum(0.2, 0, 0);
            arm.setTargetPosition(2000);
            sleep(800);
            arm.setTargetPosition(3250);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
            sleep(250);
        // park
            tracker.moveTo(5, -42, 0, 1, 0.5, 0.9);

        //stalls until time runs out
            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}