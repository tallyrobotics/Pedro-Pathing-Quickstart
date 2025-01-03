package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTLeftAuto3Samp extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor arm;
    private Servo claw;

    private SparkFunOTOS otos;
    final double robotHalfWidth = 8.25;

    private SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D(0, 24-robotHalfWidth, 0);


    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        claw = hardwareMap.get(Servo.class, "claw");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        claw.scaleRange(0.45, 1);

        JTracking tracker = new JTracking(this, hardwareMap);
        /* it's easier to set the back wall to be x = 0, but if we want to reuse positions for
        left and right autos, we have to use specific y values for the starting position. */
        tracker.setPosition(pose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
        // 1st specimen
            claw.setPosition(1);
            arm.setTargetPosition(4180);
            sleep(1250);
            tracker.moveTo(25.85, 6.75, 0, 0.1, 0.5, 0.6);
            claw.setPosition(1);
            tracker.setMotorsOmni(0.2, 0, 0);
            arm.setTargetPosition(2000);
            sleep(1100);
            arm.setTargetPosition(3250);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
            sleep(500);

        // human player 1st cycle
            // move to 1
            tracker.moveTo(22, 6.75, 0, 2, 0.5, 0.9);
            tracker.moveTo(26, 34, 0, 2, 0.5, 0.9);
            tracker.moveTo(52, 34, 0, 2, 0.5, 0.9);
            tracker.moveTo(52, 46, 0, 2, 0.5,0.8);
            // 1
            tracker.moveTo(10, 50, -7, 2, 0.5,0.9);
            tracker.moveTo(52, 46, 0, 2, 0.5,0.9);
            tracker.moveTo(52, 58, 0, 2, 0.5,0.8);
            // 2
            tracker.moveTo(10, 58, 0, 2, 0.5,0.9);
            tracker.moveTo(52, 58, 0, 2, 0.5,0.9);
            // align with left wall
            tracker.setMotorsOmni(0, -0.4, 0);
            sleep(1000);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(pose.x, 72-robotHalfWidth, 0));
            // move away slightly to prevent dragging on left wall
            tracker.moveTo(50, 71-robotHalfWidth, 0, 0.1, 0.5,0.8);
            // 3
            tracker.moveTo(15, 71-robotHalfWidth, 0, 2, 0.5,0.9);
        // park
            arm.setTargetPosition(5000);
            tracker.moveTo(52.5, 48, -90, 2, 0.5,0.9);
            tracker.moveTo(52.5, 25, -90, 0.5, 0.5, 0.9);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(-0.1);

        //stalls until time runs out
            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}