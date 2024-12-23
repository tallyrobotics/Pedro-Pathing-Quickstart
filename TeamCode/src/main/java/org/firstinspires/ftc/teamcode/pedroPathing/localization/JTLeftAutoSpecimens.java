package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous
public class JTLeftAutoSpecimens extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor arm;
    private Servo claw;

    private SparkFunOTOS otos;


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

        claw.scaleRange(0.5, 1);

        JTracking tracker = new JTracking(this, hardwareMap);
        /* it's easier to set the back wall to be x = 0, but if we want to reuse positions for
        left and right autos, we have to use specific y values for the starting position. */
        tracker.setPosition(new SparkFunOTOS.Pose2D(0, 15.75, 0));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
        // 1st specimen
            claw.setPosition(1);
            arm.setTargetPosition(4180);
            sleep(1500);
            tracker.moveTo(26, 6.75, 0, 0.1, 0.5);
            tracker.setMotors(0.15, 0.15, 0.15, 0.15);
            arm.setTargetPosition(3250);
            sleep(1500);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
            sleep(500);

        // human player 1st cycle
            // move to human player
            tracker.moveTo(22, 6.75, 0, 1, 0.5);
            arm.setTargetPosition(1775);
            tracker.moveTo(26, -36, 0, 1, 0.5);
            tracker.moveTo(52, -36, 0, 1, 0.5);
            tracker.moveTo(52, -48, 0, 1, 0.5);
            // 1
            tracker.moveTo(10, -48, 0, 1, 0.5);
            tracker.moveTo(52, -48, 0, 1, 0.5);
            tracker.moveTo(52, -60, 0, 1, 0.5);
            // 2
            tracker.moveTo(10, -60, 0, 1, 0.5);
            // turn to get 1
            tracker.moveTo(28, -60, 180, 1, 0.5);
            tracker.moveTo(6, -60, 180, 0.1, 0.5);
            claw.setPosition(1);
            sleep(500);
            // back away
            tracker.moveTo(6, -60, 180, 1, 0.5);

        // 2nd specimen
            arm.setTargetPosition(4180);
            tracker.moveTo(26, 6.75, 0, 0.1, 0.5);
            tracker.setMotors(0.15, 0.15, 0.15, 0.15);
            arm.setTargetPosition(3250);
            sleep(1500);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
            sleep(500);

        // human player 2nd cycle
            // get 2
            tracker.moveTo(28, -60, 180, 1, 0.5);
            tracker.moveTo(6, -60, 180, 0.1, 0.5);
            claw.setPosition(1);
            sleep(500);
            // back away
            tracker.moveTo(6, -60, 180, 1, 0.5);

        // 3rd specimen
            arm.setTargetPosition(4180);
            tracker.moveTo(26, 6.75, 0, 0.1, 0.5);
            tracker.setMotors(0.15, 0.15, 0.15, 0.15);
            arm.setTargetPosition(3250);
            sleep(1500);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
        // park
            tracker.moveTo(5, -72, 0, 1, 0.5);

        //stalls until time runs out
            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}