package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTRightAuto3Spec extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor armOther;
    private Servo elbowClaw;

    JTracking tracker;
    JArm armPID;

    private SparkFunOTOS.Pose2D pose;

    final SparkFunOTOS.Pose2D specimenGrabPose = new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, -55, -90);

    final double specimenPlaceX = 32.5;
    final double specimenInitPlaceY = -6.5;
    final double specimenPlaceYOffset = -3;

    final double sampleX = 57;


    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        armOther = hardwareMap.get(DcMotor.class,"armOther");

//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        elbowClaw = hardwareMap.get(Servo.class, "elbowClaw");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowClaw.scaleRange(0.51, 1);

        tracker = new JTracking(this, hardwareMap);
//        armPID = new JArm(this, hardwareMap);
        /* it's easier to set the back wall to be x = 0, but if we want to reuse positions for
        left and right autos, we have to use specific y values for the starting position. */
        pose = new SparkFunOTOS.Pose2D(JTracking.robotHeight/2, -24+JTracking.robotWidth/2, 0);
        tracker.setPosition(pose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
        // 1st specimen
            elbowClaw.setPosition(1);
//            armPID.setTarget(JArm.specimenPlace);
            sleep(900);
            tracker.moveTo(specimenPlaceX, specimenInitPlaceY, 0, 0.1, 0.5, 0.6); // Robot moves to the high chamber
            elbowClaw.setPosition(1);
            // clip via movement
            tracker.setMotorsMecanum(0.6, 0, 0);
            sleep(800);
            tracker.stopMotors();
            // open claw
            elbowClaw.setPosition(0);
//            armPID.setTarget(JArm.betweenChamber);
            sleep(200);

            // human player 1st cycle
            // move to 1
            tracker.moveTo(29, -6.5, 0, 1, 0.5, 0.7);
//            armPID.setTarget(JArm.specimenGrab);
            tracker.moveTo(33, -36, 0, 1, 0.5, 0.9);
            tracker.moveTo(sampleX, -36, 0, 1, 0.5,0.9);
            tracker.moveTo(sampleX, -49, 0, 1, 0.5,0.8);
            // 1
            tracker.moveTo(17, -49, 0, 1, 0.5,1.0);
            tracker.moveTo(sampleX, -49, 0, 1, 0.5,1.0);
            tracker.moveTo(sampleX, -58.5, 0, 1, 0.5,0.9);
            // 2
            tracker.moveTo(17, -58.5, 0, 1, 0.5,1.0);

            // turn to get 1
            tracker.moveTo(35, -58.5, -90, 1, 0.5, 0.6);
            // square with back wall
            tracker.setMotorsMecanum(0.5, 0.5, 0);
            sleep(500);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, pose.y, -90));
            tracker.moveTo(specimenGrabPose, 0.1, 0.5, 0.6);
            elbowClaw.setPosition(1);
            sleep(500);
            // back away
            tracker.moveTo(JTracking.robotWidth/2, -42, 180, 1, 0.5, 0.9);

        // 2nd specimen
//            armPID.setTarget(JArm.specimenPlace);
            tracker.moveTo(specimenPlaceX-6, specimenInitPlaceY + specimenPlaceYOffset, 0, 1, 0.5, 0.9);
            tracker.moveTo(specimenPlaceX, specimenInitPlaceY + specimenPlaceYOffset, 0, 0.1, 0.5, 0.6);
            // clip via movement
            tracker.setMotorsMecanum(0.6, 0, 0);
            sleep(800);
            tracker.stopMotors();
            // open claw
            elbowClaw.setPosition(0);
//            armPID.setTarget(JArm.betweenChamber);
            sleep(200);

        // human player 2nd cycle
            // get 2
            tracker.moveTo(22, -0.75, 0, 1, 0.5, 0.7);
//            armPID.setTarget(JArm.specimenGrab);
            tracker.moveTo(12, -12, -90, 1, 0.5, 0.9);
            // square with back wall
            tracker.setMotorsMecanum(0.5, 0.5, 0);
            sleep(500);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, pose.y, -90));
            tracker.moveTo(specimenGrabPose, 0.1, 0.5, 0.6);
            elbowClaw.setPosition(1);
            sleep(500);
            // back away
            tracker.moveTo(10, -42, 180, 1, 0.5, 0.9);

        // 3rd specimen
//            armPID.setTarget(JArm.specimenPlace);
            tracker.moveTo(specimenPlaceX-6, specimenInitPlaceY + specimenPlaceYOffset*2, 0, 1, 0.5, 0.9);
            tracker.moveTo(specimenPlaceX, specimenInitPlaceY + specimenPlaceYOffset*2, 0, 0.1, 0.5, 0.6);
            // clip via movement
            tracker.setMotorsMecanum(0.6, 0, 0);
            sleep(800);
            tracker.stopMotors();
            // open claw
            elbowClaw.setPosition(0);
//            armPID.setTarget(JArm.betweenChamber);
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