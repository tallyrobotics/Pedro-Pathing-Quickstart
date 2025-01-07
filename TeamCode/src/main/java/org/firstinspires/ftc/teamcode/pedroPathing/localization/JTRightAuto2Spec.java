package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTRightAuto2Spec extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor arm;
    private Servo claw;

//    private SparkFunOTOS otos;

    private SparkFunOTOS.Pose2D pose;


    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
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
            sleep(1250);
            tracker.moveTo(25.85, -6.75, 0, 0.1, 0.5, 0.6); // Robot moves to the high chamber
            claw.setPosition(1);
            tracker.setMotorsOmni(0.2, 0, 0); // Robot moves forward while clipping
            arm.setTargetPosition(2000);
            sleep(1100);
            arm.setTargetPosition(3250);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
            sleep(500);

        // human player 1st cycle
            // move to 1
            tracker.moveTo(22, -6.75, 0, 2, 0.5, 0.9); // Robot moves away from the high chamber.
            arm.setTargetPosition(1800); // Robot moves arm down
            tracker.moveTo(26, -36, 0, 2, 0.5, 0.9); // Robot moves to the right in between strut and the samples
            tracker.moveTo(50, -36, 0, 2, 0.5, 0.9); // Robot moves forward to position itself in front of the samples
            tracker.moveTo(50, -48, 0, 2, 0.5,0.8); // Robot moves right to position itself to collect the leftmost sample
            // 1
            tracker.moveTo(10, -48, 0, 2, 0.5,0.9); // Robot moves backwards to push sample into the HP area
            tracker.moveTo(50, -48, 0, 2, 0.5,0.9); // Robot moves forward to be in front of samples
            tracker.moveTo(50, -60, 0, 2, 0.5,0.8); // Robot moves right to position itself to collect the second sample
            // 2
            tracker.moveTo(10, -60, 0, 1, 0.5,0.9); // Robot moves backward to push sample into the HP area
            // align with right wall
            tracker.setMotorsOmni(0, 0.4, 0);
            sleep(500);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(pose.x, -72+tracker.robotWidth/2, 0));
            sleep(250);

            // turn to get 1
            tracker.moveTo(28, -42, 180, 1, 0.5, 0.6); // Robot turns around to position itself to grab a specimen off of the wall
            tracker.moveTo(7.2, -42, 180, 0.1, 0.5,0.6); // Robot moves forward to put the claw around the specimen
            claw.setPosition(1); // Robot grabs the specimen
            sleep(500);
            // back away
            tracker.moveTo(10, -42, 180, 1, 0.5, 0.9); // Robot backs away from the wall

        // 2nd specimen
            arm.setTargetPosition(4180); // Sets arm to the high chamber height
            tracker.moveTo(17, -0.75, 0, 2, 0.5, 0.9); // Robot moves close to the high chamber
            tracker.moveTo(25.85, -0.75, 0, 0.1, 0.5, 0.6); // Robot moves directly in front of the high chamber
            tracker.setMotorsOmni(0.2, 0, 0); // Robot moves forward slightly while clipping
            arm.setTargetPosition(2000); // Robot arm moves down to clip
            sleep(1100);
            arm.setTargetPosition(3250);
            tracker.stopMotors();
            // open claw
            claw.setPosition(0);
            sleep(500);

        // human player 2nd cycle
            // get 2
            tracker.moveTo(22, -0.75, 0, 1, 0.5, 0.9); // Robot moves away from the high chamber
            arm.setTargetPosition(1800);
            tracker.moveTo(28, -42, 180, 1, 0.5, 0.9); // Robot moves for second wall specimen Y /w 180
            tracker.moveTo(7.2, -42, 180, 0.1, 0.5, 0.6); // Robot moves for second wall specimen X
            claw.setPosition(1); // Grabs the 2nd specimen

        // park
//            tracker.moveTo(5, -42, 0, 1, 0.5, 0.9);

        //stalls until time runs out
            while (opModeIsActive()) {
                sleep(1000);
                claw.setPosition(1); // HOLDS the 2nd specimen
            }
        }
    }
}