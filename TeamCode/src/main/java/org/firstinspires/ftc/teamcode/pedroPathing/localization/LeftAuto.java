package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class LeftAuto extends LinearOpMode {
    private IMU imu;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private DcMotor arm;
    private DcMotor armOther;
    private Servo claw;
    private DcMotor wrist;
    private AnalogInput pot;

    @Override
    public void runOpMode() {
        pot = hardwareMap.get(AnalogInput.class, "pot");
        imu = hardwareMap.get(IMU.class, "imu");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        arm.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.7);

        claw.scaleRange(0.5, 1);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        claw.setPosition(1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            claw.setPosition(1);
            imu.resetYaw();
            telemetry.addData("Status", "Running");
            telemetry.update();

            // raising arm
            arm.setTargetPosition(4200);
            sleep(1500);

            // move forward to get into position to place specimen
            move(0.4, 1.42);
            claw.setPosition(1);
            sleep(1000);

            // drop arm
            backLeft.setPower(0.12);
            backRight.setPower(0.12);
            frontLeft.setPower(0.12);
            frontRight.setPower(0.12);
            arm.setTargetPosition(400);
            sleep(1500);
            stopMotors();
            arm.setPower(0);
            // open claw
            claw.setPosition(0);
            // raise arm out of the beam
            arm.setPower(0.5);
            arm.setTargetPosition(3250);
            sleep(1500);

            // 3 samples and park
            move(-0.4, 0.75);
            arm.setTargetPosition(4500);
            sleep(150);
            strafe(-0.8, 0.95);
            sleep(150);
            move(0.8, 1);
            sleep(150);
            // first cycle
            strafe(-0.8, 0.35);
            sleep(100);
            turnClockwiseToAngle(8);
            move(-0.8, 1.15);
            turnCounterclockwiseToAngle(8);
            moveStraight(0.8, 1.15);
            //second cycle
            strafe(-0.8, 0.4);
            moveStraight(-0.8, 1.35);
            moveStraight(0.8, 1.4);
            // third cycle
            strafe(-0.8, 0.43);
            moveStraight(-0.8, 1.2);
            moveStraight(0.8, 1.3);
            strafe(0.6, 1.0);
            turnClockwiseToAngle(89);
            move(0.4, 1.0);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(-0.05);
            sleep(2000);
            arm.setPower(0);

            // just to make sure everything is completely off
            stopMotors();

            while (opModeIsActive()) {
                sleep(1000);
            }

        }
    }

    public void stopMotors() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    public void strafe(double power, double time) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
        double previousHeading = currentHeading;
        double headingDiff = 0.0;
        double target = currentHeading;
        double headingError = 0.0;

        double startTime = getRuntime();
        double yaw = 0;


        while ((getRuntime() < (startTime + time)) && (opModeIsActive())) {
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
            headingDiff = currentHeading - previousHeading;
            if (Math.abs(headingDiff + 360) < Math.abs(headingDiff)) {
                headingDiff += 360;
            } else if (Math.abs(headingDiff - 360) < Math.abs(headingDiff)) {
                headingDiff -= 360;
            }
            headingError = target - currentHeading;
            if (Math.abs(headingError + 360) < Math.abs(headingError)) {
                headingError += 360;
            } else if (Math.abs(headingError - 360) < Math.abs(headingError)) {
                headingError -= 360;
            }

            // negative around whole thing because positive yaw makes robot turn right while heading is measured counterclockwise
            yaw = -(headingError * 0.03 - headingDiff * 0.02);

            backLeft.setPower(-power + yaw);
            backRight.setPower(power - yaw);
            frontLeft.setPower(power + yaw);
            frontRight.setPower(-power - yaw);

            previousHeading = currentHeading;
        }

        stopMotors();
    }

    public void turnClockwiseToAngle(double angle) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
        double idealHeading = currentHeading - angle;

        if (Math.abs(idealHeading + 360) < Math.abs(idealHeading)) {
            idealHeading += 360;
        } else if (Math.abs(idealHeading - 360) < Math.abs(idealHeading)) {
            idealHeading -= 360;
        }

        double headingError = idealHeading - currentHeading;
        if (Math.abs(headingError + 360) < Math.abs(headingError)) {
            headingError += 360;
        } else if (Math.abs(headingError - 360) < Math.abs(headingError)) {
            headingError -= 360;
        }

        while ((headingError < -3) && (opModeIsActive())) {
            telemetry.addData("Error",headingError);
            telemetry.update();

            currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
            headingError = idealHeading - currentHeading;
            if (Math.abs(headingError + 360) < Math.abs(headingError)) {
                headingError += 360;
            } else if (Math.abs(headingError - 360) < Math.abs(headingError)) {
                headingError -= 360;
            }

            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
        }
        stopMotors();
    }

    public void turnCounterclockwiseToAngle(double angle) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
        double idealHeading = currentHeading + angle;
        if (Math.abs(idealHeading + 360) < Math.abs(idealHeading)) {
            idealHeading += 360;
        } else if (Math.abs(idealHeading - 360) < Math.abs(idealHeading)) {
            idealHeading -= 360;
        }

        double headingError = idealHeading - currentHeading;
        if (Math.abs(headingError + 360) < Math.abs(headingError)) {
            headingError += 360;
        } else if (Math.abs(headingError - 360) < Math.abs(headingError)) {
            headingError -= 360;
        }

        while ((headingError > 3) && (opModeIsActive())) {
            telemetry.addData("Error",headingError);
            telemetry.update();


            currentHeading = imu.getRobotYawPitchRollAngles().getYaw();

            headingError = idealHeading - currentHeading;
            if (Math.abs(headingError + 360) < Math.abs(headingError)) {
                headingError += 360;
            } else if (Math.abs(headingError - 360) < Math.abs(headingError)) {
                headingError -= 360;
            }

            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
        }

        stopMotors();
    }


    public void move(double axial, double time) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
        double previousHeading = currentHeading;
        double headingDiff = 0.0;
        double target = currentHeading;
        double headingError = 0.0;

        double startTime = getRuntime();
        double yaw = 0;


        while ((getRuntime() < (startTime + time)) && (opModeIsActive())) {
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
            headingDiff = currentHeading - previousHeading;
            if (Math.abs(headingDiff + 360) < Math.abs(headingDiff)) {
                headingDiff += 360;
            } else if (Math.abs(headingDiff - 360) < Math.abs(headingDiff)) {
                headingDiff -= 360;
            }
            headingError = target - currentHeading;
            if (Math.abs(headingError + 360) < Math.abs(headingError)) {
                headingError += 360;
            } else if (Math.abs(headingError - 360) < Math.abs(headingError)) {
                headingError -= 360;
            }

            // negative around whole thing because positive yaw makes robot turn right while heading is measured counterclockwise
            yaw = -(headingError * 0.03 - headingDiff * 0.02);

            backLeft.setPower(axial + yaw);
            backRight.setPower(axial - yaw);
            frontLeft.setPower(axial + yaw);
            frontRight.setPower(axial - yaw);

            previousHeading = currentHeading;
        }

        stopMotors();
    }

    public void moveStraight(double axial, double time) {
        double currentHeading = 0;
        double previousHeading = currentHeading;
        double headingDiff = 0.0;
        double target = currentHeading;
        double headingError = 0.0;

        double startTime = getRuntime();
        double yaw = 0;


        while ((getRuntime() < (startTime + time)) && (opModeIsActive())) {
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
            headingDiff = currentHeading - previousHeading;
            if (Math.abs(headingDiff + 360) < Math.abs(headingDiff)) {
                headingDiff += 360;
            } else if (Math.abs(headingDiff - 360) < Math.abs(headingDiff)) {
                headingDiff -= 360;
            }
            headingError = target - currentHeading;
            if (Math.abs(headingError + 360) < Math.abs(headingError)) {
                headingError += 360;
            } else if (Math.abs(headingError - 360) < Math.abs(headingError)) {
                headingError -= 360;
            }

            // negative around whole thing because positive yaw makes robot turn right while heading is measured counterclockwise
            yaw = -(headingError * 0.03 - headingDiff * 0.02);

            backLeft.setPower(axial + yaw);
            backRight.setPower(axial - yaw);
            frontLeft.setPower(axial + yaw);
            frontRight.setPower(axial - yaw);

            previousHeading = currentHeading;
        }

        stopMotors();
    }
}