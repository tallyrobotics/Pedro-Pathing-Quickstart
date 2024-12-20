package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class JTracking {
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor arm;
    private Servo claw;

    private Telemetry telemetryA;
    private SparkFunOTOS otos;

    // tune these values to the point where moveFieldCentric(1, 1, 0, 0.2, 0) moves it exactly diagonally to the top-right
    final double forwardFactor = 1.0;
    final double strafeFactor = 1.0;
    final double headingErrorTolerance = 1.0;

    final double minErrorTolerance = 0.05;
    final double reductionErrorTolerance = 12.0;
    final int reductionCycles = 5;
    final double stoppingPower = 0.1;

    final double maxYawPower = 0.4;

    public JTracking(LinearOpMode initOpMode, HardwareMap initHardwareMap) {
        opMode = initOpMode;
        hardwareMap = initHardwareMap;


        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        otos.setOffset(new SparkFunOTOS.Pose2D(0.075, 3.40625, 0));
        otos.setLinearScalar(1.0739); //0.9637
        otos.setAngularScalar(0.9798);

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.resetTracking();

        telemetryA = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.update();
    }

    // Coordinate system: (0,0) is the center of the 4 central tiles, 1 unit is 1 inch
    // angle of 0 is pointing in +X direction, angles increase COUNTERCLOCKWISE

    public void setMotors(double bl, double br, double fl, double fr) {
        backLeft.setPower(bl);
        backRight.setPower(br);
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
    }

    public void stopMotors() {
        setMotors(0,0,0,0);
    }

    public void setMotorsOmni(double axial, double lateral, double yaw) {
        // again, a positive yaw makes the robot rotate COUNTERCLOCKWISE
        setMotors(axial - lateral - yaw, axial + lateral + yaw, axial + lateral - yaw, axial - lateral + yaw);
    }

    public void moveFieldCentric(double dx, double dy, double heading, double power, double yaw) {
        // strafing often requires more power than going forward, so tweaking the forward and strafe factors may be necessary.
        double scaledX = dx * forwardFactor;
        double scaledY = dy * strafeFactor;

        double d = Math.sqrt(scaledX*scaledX + scaledY*scaledY);

        // if you think about what should happen when we have a heading of 0, this should make sense.
        double normalizedX = scaledX / d;
        double normalizedY = scaledY / d;

        // we initialize
        double newLateral = normalizedX * Math.sin(heading / 180 * Math.PI) - normalizedY * Math.cos(heading / 180 * Math.PI);
        double newAxial = normalizedX * Math.cos(heading / 180 * Math.PI) + normalizedY * Math.sin(heading / 180 * Math.PI);

        newAxial *= power;
        newLateral *= power;

        setMotorsOmni(newAxial, newLateral, yaw);
    }

    public double getError(double targetX, double targetY, double currentX, double currentY) {
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;

        return Math.sqrt(errorX*errorX + errorY*errorY);
    }

    public double minimizeAngle(double angle) {
        return ((angle + 180.0) % 360.0) - 180.0;
    }

    public void moveTo(double targetX, double targetY, double targetHeading, double power) {
        SparkFunOTOS.Pose2D pos = otos.getPosition();

        double powerReductionFactor = Math.pow(stoppingPower / power, 1.0 / reductionCycles);
        double toleranceReductionFactor = Math.pow(minErrorTolerance / reductionErrorTolerance, 1.0 / reductionCycles);

        double changingTolerance = reductionErrorTolerance;
        double changingPower = power;


        double yaw = 0;
        double errorX = targetX - pos.x;
        double errorY = targetY - pos.y;
        double posError = Math.sqrt(errorX*errorX + errorY*errorY);
        double headingError = minimizeAngle(targetHeading - pos.h);

        while (opMode.opModeIsActive() && ((posError > minErrorTolerance) || (Math.abs(headingError) > headingErrorTolerance))) {
            // recalculate position and error
            pos = otos.getPosition();

            errorX = targetX - pos.x;
            errorY = targetY - pos.y;

            posError = Math.sqrt(errorX*errorX + errorY*errorY);
            headingError = minimizeAngle(targetHeading - pos.h);

            telemetryA.addData("current tolerance",  changingTolerance);
            telemetryA.addData("min tolerance", minErrorTolerance);
            telemetryA.addData("reduction factor", powerReductionFactor);
            telemetryA.addData("power", changingPower);
            telemetryA.addData("heading", pos.h);
            telemetryA.update();

            // potential pid controller may help
            // remember: positive yaw means rotation COUNTERCLOCKWISE
            yaw = 0.02 * headingError;
            yaw = Math.min(Math.max(-maxYawPower, yaw), maxYawPower);

            if (posError < changingTolerance) {
                changingTolerance *= toleranceReductionFactor;
                changingPower *= powerReductionFactor;
            } else if (posError * toleranceReductionFactor > changingTolerance) {
                changingTolerance /= toleranceReductionFactor;
                changingPower /= powerReductionFactor;
            }

            // using error x and y, we know the exact direction we need to travel in the x and y directions
            moveFieldCentric(errorX, errorY, pos.h, changingPower, yaw);
        }

        stopMotors();
    }
}