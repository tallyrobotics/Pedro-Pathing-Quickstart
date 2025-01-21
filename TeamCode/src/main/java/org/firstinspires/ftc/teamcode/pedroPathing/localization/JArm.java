package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JArm implements Runnable {
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    private Telemetry telemetryAll;

    private DcMotor arm;
    private DcMotor armOther;
    private AnalogInput pot;

    private double targetPot = 0;

    final double p = 0.0030;
    final double i = 0.0005;
    final double d = 0.0020;

    static double specimenPlace = 0;
    static double betweenChamber = 0;
    static double specimenGrab = 0;
    static double basketPlace = 0;
    static double groundGrab = 0;

    public JArm(LinearOpMode initOpMode, HardwareMap initHardwareMap) {
        opMode = initOpMode;
        hardwareMap = initHardwareMap;

        arm = hardwareMap.get(DcMotor.class, "arm");
        armOther = hardwareMap.get(DcMotor.class, "armOther");
        pot = hardwareMap.get(AnalogInput.class, "pot");

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armOther.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetryAll = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryAll.addData("Status", "Initialized");
        telemetryAll.update();
    }

    public void setTarget(double newPot) {
        targetPot = newPot;
    }

    public double getTarget() {
        return targetPot;
    }

    public void run() {
        double currentPot = pot.getVoltage();
        double error = targetPot - currentPot;
        double prevError = error;
        double errorDiff = 0;
        double errorSum = 0;
        while (opMode.opModeIsActive()) {
            errorSum += error;
            prevError = error;

            currentPot = pot.getVoltage();
            error = targetPot - currentPot;
            errorDiff = error - prevError;


            double power = error * p + errorSum * i + errorDiff * d;
            arm.setPower(power);
            armOther.setPower(power);

            telemetryAll.addData("error", error);
            telemetryAll.addData("pot", currentPot);
        }
    }
}
