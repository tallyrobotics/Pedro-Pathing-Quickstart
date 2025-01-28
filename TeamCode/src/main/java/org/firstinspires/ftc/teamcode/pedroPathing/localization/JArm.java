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

    private DcMotor arm1;
    private DcMotor arm2;
    private AnalogInput pot;

    private double targetPot = 0;

    final double p = 0.80;
    final double i = 0.10;
    final double d = 0.60;

    static double specimenPlace = 1.791;
    static double specimenGrab = 1.175;
    static double basketPlace = 2.908;
    static double groundGrab = 0;
    static double parkingHeight = 0;

    public JArm(LinearOpMode initOpMode, HardwareMap initHardwareMap) {
        opMode = initOpMode;
        hardwareMap = initHardwareMap;

        arm1 = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "armOther");
        pot = hardwareMap.get(AnalogInput.class, "pot");

        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            arm1.setPower(power);
            arm2.setPower(power);

            telemetryAll.addData("error", error);
            telemetryAll.addData("pot", currentPot);
        }
    }
}
