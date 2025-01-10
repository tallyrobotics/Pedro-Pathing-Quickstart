package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArmPID implements Runnable {
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    private Telemetry telemetryAll;

    private DcMotor arm;
    private AnalogInput pot;

    private double targetPot = 0;

    final double p = 0.0030;
    final double i = 0.0005;
    final double d = 0.0020;

    public ArmPID(LinearOpMode initOpMode, HardwareMap initHardwareMap) {
        opMode = initOpMode;
        hardwareMap = initHardwareMap;

        arm = hardwareMap.get(DcMotor.class, "arm");
        pot = hardwareMap.get(AnalogInput.class, "pot");

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
        double errorDiff = error - prevError;
        double errorSum = 0;
        while (opMode.opModeIsActive()) {
            prevError = error;
            currentPot = pot.getVoltage();
            error = targetPot - currentPot;
            errorDiff = error - prevError;
            errorSum += error;

            double power = error * p + errorSum * i + errorDiff * d;
            arm.setPower(power);
        }
    }
}
