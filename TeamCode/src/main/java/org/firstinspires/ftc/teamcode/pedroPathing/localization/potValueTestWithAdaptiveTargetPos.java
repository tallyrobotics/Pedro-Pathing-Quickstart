package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "potValueTestWithAdaptiveTargetPos (Blocks to Java)")
public class potValueTestWithAdaptiveTargetPos extends LinearOpMode {

  private DcMotor arm;
  private AnalogInput pot;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    double targetPot;
    int ratio;
    int initialPot;
    double targetEncoderPos;
    double prevPot;
    double currentPot = 0;
    double prevEncoderPos;
    int currentEncoderPos = 0;

    arm = hardwareMap.get(DcMotor.class, "arm");
    pot = hardwareMap.get(AnalogInput.class, "pot");

    // Put initialization blocks here.
    targetPot = 0.648;
    ratio = 2200;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      arm.setTargetPosition(0);
      arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      arm.setPower(0.4);
      initialPot = 0;
      targetEncoderPos = (targetPot - initialPot) * ratio;
      while (opModeIsActive()) {
        // Put loop blocks here.
        prevPot = currentPot;
        currentPot = pot.getVoltage();
        prevEncoderPos = currentEncoderPos;
        currentEncoderPos = arm.getCurrentPosition();
        if (prevPot - initialPot < 0.05) {
          targetEncoderPos += currentEncoderPos - prevEncoderPos;
        }
        arm.setTargetPosition((int) targetEncoderPos);
        telemetry.addData("raw pot value", currentPot);
        telemetry.addData("offset pot value", currentPot - initialPot);
        telemetry.addData("arm position", currentEncoderPos);
        telemetry.addData("targetPos", targetEncoderPos);
        telemetry.update();
      }
    }
  }
}