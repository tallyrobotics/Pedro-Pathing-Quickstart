package org.firstinspires.ftc.teamcode.pedroPathing.localization;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config
@Autonomous (name = "runPedroRightAuto", group = "Autonomous Pathing")
public class runPedroRightAuto extends OpMode
{
    private Servo claw;
    private DcMotor wrist;
    private AnalogInput pot;
    private DcMotor arm;

    private Follower follower;
    private Pose startPose = new Pose(9, 60, 0);
    private GeneratedPathPedroRightAuto gppra;
    private int pathNumber = -1;
    private int actionNumber = -1;
    private boolean waitingForActions = true;
    private String[] telemetryMessages;

    private double targetPot = 0;
    private double prevTargetPot = 0;
    private double prevPot = 0;
    private double currentPot = 0;
    private int prevEncoderPos = 0;
    private int currentEncoderPos = 0;
    private int targetEncoderPos;
    final double ratio = 2200;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init()
    {
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        pot = hardwareMap.get(AnalogInput.class, "pot");
        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.8);
        follower.setStartingPose(startPose);
        gppra = new GeneratedPathPedroRightAuto();
        pathNumber = 0;
        actionNumber = 0;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        targetEncoderPos = (int) (targetPot * ratio);

        telemetryMessages = new String[] {
                "specimen placement",
                "cycle 1 grab",
                "cycle 1 place",
                "cycle 2 grab",
                "cycle 2 place",
                "cycle 3 grab",
                "cycle 3 place",
                "park"
        };

    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */

    public void armMovement() {
        // Put loop blocks here.
        prevPot = currentPot;
        currentPot = pot.getVoltage();
        prevEncoderPos = currentEncoderPos;
        currentEncoderPos = arm.getCurrentPosition();
        if (prevPot < 0.05) {
            targetEncoderPos += currentEncoderPos - prevEncoderPos;
        }
        arm.setTargetPosition((int) targetEncoderPos);
    }

    private void updateTargetEncoderPosition(double newPot) {
        prevTargetPot = targetPot;
        targetPot = newPot;

        double potDifference = targetPot - prevTargetPot;

        targetEncoderPos += (int) (potDifference * ratio);
    }

    @Override
    public void loop() {
        armMovement();

        if (pathNumber < 8) {
            telemetry.addLine(telemetryMessages[pathNumber]);
        }
        telemetry.addData("raw pot value", currentPot);
        telemetry.addData("offset pot value", currentPot);
        telemetry.addData("arm position", currentEncoderPos);
        telemetry.addData("targetPos", targetEncoderPos);
        follower.update();

        // if the follower has finished moving and isn't going to receive
        if (!follower.isBusy() && !waitingForActions) {
            pathNumber++;
            actionNumber = 0;
            waitingForActions = true;
        }

        switch (pathNumber) {
            case 0: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.648);
                } else if (actionNumber == 1) {
                    follower.followPath(gppra.specimenPlacement);
                } else {
                    waitingForActions = false;
                }

                actionNumber++;
                break;
            }
            case 1: {
                if (actionNumber == 0) {
                    updateTargetEncoderPosition(0.5);
                } else if (actionNumber == 1) {
                    claw.setPosition(0);
                } else if (actionNumber == 2) {
                    updateTargetEncoderPosition(0.648);
                } else if (actionNumber == 3) {
                    follower.followPath(gppra.firstSampleGrab);
                } else {
                    waitingForActions = false;
                }

                actionNumber++;
                break;
            }
            case 2: {
                if (actionNumber == 0) {
                    claw.setPosition(1);
                } else if (actionNumber == 1) {
                    follower.followPath(gppra.firstSampleRelease);
                } else {
                    waitingForActions = false;
                }

                actionNumber++;
                break;
            }
            case 3: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1) {
                    follower.followPath(gppra.secondSampleGrab);
                } else {
                    waitingForActions = false;
                }

                actionNumber++;
                break;
            }
            case 4: {
                if (actionNumber == 0) {
                    claw.setPosition(1);
                } else if (actionNumber == 1) {
                    follower.followPath(gppra.secondSampleRelease);
                } else {
                    waitingForActions = false;
                }

                actionNumber++;
                break;
            }
            case 5: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1) {
                    follower.followPath(gppra.thirdSampleGrab);
                } else {
                    waitingForActions = false;
                }

                actionNumber++;
                break;
            }
            case 6: {
                if (actionNumber == 0) {
                    claw.setPosition(1);
                } else if (actionNumber == 1){
                    follower.followPath(gppra.thirdSampleRelease);
                } else {
                    waitingForActions = false;
                }

                actionNumber++;
                break;
            }
            case 7: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1) {
                    follower.followPath(gppra.park);
                } else if (actionNumber ==2) {
                    updateTargetEncoderPosition(0);
                } else {
                    waitingForActions = false;
                }

                actionNumber++;
                break;
            }

            default: {

                requestOpModeStop();
                break;
            }
        }

        telemetry.update();
    }
}