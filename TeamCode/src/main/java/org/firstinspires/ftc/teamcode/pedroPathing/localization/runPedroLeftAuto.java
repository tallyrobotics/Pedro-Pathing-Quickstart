package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;

/**
 * This is the CurvedBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector. Remember to test your tunings on StraightBackAndForth as well, since tunings
 * that work well for curves might have issues going in straight lines.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous (name = "runPedroLeftAuto", group = "Autonomous Pathing")
public class runPedroLeftAuto extends OpMode
{
    private Servo claw;
    
    private Follower follower;
    private Pose startPose = new Pose(9, 84, 0);
    private GeneratedPathPedroLeftAuto gppla;
    private int pathNumber = -1;
    private int actionNumber = -1;
    private boolean tryingToFollow = false;
    private String[] telemetryMessages;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init()
    {
        claw = hardwareMap.get(Servo.class, "claw");
        
        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.8);
        follower.setStartingPose(startPose);
        gppla = new GeneratedPathPedroLeftAuto();
        pathNumber = 0;
        actionNumber = 1;
        follower.followPath(gppla.specimenPlacement);

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
    @Override
    public void loop() {
        telemetry.addLine(telemetryMessages[pathNumber]);
        telemetry.update();

        follower.update();
        if (!follower.isBusy() && !tryingToFollow) {
            pathNumber++;
            actionNumber = 0;
            tryingToFollow = true;
        }

        switch (pathNumber) {
            case 0: {
                break;
            }
            case 1: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1){
                    follower.followPath(gppla.firstSampleGrab);
                } else {
                    tryingToFollow = false;
                }

                actionNumber++;
                break;
            }
            case 2: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1){
                    follower.followPath(gppla.firstSampleRelease);
                } else {
                    tryingToFollow = false;
                }

                actionNumber++;
                break;
            }
            case 3: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1){
                    follower.followPath(gppla.secondSampleGrab);
                } else {
                    tryingToFollow = false;
                }

                actionNumber++;
                break;
            }
            case 4: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1){
                    follower.followPath(gppla.secondSampleRelease);
                } else {
                    tryingToFollow = false;
                }

                actionNumber++;
                break;
            }
            case 5: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1){
                    follower.followPath(gppla.thirdSampleGrab);
                } else {
                    tryingToFollow = false;
                }

                actionNumber++;
                break;
            }
            case 6: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1){
                    follower.followPath(gppla.thirdSampleRelease);
                } else {
                    tryingToFollow = false;
                }

                actionNumber++;
                break;
            }
            case 7: {
                if (actionNumber == 0) {
                    claw.setPosition(0);
                } else if (actionNumber == 1){
                    follower.followPath(gppla.park);
                } else {
                    tryingToFollow = false;
                }

                actionNumber++;
                break;
            }

            default: {
                requestOpModeStop();
                break;
            }
        }
    }
}
