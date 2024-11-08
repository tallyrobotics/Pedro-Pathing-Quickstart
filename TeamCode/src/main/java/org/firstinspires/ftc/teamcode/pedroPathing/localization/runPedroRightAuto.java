package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

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
@Autonomous (name = "runPedroRightAuto", group = "Autonomous Pathing")
public class runPedroRightAuto extends OpMode
{
    private Follower follower;

    private Pose startPose = new Pose(9, 60, 0);

    private GeneratedPathPedroRightAuto gppra;

    private int pathNumber = -1;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init()
    {
        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.8);

        follower.setStartingPose(startPose);

        gppra = new GeneratedPathPedroRightAuto();

        pathNumber = 0;
        follower.followPath(gppra.specimenPlacement);

    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop()
    {
        follower.update();
        if (!follower.isBusy()) {
            pathNumber++;

            switch (pathNumber) {
                case 1: {
                    telemetry.addLine("specimen completed.");
                    telemetry.update();

                    follower.followPath(gppra.firstSampleGrab);
                    break;
                }
                case 2: {
                    telemetry.addLine("first sample grab");
                    telemetry.update();

                    follower.followPath(gppra.firstSampleRelease);
                    break;
                }
                case 3: {
                    telemetry.addLine("first sample release");
                    telemetry.update();

                    follower.followPath(gppra.secondSampleGrab);
                    break;
                }
                case 4: {
                    telemetry.addLine("second sample grab");
                    telemetry.update();

                    follower.followPath(gppra.secondSampleRelease);
                    break;
                }
                case 5: {
                    telemetry.addLine("second sample release");
                    telemetry.update();

                    follower.followPath(gppra.thirdSampleGrab);
                    break;
                }
                case 6: {
                    telemetry.addLine("third sample grab");
                    telemetry.update();

                    follower.followPath(gppra.thirdSampleRelease);
                    break;
                }
                case 7: {
                    telemetry.addLine("third sample release");
                    telemetry.update();

                    follower.followPath(gppra.park);
                    break;
                }

                default: {
                    requestOpModeStop();
                    break;
                }
            }

        }
    }
}
