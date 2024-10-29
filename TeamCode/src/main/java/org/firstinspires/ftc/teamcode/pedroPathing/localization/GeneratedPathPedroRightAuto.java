package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class GeneratedPathPedroRightAuto {
   public PathChain pc = new PathChain();
   //PathBuilder builder = new PathBuilder();
  public GeneratedPathPedroRightAuto() {
    PathBuilder builder = new PathBuilder();

    pc = builder
            .addPath(
                    // Line 1
                    new BezierCurve(
                            new Point(9.000, 60.000, Point.CARTESIAN),
                            new Point(24.000, 68.000, Point.CARTESIAN),
                            new Point(12.000, 68.000, Point.CARTESIAN),
                            new Point(36.000, 68.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(
                    // Line 2
                    new BezierCurve(
                            new Point(36.000, 68.000, Point.CARTESIAN),
                            new Point(24.000, 68.000, Point.CARTESIAN),
                            new Point(24.000, 48.000, Point.CARTESIAN),
                            new Point(36.000, 44.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-60))
            .addPath(
                    // Line 3
                    new BezierLine(
                            new Point(36.000, 44.000, Point.CARTESIAN),
                            new Point(30.000, 30.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(-135))
            .addPath(
                    // Line 4
                    new BezierLine(
                            new Point(30.000, 30.000, Point.CARTESIAN),
                            new Point(36.000, 34.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-60))
            .addPath(
                    // Line 5
                    new BezierLine(
                            new Point(36.000, 34.000, Point.CARTESIAN),
                            new Point(30.000, 30.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(-135))
            .addPath(
                    // Line 6
                    new BezierLine(
                            new Point(30.000, 30.000, Point.CARTESIAN),
                            new Point(36.000, 24.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-60))
            .addPath(
                    // Line 7
                    new BezierLine(
                            new Point(36.000, 24.000, Point.CARTESIAN),
                            new Point(30.000, 30.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(-135))
            .addPath(
                    // Line 8
                    new BezierCurve(
                            new Point(30.000, 30.000, Point.CARTESIAN),
                            new Point(63.000, 4.000, Point.CARTESIAN),
                            new Point(63.000, 14.000, Point.CARTESIAN),
                            new Point(63.000, 44.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
            .build();
  }
}