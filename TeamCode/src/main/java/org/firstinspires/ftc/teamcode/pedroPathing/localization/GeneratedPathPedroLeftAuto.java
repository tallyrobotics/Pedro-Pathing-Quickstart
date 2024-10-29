package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class GeneratedPathPedroLeftAuto {
   public PathChain pc = new PathChain();
   //PathBuilder builder = new PathBuilder();
  public GeneratedPathPedroLeftAuto() {
    PathBuilder builder = new PathBuilder();

    pc = builder
            .addPath(
                    // Line 1
                    new BezierCurve(
                            new Point(9.000, 84.000, Point.CARTESIAN),
                            new Point(24.000, 76.000, Point.CARTESIAN),
                            new Point(12.000, 76.000, Point.CARTESIAN),
                            new Point(36.000, 76.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(
                    // Line 2
                    new BezierCurve(
                            new Point(36.000, 76.000, Point.CARTESIAN),
                            new Point(24.000, 76.000, Point.CARTESIAN),
                            new Point(24.000, 96.000, Point.CARTESIAN),
                            new Point(36.000, 100.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
            .addPath(
                    // Line 3
                    new BezierLine(
                            new Point(36.000, 100.000, Point.CARTESIAN),
                            new Point(16.000, 128.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(135))
            .addPath(
                    // Line 4
                    new BezierLine(
                            new Point(16.000, 128.000, Point.CARTESIAN),
                            new Point(36.000, 110.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(60))
            .addPath(
                    // Line 5
                    new BezierLine(
                            new Point(36.000, 110.000, Point.CARTESIAN),
                            new Point(16.000, 128.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(135))
            .addPath(
                    // Line 6
                    new BezierLine(
                            new Point(16.000, 128.000, Point.CARTESIAN),
                            new Point(36.000, 120.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(60))
            .addPath(
                    // Line 7
                    new BezierLine(
                            new Point(36.000, 120.000, Point.CARTESIAN),
                            new Point(16.000, 128.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(135))
            .addPath(
                    // Line 8
                    new BezierCurve(
                            new Point(16.000, 128.000, Point.CARTESIAN),
                            new Point(63.000, 140.000, Point.CARTESIAN),
                            new Point(63.000, 130.000, Point.CARTESIAN),
                            new Point(63.000, 100.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))
            .build();
  }
}