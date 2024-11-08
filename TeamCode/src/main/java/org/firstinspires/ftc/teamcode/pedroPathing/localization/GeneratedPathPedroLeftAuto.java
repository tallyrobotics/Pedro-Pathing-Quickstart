package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import android.graphics.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class GeneratedPathPedroLeftAuto {
   public PathChain specimenPlacement;
   public PathChain firstSampleGrab, firstSampleRelease;
   public PathChain secondSampleGrab, secondSampleRelease;
   public PathChain thirdSampleGrab, thirdSampleRelease;
   public PathChain park;

   //PathBuilder builder = new PathBuilder();
  public GeneratedPathPedroLeftAuto() {
    specimenPlacement = new PathBuilder()
                    // Line 1
            .addPath(
                    new BezierCurve(
                            new Point(9.000, 84.000, Point.CARTESIAN),
                            new Point(24.000, 76.000, Point.CARTESIAN),
                            new Point(12.000, 76.000, Point.CARTESIAN),
                            new Point(36.000, 76.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    firstSampleGrab = new PathBuilder()
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
            .build();

    firstSampleRelease = new PathBuilder()
            .addPath(
                    // Line 3
                    new BezierLine(
                            new Point(36.000, 100.000, Point.CARTESIAN),
                            new Point(30.000, 114.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(135))
            .build();

    secondSampleGrab = new PathBuilder()
            .addPath(
                    // Line 4
                    new BezierLine(
                            new Point(30.000, 114.000, Point.CARTESIAN),
                            new Point(36.000, 110.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(60))
            .build();

    secondSampleRelease = new PathBuilder()
            .addPath(
                    // Line 5
                    new BezierLine(
                            new Point(36.000, 110.000, Point.CARTESIAN),
                            new Point(30.000, 114.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(135))
            .build();

    thirdSampleGrab = new PathBuilder()
            .addPath(
                    // Line 6
                    new BezierLine(
                            new Point(30.000, 114.000, Point.CARTESIAN),
                            new Point(36.000, 120.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(60))
            .build();

    thirdSampleRelease = new PathBuilder()
            .addPath(
                    // Line 7
                    new BezierLine(
                            new Point(36.000, 120.000, Point.CARTESIAN),
                            new Point(30.000, 114.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(135))
            .build();

    park = new PathBuilder()
            .addPath(
                    // Line 8
                    new BezierCurve(
                            new Point(30.000, 114.000, Point.CARTESIAN),
                            new Point(61.000, 140.000, Point.CARTESIAN),
                            new Point(61.000, 130.000, Point.CARTESIAN),
                            new Point(61.000, 100.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))
            .build();
  }
}