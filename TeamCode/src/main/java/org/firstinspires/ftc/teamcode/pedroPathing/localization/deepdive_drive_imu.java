// 6.65625 is center
// 3.25

package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "deepdive_drive_imu")
public class deepdive_drive_imu extends LinearOpMode {

  private DcMotor frontLeft;
  private DcMotor backLeft;
  private DcMotor frontRight;
  private DcMotor backRight;
  private SparkFunOTOS sensor_otos;
  private DcMotor arm;
  private DcMotor wrist;
  private Servo claw;
  private IMU imu;
  private CRServo intake;
  private AnalogInput pot;


  final double ratio = 1100;

  double leftFrontPower;
  double heading;
  ElapsedTime runtime;
  double leftBackPower;
  double rightFrontPower;
  SparkFunOTOS.Pose2D pos;
  double headingDiff;
  double rightBackPower;
  double prevHeading;
  double targetHeading;
  double headingError;
  double correctionFactor = 2;

  private int currentEncoderPos = 0;
  private int prevEncoderPos = 0;

  double potValue = 0;
  double targetPot = 0;
  double potChange = 0;
  double potIntegral = 0;
  double potError = 0;
  double prevPotError = 0;
  double totalPotCorrection = 0;

  boolean toggleFieldCentric;
  int fieldCentric;
  boolean toggleClaw;
  int clawPosition = 1;

  double armPower = 0;
  boolean armPowerApplied = true;
  int armTargetPosition = 0;
  int armTargetOffset = 0;

  double wristPower = 0;
  boolean wristPowerApplied = true;
  int wristTargetPosition = 0;



  /**
   * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
   * This code will work with either a Mecanum-Drive or an X-Drive train.
   * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
   *
   * Also note that it is critical to set the correct rotation direction for each motor. See details below.
   *
   * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
   * Each motion axis is controlled by one Joystick axis.
   *
   * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
   * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
   * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
   *
   * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
   * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
   * the direction of all 4 motors (see code below).
   */
  @Override
  public void runOpMode() {
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
//    sensor_otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
    arm = hardwareMap.get(DcMotor.class, "arm");
    wrist = hardwareMap.get(DcMotor.class, "wrist");
    claw = hardwareMap.get(Servo.class, "claw");
    pot = hardwareMap.get(AnalogInput.class, "pot");

    imu = hardwareMap.get(IMU.class, "imu");
    intake = hardwareMap.get(CRServo.class, "intake");

    claw.scaleRange(0.51, 1);

//
//    configureOTOS();
    runtime = new ElapsedTime();
    // ########################################################################################
    // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
    // ########################################################################################
    //
    // Most robots need the motors on one side to be reversed to drive forward.
    // The motor reversals shown here are for a "direct drive" robot
    // (the wheels turn the same direction as the motor shaft).
    //
    // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
    // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
    // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
    //
    // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
    // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
    // <--- Click blue icon to see important note re. testing motor directions.
    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    frontRight.setDirection(DcMotor.Direction.REVERSE);
    backRight.setDirection(DcMotor.Direction.REVERSE);

    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    runtime.reset();
    prevHeading = 0;
    heading = 0;
    targetHeading = 0;
    fieldCentric = 0;
    toggleFieldCentric = true;

    arm.setTargetPosition(0);
    wrist.setTargetPosition(0);

    imu.resetYaw();

    while (opModeIsActive()) {
//      arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      prevEncoderPos = currentEncoderPos;
//      currentEncoderPos = arm.getCurrentPosition();

      // Put loop blocks here.
//      pos = sensor_otos.getPosition();
      omniDrive();
      // Get the latest position, which includes the x and y coordinates, plus the
      // heading angle.
      // Reset the tracking if the user requests it.
      if (gamepad1.right_stick_button) {
        imu.resetYaw();
        prevHeading = 0; // I noticed that after resetting it often jittered, and it's because the heading_diff becomes large and it tries to correct for it.
        targetHeading = 0;
      }
      // Re-calibrate the IMU if the user requests it.

      // Inform user of available controls
      telemetry.addLine("");
      // Log the position to the telemetry.
//      telemetry.addData("X coordinate", JavaUtil.formatNumber(pos.x, 2));
//      telemetry.addData("Y coordinate", JavaUtil.formatNumber(pos.y, 2));
//      telemetry.addData("Heading angle", JavaUtil.formatNumber(pos.h, 2));
      // Update the telemetry on the driver station.
      telemetry.addData("Difference in Heading between frames", JavaUtil.formatNumber(headingDiff, 2));
      telemetry.addData("heading Error", JavaUtil.formatNumber(headingError, 2));
      telemetry.addData("Correction Factor", JavaUtil.formatNumber(correctionFactor, 2));
      telemetry.update();
    }
    // Run until the end of the match (driver presses STOP)
  }

  /**
   * This function is used to test your motor directions.
   *
   * Each button should make the corresponding motor run FORWARD.
   *
   *   1) First get all the motors to take to correct positions on the robot
   *      by adjusting your Robot Configuration if necessary.
   *
   *   2) Then make sure they run in the correct direction by modifying the
   *      the setDirection() calls above.
   */
  private void testMotorDirections() {
    leftFrontPower = gamepad1.x ? 1 : 0;
    leftBackPower = gamepad1.a ? 1 : 0;
    rightFrontPower = gamepad1.y ? 1 : 0;
    rightBackPower = gamepad1.b ? 1 : 0;
  }

  /**
   * Configures the SparkFun OTOS.
   */
  private void configureOTOS() {
    SparkFunOTOS.Pose2D offset;
    SparkFunOTOS.Pose2D currentPosition;
    SparkFunOTOS.Version hwVersion;
    SparkFunOTOS.Version fwVersion;

    telemetry.addLine("Configuring OTOS...");
    telemetry.update();
    // Set the desired units for linear and angular measurements. Can be either
    // meters or inches for linear, and radians or degrees for angular. If not
    // set, the default is inches and degrees. Note that this setting is not
    // stored in the sensor, so you need to set at the start of all your OpModes. :P
    sensor_otos.setLinearUnit(DistanceUnit.INCH);
    sensor_otos.setAngularUnit(AngleUnit.DEGREES);
    // Assuming you've mounted your sensor to a robot and it's not centered,
    // you can specify the offset for the sensor relative to the center of the
    // robot. The units default to inches and degrees, but if you want to use
    // different units, specify them before setting the offset! Note that as of
    // firmware version 1.0, these values will be lost after a power cycle, so
    // you will need to set them each time you power up the sensor. For example, if
    // the sensor is mounted 5 inches to the left (negative X) and 10 inches
    // forward (positive Y) of the center of the robot, and mounted 90 degrees
    // clockwise (negative rotation) from the robot's orientation, the offset
    // would be {-5, 10, -90}. These can be any value, even the angle can be
    // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
    offset = new SparkFunOTOS.Pose2D(0.47, 4.6, 0);
    sensor_otos.setOffset(offset);
    // Here we can set the linear and angular scalars, which can compensate for
    // scaling issues with the sensor measurements. Note that as of firmware
    // version 1.0, these values will be lost after a power cycle, so you will
    // need to set them each time you power up the sensor. They can be any value
    // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
    // first set both scalars to 1.0, then calibrate the angular scalar, then
    // the linear scalar. To calibrate the angular scalar, spin the robot by
    // multiple rotations (eg. 10) to get a precise error, then set the scalar
    // to the inverse of the error. Remember that the angle wraps from -180 to
    // 180 degrees, so for example, if after 10 rotations counterclockwise
    // (positive rotation), the sensor reports -15 degrees, the required scalar
    // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
    // robot a known distance and measure the error; do this multiple times at
    // multiple speeds to get an average, then set the linear scalar to the
    // inverse of the error. For example, if you move the robot 100 inches and
    // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
    sensor_otos.setLinearScalar(1);
    sensor_otos.setAngularScalar(1);
    // The IMU on the OTOS includes a gyroscope and accelerometer, which could
    // have an offset. Note that as of firmware version 1.0, the calibration
    // will be lost after a power cycle; the OTOS performs a quick calibration
    // when it powers up, but it is recommended to perform a more thorough
    // calibration at the start of all your OpModes. Note that the sensor must
    // be completely stationary and flat during calibration! When calling
    // calibrateImu, you can specify the number of samples to take and whether
    // to wait until the calibration is complete. If no parameters are provided,
    // it will take 255 samples and wait until done; each sample takes about
    // 2.4ms, so about 612ms total.
    sensor_otos.calibrateImu();
    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors.
    sensor_otos.resetTracking();
    // After resetting the tracking, the OTOS will report that the robot is at
    // the origin. If your robot does not start at the origin, or you have
    // another source of location information (eg. vision odometry), you can set
    // the OTOS location to match and it will continue to track from there.
    currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
    sensor_otos.setPosition(currentPosition);
    // Get the hardware and firmware version
    hwVersion = new SparkFunOTOS.Version();
    fwVersion = new SparkFunOTOS.Version();
    sensor_otos.getVersionInfo(hwVersion, fwVersion);
    telemetry.addLine("OTOS configured! Press start to get position data!");
    telemetry.addLine("");
    telemetry.addLine("OTOS Hardware Version: v" + hwVersion.major + "." + hwVersion.minor);
    telemetry.addLine("OTOS Firmware Version: v" + fwVersion.major + "." + fwVersion.minor);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void omniDrive() {
    double correction;
    double driveScale = 0.0;
    double armScale = 0.0;
    double wristScale = 0.0;
    double axial;
    double lateral;
    double yaw = 0.0;
    double joystickMagnitude;
    double turnTimer = 0.0;
    double newAxial;
    double newLateral;
    double max;
    int intakeValue = 0;
    double intakePower = 0;
    boolean isYPressed = false;

    potValue = pot.getVoltage();
    potError = targetPot - potValue;
    potChange = potError - prevPotError;
    prevPotError = potError;
    potIntegral += potError; // potIntegral = potIntegral + potError;

    heading = imu.getRobotYawPitchRollAngles().getYaw();
    headingDiff = heading - prevHeading;
    // fix the issue w/ angle looping from 180 to -180
    if (Math.abs(headingDiff + 360) < Math.abs(headingDiff)) {
      headingDiff += 360;
    } else if (Math.abs(headingDiff - 360) < Math.abs(headingDiff)) {
      headingDiff -= 360;
    }

    // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
    // Note: pushing stick forward gives negative value
    if (gamepad1.left_bumper) {
      driveScale = 0.33;
    } else if (gamepad1.right_bumper) {
      driveScale = 1;
    } else {
      driveScale = 0.66;
    }

    if (gamepad2.left_bumper) {
      armScale = 0.35;
      wristScale = 0.5;
    } else if (gamepad2.right_bumper) {
      armScale = 0.7;
      wristScale = 0.9;
//    } else if (isYPressed == true) {
//      armScale = 0.8;
    }
    else{
      armScale = 0.5;
      wristScale = 0.7;
    }

    // toggle logic for field centric
    if (gamepad1.left_stick_button) {
      if (toggleFieldCentric) {
        fieldCentric = 1 - fieldCentric;
        toggleFieldCentric = false;
      }
    } else {
      toggleFieldCentric = true;
    }
    if (gamepad2.a) {
      if (toggleClaw) {
        clawPosition = 1 - clawPosition;
        toggleClaw = false;
      }
    } else {
      toggleClaw = true;
    }

    if (gamepad2.y) {
      intake.setPower(1);
    } else if (gamepad2.x) {
      intake.setPower(-1);
    } else {
      intake.setPower(0);
    }

    axial = -gamepad1.left_stick_y;
    lateral = gamepad1.left_stick_x;
    yaw = gamepad1.right_stick_x;
    joystickMagnitude = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));

    axial = joystickMagnitude * axial;
    lateral = joystickMagnitude * lateral;

    axial *= driveScale;
    lateral *= driveScale;
    yaw *= driveScale;

    if (Math.abs(yaw) > 0.1) {
      turnTimer = 10;
    }
    turnTimer = Math.max(turnTimer - 1, 0);
    if (turnTimer > 0) {
      targetHeading = heading;
    }

    headingError =  targetHeading - heading;
    if (Math.abs(headingError + 360) < Math.abs(headingError)) {
      headingError += 360;
    } else if (Math.abs(headingError - 360) < Math.abs(headingError)) {
      headingError -= 360;
    }

    if (fieldCentric == 1) {
      correction = -heading - headingDiff * correctionFactor; // accounts for the robot rotating while driving
      // rotates the movement vectors in the opposite direction so they point along the field coordinates
      newAxial = lateral * Math.sin(correction / 180 * Math.PI) + axial * Math.cos(correction / 180 * Math.PI);
      newLateral = lateral * Math.cos(correction / 180 * Math.PI) - axial * Math.sin(correction / 180 * Math.PI);
      axial = newAxial;
      lateral = newLateral;
    }
//    if (turnTimer == 0) {
//      yaw = -(headingError * 0.015 - headingDiff * 0.01);
//    }
    // Combine the joystick requests for each axis-motion to determine each wheel's power.
    // Set up a variable for each drive wheel to save the power level for telemetry.
    leftFrontPower = (axial + lateral) + yaw;
    rightFrontPower = (axial - lateral) - yaw;
    leftBackPower = (axial - lateral) + yaw;
    rightBackPower = (axial + lateral) - yaw;
    // Normalize the values so no wheel power exceeds 100%
    // This ensures that the robot maintains the desired motion.
    max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
    if (max > 1) {
      leftFrontPower = leftFrontPower / max;
      rightFrontPower = rightFrontPower / max;
      leftBackPower = leftBackPower / max;
      rightBackPower = rightBackPower / max;
    }
    // Send calculated power to wheels.
    frontLeft.setPower(leftFrontPower);
    frontRight.setPower(rightFrontPower);
    backLeft.setPower(leftBackPower);
    backRight.setPower(rightBackPower);

//    arm.setPower(armPower);
//    wrist.setPower(gamepad2.right_stick_y*0.5);
//    if (arm.getCurrentPosition()>= 0) //set that 0 value to the actual limit value.
//    {
//      arm.setPower(-armPower);
//    }

    claw.setPosition(clawPosition);

    wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Put loop blocks here.

    // getting the input to check whether or not the joystick is actively being used.
    wristPower = Math.pow(gamepad2.right_stick_y, 3);
    armPower = Math.pow(gamepad2.left_stick_y, 3);

    if (wristPower == 0) {
      wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      if (gamepad2.dpad_up) {
        wristTargetPosition = 388;
      } else if (gamepad2.dpad_down) {
        wristTargetPosition = 396;
      } else if (gamepad2.dpad_left) {
        wristTargetPosition = 25;
      } else if (gamepad2.dpad_right) {
        wristTargetPosition = 25;
      } else if (wristPowerApplied) {
        wristTargetPosition = wrist.getCurrentPosition();
        wristPowerApplied = false;
      }
      wrist.setTargetPosition(wristTargetPosition);
      wristPower = 0.8;
    } else {
      wristPowerApplied = true;
      wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    if (armPower == 0) {
//      arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      arm.setTargetPosition(arm.getCurrentPosition());

      if (gamepad2.dpad_up) {
        targetPot = 0.576;
      } else if (gamepad2.dpad_down) {
        targetPot = 0.277;
      } else if (gamepad2.dpad_left) {
        targetPot = 0.648;
      } else if (gamepad2.dpad_right) {
        targetPot = 0.199;
      } else if (armPowerApplied) {
        armTargetPosition = arm.getCurrentPosition();// potValue;
        armPowerApplied = false;
      }

//      armTargetPosition = (int) (ratio * targetPot);

//      if (potChange == 0.0) {
//        armTargetOffset += currentEncoderPos - prevEncoderPos;
//      }

      arm.setTargetPosition(armTargetPosition + armTargetOffset);
      armPower = 0.6;
      arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    } else {
      armPowerApplied = true;
      arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

  if (armPower < 0 && arm.getCurrentPosition() < -5876) {
      armPower = 0;
  }

    if(gamepad1.y){
      armScale = 1;
      isYPressed = true;
    }

    wrist.setPower(wristPower * wristScale);
    arm.setPower(armPower * armScale);


    // Show the elapsed game time and wheel power.
    telemetry.addData("Status", "Run Time: " + runtime);
    telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
    telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
    telemetry.addData("targetPos", arm.getTargetPosition());
    telemetry.addData("currentPos", arm.getCurrentPosition());

    prevHeading = heading;
  }
}