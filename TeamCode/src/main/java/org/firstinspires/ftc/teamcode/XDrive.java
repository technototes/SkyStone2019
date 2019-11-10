package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

// This is a self-contained implementation of XDrive
// It supports 'turbo' and 'snail' mode.
// It also enables you to just set motors if you really want, but why would you want to do that?
public class XDrive {

  // The speed scaling factor for running in snail mode
  private static final double SNAIL_MODE_SCALE = 0.5;

  // The combined of ROT_STEPS and DRIVE_STEPS should be <= 1.0
  // The final value is 'max' and should probably be the same as the 'not quite max' value

  // The values in range to use for rotation driving
  private static double[] ROT_STEPS = {0.0, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.35};
  // The values in range to use for drive speeds (normal mode)
  private static double[] DRIVE_STEPS = {0.0, 0.12, 0.24, 0.36, 0.48, 0.60, 0.60};

  public enum DriveSpeed {
    Normal, Snail, Turbo
  }

  // The 4 motors for the drive train
  private DcMotor flMotor;
  private DcMotor frMotor;
  private DcMotor rlMotor;
  private DcMotor rrMotor;

  // The current drive speed of the system
  private DriveSpeed speed = DriveSpeed.Normal;

  /**
   * @param fl Front Left motor
   * @param fr Front Right motor
   * @param rl Rear Left motor
   * @param rr Rear Right motor
   */
  public XDrive(DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr) {
    flMotor = fl;
    frMotor = fr;
    rlMotor = rl;
    rrMotor = rr;
  }

  /**
   * Set the speed mode for the drive train: Turbo, Normal, or Snail
   *
   * @param ds The speed
   */
  public void setSpeed(DriveSpeed ds) {
    speed = ds;
  }

  /**
   * @return The DriveSpeed currently set for the drive train
   */
  public DriveSpeed getSpeed() {
    return speed;
  }

  /**
   * Calculate the value to scale the motor powers according to the current speed configured
   *
   * @param fl front left power
   * @param fr front right power
   * @param rl rear left power
   * @param rr rear right power
   * @return The scaling factor
   */
  private double getScale(double fl, double fr, double rl, double rr) {
    return getSpeedScale(speed, fl, fr, rl, rr);
  }

  /**
   * Calculate the value to scale the motor powers according to the speed provided
   *
   * @param ds The drive speed
   * @param fl front left power
   * @param fr front right power
   * @param rl rear left power
   * @param rr rear right power
   * @return The scaling factor
   */
  private static double getSpeedScale(DriveSpeed ds, double fl, double fr, double rl, double rr) {
    switch (ds) {
      case Snail:
        return XDrive.SNAIL_MODE_SCALE;
      case Turbo:
        // Get the magnitude of the powers for the motors
        double afl = Math.abs(fl);
        double afr = Math.abs(fr);
        double arl = Math.abs(rl);
        double arr = Math.abs(rr);
        double scale = Math.max(Math.max(afl, afr), Math.max(arl, arr));
        // Calculate the value to scale by such that the fastest motor is at either 1 or -1
        return (scale < 1e-4) ? 1.0 : (1.0 / scale);
      case Normal:
      default:
        return 1.0;
    }
  }

  /**
   * This overrides the current mode. Perhaps useful in autonomous operation
   *
   * @param scale The value by which to scale the power
   * @param fl    Front Left Power
   * @param fr    Front Right Power
   * @param rl    Rear Left Power
   * @param rr    Rear Right Power
   */
  private void setPowerScaled(double scale, double fl, double fr, double rl, double rr) {
    flMotor.setPower(Range.clip(fl * scale, -1, 1));
    frMotor.setPower(Range.clip(fl * scale, -1, 1));
    rlMotor.setPower(Range.clip(fl * scale, -1, 1));
    rrMotor.setPower(Range.clip(fl * scale, -1, 1));
  }

  /**
   * Set the power to the 4 different motors
   * This is a helper, because it scales the speed according to the speed mode currently set
   *
   * @param fl Front Left Power
   * @param fr Front Right Power
   * @param rl Rear Left Power
   * @param rr Rear Right Power
   */
  private void setPower(double fl, double fr, double rl, double rr) {
    double scale = getScale(fl, fr, rl, rr);
    setPowerScaled(scale, fl, fr, rl, rr);
  }

  /**
   * Get a value from an array of values based on an analog input
   *
   * @param steps: The array (of any length) of values to map
   * @param val:   The analog value (from -1.0 to 1.0) to map
   * @return The value snapped to the steps
   */
  private static double getSteppedValue(double[] steps, double val) {
    // get the corresponding index for the scaleInput array.
    int index = Math.abs((int) (Range.clip(val, -1.0, 1.0) * steps.length));

    // get value from the array.
    if (val < 0) {
      return -ROT_STEPS[index];
    } else {
      return ROT_STEPS[index];
    }
  }

  /**
   * Move the robot in the direction specified, relative to the angle provided.
   * For "absolute" driving, just leave gyroAngle at zero
   *
   * @param x:         The X direction to move (-1 to 1)
   * @param y:         The Y direction to move (-1 to 1)
   * @param rotation:  The rotation to apply (-1 to 1)
   * @param gyroAngle: The heading of the robot
   */
  public void move(double x, double y, double rotation, double gyroAngle) {
    moveAtSpeed(speed, x, y, rotation, gyroAngle);
  }

  /**
   * Move the robot at the particular speed,
   * in the direction specified, relative to the angle provided.
   * For "absolute" driving, just leave gyroAngle at zero
   *
   * @param ds:        The speed to move the bot at (Normal, Turbo, or Snail)
   * @param x:         The X direction to move (-1 to 1)
   * @param y:         The Y direction to move (-1 to 1)
   * @param rotation:  The rotation to apply (-1 to 1)
   * @param gyroAngle: The heading of the robot
   */
  public void moveAtSpeed(DriveSpeed ds, double x, double y, double rotation, double gyroAngle) {
    double headingRad = 0.0;
    double powerCompY = 0.0;
    double powerCompX = 0.0;

    double flPower = 0.0;
    double frPower = 0.0;
    double rlPower = 0.0;
    double rrPower = 0.0;

    y = -getSteppedValue(DRIVE_STEPS, y);
    x = getSteppedValue(DRIVE_STEPS, x);
    rotation = -getSteppedValue(ROT_STEPS, rotation);

    headingRad = Math.toRadians(gyroAngle);
    powerCompY = (Math.cos(headingRad) * y) + (Math.sin(headingRad) * x);
    powerCompX = -(Math.sin(headingRad) * y) + (Math.cos(headingRad) * x);

    flPower = powerCompY + powerCompX + rotation;
    frPower = -powerCompY + powerCompX + rotation;
    rlPower = powerCompY - powerCompX + rotation;
    rrPower = -powerCompY - powerCompX + rotation;

    double scale = getSpeedScale(ds, flPower, frPower, rlPower, rrPower);
    setPowerScaled(scale, flPower, frPower, rlPower, rrPower);
  }

  /**
   * Stop the robot
   */
  public void stop() {
    setPowerScaled(1.0, 0, 0, 0, 0);
  }
}
