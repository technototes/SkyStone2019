package org.firstinspires.ftc.testcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

// IMU = Internal Motion Unit
// It's a gyro with lots of interesting data. This sample has all the stuff
// necessary to generate a heading. Need to wrap it up inside the Robot class
@Disabled
@TeleOp(name = "Sensor: BNO055 imu sample", group = "Sensor")
public class TestImuGyro extends LinearOpMode {
  // ----------------------------------------------------------------------------------------------
  // State
  // ----------------------------------------------------------------------------------------------

  // The IMU sensor object
  BNO055IMU imu1;

  // State used for updating telemetry
  Orientation angles1;
  Acceleration gravity1;

  // ----------------------------------------------------------------------------------------------
  // Main logic
  // ----------------------------------------------------------------------------------------------

  @Override
  public void runOpMode() {

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    //        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration
    // sample opmode
    parameters.calibrationDataFile =
        "AdafruitIMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
    imu1.initialize(parameters);

    // Set up our telemetry dashboard
    composeTelemetry();

    // Wait until we're told to go
    waitForStart();

    // Start the logging of measured acceleration
    imu1.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    // Loop and update the dashboard
    while (opModeIsActive()) {
      telemetry.update();
    }
  }

  // Kevin: This is just oh-so-more-complicated than it needs to be :/
  void composeTelemetry() {

    // At the beginning of each telemetry update, grab a bunch of data
    // from the IMU that we will then display in separate lines.
    telemetry.addAction(
        new Runnable() {
          @Override
          public void run() {
            // Acquiring the angles1 is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles1 =
                imu1.getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity1 = imu1.getGravity();
          }
        });

    telemetry
        .addLine()
        .addData(
            "status1",
            new Func<String>() {
              @Override
              public String value() {
                return imu1.getSystemStatus().toShortString();
              }
            })
        .addData(
            "calib1",
            new Func<String>() {
              @Override
              public String value() {
                return imu1.getCalibrationStatus().toString();
              }
            });

    telemetry
        .addLine()
        .addData(
            "heading1",
            new Func<String>() {
              @Override
              public String value() {
                return formatAngle(angles1.angleUnit, angles1.firstAngle);
              }
            })
        .addData(
            "roll1",
            new Func<String>() {
              @Override
              public String value() {
                return formatAngle(angles1.angleUnit, angles1.secondAngle);
              }
            })
        .addData(
            "pitch1",
            new Func<String>() {
              @Override
              public String value() {
                return formatAngle(angles1.angleUnit, angles1.thirdAngle);
              }
            });

    telemetry
        .addLine()
        .addData(
            "grvty1",
            new Func<String>() {
              @Override
              public String value() {
                return gravity1.toString();
              }
            })
        .addData(
            "mag1",
            new Func<String>() {
              @Override
              public String value() {
                return String.format(
                    Locale.getDefault(),
                    "%.3f",
                    Math.sqrt(
                        gravity1.xAccel * gravity1.xAccel
                            + gravity1.yAccel * gravity1.yAccel
                            + gravity1.zAccel * gravity1.zAccel));
              }
            });
  }

  // ----------------------------------------------------------------------------------------------
  // Formatting
  // ----------------------------------------------------------------------------------------------

  String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  String formatDegrees(double degrees) {
    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
  }
}
