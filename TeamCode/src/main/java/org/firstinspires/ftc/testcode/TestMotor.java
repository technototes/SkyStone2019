package org.firstinspires.ftc.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in
 * either the autonomous or the teleop period of an FTC match. The names of OpModes appear on the
 * menu of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * <p>This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot It
 * includes all the skeletal structure that all linear OpModes contain.
 *
 * <p>Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new
 * name. Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode
 * list
 */
@TeleOp(name = "Basic: Motor Test", group = "Test")
public class TestMotor extends LinearOpMode {

  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor mother = null;
  private DcMotor daughter = null;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    mother = hardwareMap.get(DcMotor.class, "motorLiftLeft");
    daughter = hardwareMap.get(DcMotor.class, "motorLiftRight");
    mother.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    daughter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mother.setDirection(DcMotorSimple.Direction.FORWARD);
    daughter.setDirection(DcMotorSimple.Direction.REVERSE);

    int mPosition = mother.getCurrentPosition();
    int dPosition = daughter.getCurrentPosition();
    telemetry.addData("mPos, dPos", "%d, %d", mPosition, dPosition);
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    double power = -.25, inc = .05;
    double last = runtime.milliseconds();
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      // Every 250 ms, update the power rating
      double stick = gamepad1.left_stick_y;
      if (Math.abs(stick) > 0.1) {
        mother.setPower(stick / 4);
        daughter.setPower(stick / 4);
      } else {
        mother.setPower(0);
        daughter.setPower(0);
      }
      mPosition = mother.getCurrentPosition();
      dPosition = daughter.getCurrentPosition();
      telemetry.addData("mPos, dPos", "%d, %d", mPosition, dPosition);
      telemetry.update();
    }
  }
}
