package org.firstinspires.ftc.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Disabled
public class TestMotor extends LinearOpMode {

  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor motor = null;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    motor = hardwareMap.get(DcMotor.class, "motor");

    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery

    motor.setDirection(DcMotor.Direction.FORWARD);
    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    double power = -.25, inc = .05;
    double last = runtime.milliseconds();
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      // Every 250 ms, update the power rating
      if (runtime.milliseconds() - last > 250) {
        last = runtime.milliseconds();
        motor.setPower(power);
        power += inc;
        if (power < -.799) {
          inc = .05;
        } else if (power > .799) {
          inc = -.05;
        }
      }
      // Show the elapsed game time and wheel power.
      telemetry.addData("Power", "%.2f", power);
      telemetry.update();
    }
  }
}
