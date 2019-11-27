package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "TTAutoStoneUnmovedRed", group = "TT")
@Disabled
public class TTAutoStoneUnmovedRed extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,

    LINE_UP_STONE,
    PICK_UP_STONE,

    GO_TO_BASE_PLATE,
    PLACE_STONE,
    MOVE_BASE_PLATE,
    GO_TO_LINE,

    STOP
  }

  private enum SkyStonePos {
    UNKNOWN,
    OneAndFour,
    TwoAndFive,
    ThreeAndSix
  }

  private AutoState currentState = AutoState.INITIALIZE;
  private SkyStonePos skystonepos = SkyStonePos.UNKNOWN;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private TTRobot robot;

  @Override
  public void runOpMode() {
    double x = 0.0, y = 0.0, z = 0.0;
    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    robot = new TTRobot(hardwareMap, telemetry);

    telemetry.addData(">", "Robot Heading = %d", robot.gyroHeading());
    telemetry.update();

    // Put vuforia Here

    waitForStart();
    /*
    if (tfod != null) {
        tfod.deactivate();
    }
    */
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      telemetry.addData("Status", "Run Time: " + runtime.toString());

      switch (currentState) {
        case INITIALIZE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          /*
          if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
              tfod.activate();
          }
          */
          currentState = AutoState.LINE_UP_STONE;
          break;
        case LINE_UP_STONE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          // deciding which position to go to depending on where the stone is
          if (skystonepos.equals(SkyStonePos.OneAndFour)) {
            robot.timeDrive(x, y, z);
            // gyroHold(x, y, z);
          }
          if (skystonepos.equals(SkyStonePos.TwoAndFive)) {
            robot.timeDrive(x, y, z);
            // gyroHold(x, y, z);
          }
          if (skystonepos.equals(SkyStonePos.ThreeAndSix)) {
            robot.timeDrive(x, y, z);
            // gyroHold(x, y, z);
          }
          break;
        case PICK_UP_STONE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          // positioning the attachments to collect the stone
          /*
          robot.slide.setPower(x);
          robot.liftMotor.setPower(x);
          robot.claw.setPosition(x);
          robot.liftMotor.setPower(x);
          */
          break;
        case GO_TO_BASE_PLATE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          // driving to the baseplate
          robot.timeDrive(x, y, z);
          // distDriveRear(x, y, z);
          robot.timeDrive(x, y, z);
          break;
        case PLACE_STONE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          /*
          robot.liftMotor.setPower(x);
          robot.claw.setPosition(x);
          robot.liftMotor.setPower(x);
          robot.timeDrive(x, y, z);
          robot.liftMotor.setPower(x);
          robot.claw.setPosition(x);
          robot.timeDrive(x, y, z);
          robot.claw.setPosition(x);
          robot.liftMotor.setPower(x);

          robot.lLiftMotor(Up);
          robot.rLiftMotor(Up);
          robot.grabberClutch(Close);
          robot.lLiftMotor(Down);
          robot.rLiftMotor(Down);*/
          break;
        case MOVE_BASE_PLATE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          // robot.BPGrabber.setPosition(x);
          robot.timeDrive(x, y, z);
          // robot.BPGrabber.setPosition(x);
          break;

        case GO_TO_LINE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();

          break;
        case STOP:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
      }

      telemetry.update();
    }
  }
}
