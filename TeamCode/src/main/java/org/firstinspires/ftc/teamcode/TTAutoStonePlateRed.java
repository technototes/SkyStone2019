package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "TTAutoPlateRed", group = "TT")
@Disabled
public class TTAutoStonePlateRed extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,

    GO_TO_BASE_PLATE,
    MOVE_BASE_PLATE,
    GO_TO_STONE,
    PICK_UP_STONE,
    GO_TO_BASE_PLATE2,
    PLACE_STONE,
    GO_TO_LINE,
    GO_TO_BASE_PLATE3,

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
    robot = new TTRobot(this, hardwareMap, telemetry);

    sleep(2000);
    // start calibrating the gyro.
    telemetry.addData(">", "Gyro Calibrating. Do Not move!");
    telemetry.update();
    telemetry.addData(">", "Robot Heading = %d", robot.gyroHeading());
    telemetry.update();

    // Put vuforia Here

    waitForStart();

    // Removed: Rahul thinks it's vuphoria stuff
    /*    if (tfod != null) {
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

        case GO_TO_BASE_PLATE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          // positioning the attachments to collect the stone
          // timeDriveWithButt(x, y, z);
          break;

        case GO_TO_BASE_PLATE2:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          // driving to the baseplate
          robot.timeDrive(x, y, z);
          /*
          distDriveRear(x, y, z);
          gyroHold(x, y, z);
          */
          robot.timeDrive(x, y, z);
          break;

        case MOVE_BASE_PLATE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          robot.timeDrive(x, y, z);
          robot.timeDrive(x, y, z);
          break;

        case GO_TO_STONE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          // distToLine(x, y, z);
          robot.timeDrive(x, y, z);
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

        case GO_TO_BASE_PLATE3:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          robot.timeDrive(x, y, z);
          robot.timeDrive(x, y, z);
          break;

        case PLACE_STONE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          /*
          robot.liftMotor.setPower(x);
          robot.claw.setPosition(x);
          robot.timeDrive(x, y, z);
          robot.motorLift(x);
          robot.claw.setPosition(x);
          robot.motorLift(x);
          robot.timeDrive(x, y, z);
          robot.claw.setPosition(x);
          robot.motorLift(x);
          */
          break;

        case GO_TO_LINE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          // distToLine(x, y, z);
        case STOP:
          telemetry.addData("state", currentState.toString());

          stop();
          break;

        default:
          telemetry.addData("state", currentState.toString());

          stop();
          break;
      }
      telemetry.update();
    }
  }
}
