package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "TTAutoStoneMovedWallRed", group = "TT")

public class TTAutoStoneMovedWallRed extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,

    LINE_UP_STONE,
    PICK_UP_STONE,

    GO_TO_BASE_PLATE,
    PLACE_STONE,
    GO_TO_LINE,

    STOP
  }



  private AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private TTRobot robot;
  private ElapsedTime runTime = new ElapsedTime();

  @Override
  public void runOpMode() {
    double x = 0.0, y = 0.0, z = 0.0;

    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    TTRobot robot = new TTRobot();
    robot.init(hardwareMap, telemetry);

    sleep(2000);
    // start calibrating the gyro.
    telemetry.addData(">", "Gyro Calibrating. Do Not move!");
    telemetry.update();
    robot.calibrate();
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
          //robot.getSkystonePosition();
          currentState = AutoState.GO_TO_BASE_PLATE;
          break;
        case LINE_UP_STONE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();

          //robot.goToStone();

          currentState = AutoState.PICK_UP_STONE;
          break;
        case PICK_UP_STONE:


          telemetry.addData("state", currentState.toString());
          runtime.reset();
//          robot.grabStone();
          break;
        case GO_TO_BASE_PLATE:


          telemetry.addData("state", currentState.toString());
          runtime.reset();

          robot.timeDrive(1.0, 2.0, 180);
          robot.distRearDrive(1.0, 3.0, 270);
          robot.timeDrive(1.0, 2.0, 0);
          robot.syncTurn(270);

          break;
        case PLACE_STONE:


          telemetry.addData("state", currentState.toString());
          runTime.reset();
          while (runTime.seconds() < 2) {
            robot.liftUp();
          }
          runTime.reset();
          while (runTime.seconds() < 2) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }

          runTime.reset();
          while (runTime.seconds() < 2) {
            robot.liftDown();
          }

          robot.claw(0);

          break;

        case GO_TO_LINE:


          telemetry.addData("state", currentState.toString());
          runtime.reset();
          robot.driveToLine(1.0, 90);


          break;
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
