package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "GoToStone", group = "TT")
public class TTAutoGoToStone extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,
    GOTOBLOCK1,
    GOTOBLOCK2,
    GOTOBLOCK3,
    GOFORWARD,
    STOP
  }


  private AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private TTRobot robot;
  private ElapsedTime driveTime = new ElapsedTime();


  @Override
  public void runOpMode() {
    double x = 0.0, y = 0.0, z = 0.0;

    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    robot = new TTRobot();
    robot.init(hardwareMap, telemetry);
    robot.calibrate(); // OOPS!

    sleep(2000);
    // start calibrating the gyro.
    telemetry.addData(">", "Gyro Calibrating. Do Not move!");
    //robot.calibrate();
    //telemetry.addData(">", "Robot Heading = %d", robot.gyroHeading());

    // Put vuforia Here

    waitForStart();
    /*
    if (tfod != null) {
        tfod.deactivate();
    }
    */
    // run until the end of the match (driver presses STOP)
    int blockPos = 0;
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
          blockPos = robot.getSkystonePosition();

          switch (blockPos) {
            case 1:
              currentState = AutoState.GOTOBLOCK1;
            case 2:
              currentState = AutoState.GOTOBLOCK2;

            case 3:
              currentState = AutoState.GOTOBLOCK3;
          }
          break;

        case GOTOBLOCK1:
          telemetry.addData("state", currentState.toString());
          //telemetry.addData("sensorval", robot.);

          //robot.driveToWall();
          //currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK2:
          telemetry.addData("state", currentState.toString());
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK3:
          telemetry.addData("state", currentState.toString());
          currentState = AutoState.GOFORWARD;
          break;
        case GOFORWARD:
          telemetry.addData("state", currentState.toString());
          currentState = AutoState.STOP;
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
