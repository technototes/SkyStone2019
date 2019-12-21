//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//
//@Autonomous(name = "TTAutoStoneMovedNotWallRed", group = "TT")
//@Disabled
//public class TTAutoStoneMovedNotWallRed extends LinearOpMode {
//
//  // States
//  private enum AutoState {
//
//
//
//
//
//
//
//    GO_TO_LINE,
//
//    STOP
//  }
//
//  private enum SkyStonePos {
//    UNKNOWN,
//    OneAndFour,
//    TwoAndFive,
//    ThreeAndSix
//  }
//
//  private AutoState currentState = AutoState.GO_TO_LINE;
//  private SkyStonePos skystonepos = SkyStonePos.UNKNOWN;
//  private ElapsedTime runtime = new ElapsedTime();
//  private ElapsedTime timer = new ElapsedTime();
//  private TTRobot robot;
//
//  @Override
//  public void runOpMode() {
//    double x = 0.0, y = 0.0, z = 0.0;
//
//    /*
//     * Initialize the standard drive system variables.
//     * The init() method of the hardware class does most of the work here
//     */
//    robot = new TTRobot(this, hardwareMap, telemetry);
//
//    telemetry.addData(">", "Robot Heading = %d", robot.gyroHeading());
//    telemetry.update();
//
//    // Put vuforia Here
//
//    waitForStart();
//
//    /*
//    if (tfod != null) {
//        tfod.deactivate();
//    }
//    */
//    // run until the end of the match (driver presses STOP)
//    while (opModeIsActive()) {
//      telemetry.addData("Status", "Run Time: " + runtime.toString());
//
//      switch (currentState) {
//        /*
//        \case GO_TO_LINE:
//          telemetry.addData("state", currentState.toString());
//          runtime.reset();
//
//          /*
//          if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
//              tfod.activate();
//          }
//          */
//
//        /*
//        case LINE_UP_STONE:
//          telemetry.addData("state", currentState.toString());
//          runtime.reset();
//          // deciding which position to go to depending on where the stone is
//          if (skystonepos.equals(SkyStonePos.OneAndFour)) {
//            robot.timeDrive(x, y, z);
//            // gyroHold(x, y, z);
//          }
//          if (skystonepos.equals(SkyStonePos.TwoAndFive)) {
//            robot.timeDrive(x, y, z);
//            // gyroHold(x, y, z);
//          }
//          if (skystonepos.equals(SkyStonePos.ThreeAndSix)) {
//            robot.timeDrive(x, y, z);
//            // gyroHold(x, y, z);
//          }
//          currentState = AutoState.PICK_UP_STONE;
//          break;
//        case PICK_UP_STONE:
//          telemetry.addData("state", currentState.toString());
//          runtime.reset();
//          // positioning the attachments to collect the stone
//          /*
//          robot.slide.setPower(x);
//          robot.liftMotor.setPower(x);
//          robot.claw.setPosition(x);
//          robot.liftMotor.setPower(x);
//          */
//
//
//
//        case GO_TO_LINE:
//
//          telemetry.addData("state", currentState.toString());
//          runtime.reset();
//          // distToLine(x, y, z);
//        case STOP:
//          telemetry.addData("state", currentState.toString());
//
//          stop();
//          break;
//
//        default:
//          telemetry.addData("state", currentState.toString());
//
//          stop();
//          break;
//      }
//      telemetry.update();
//    }
//  }
//}
