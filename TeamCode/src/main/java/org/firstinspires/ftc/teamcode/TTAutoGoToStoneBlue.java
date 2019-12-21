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
//@Autonomous(name = "TTAutoGoToStoneBlue", group = "TT")
//public class TTAutoGoToStoneBlue extends LinearOpMode {
//
//  // States
//  private enum AutoState {
//    INITIALIZE,
//    GOTOBLOCK1,
//    GOTOBLOCK2,
//    GOTOBLOCK3,
//    GOFORWARD,
//    EXTENDSLIDE,
//    DROPLIFT,
//    GRABBLOCK,
//    GO_TO_BASE_PLATE,
//    PLACE_STONE,
//    GO_TO_LINE,
//    STOP
//  }
//
//
//  private AutoState currentState = AutoState.INITIALIZE;
//  private ElapsedTime runtime = new ElapsedTime();
//  private ElapsedTime timer = new ElapsedTime();
//  private TTRobot robot;
//  private ElapsedTime driveTime = new ElapsedTime();
//  private ElapsedTime runTime = new ElapsedTime();
//
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
//
//    // Put vuforia Here
//
//    waitForStart();
//
//    Truphoria tf = new Truphoria(hardwareMap, telemetry);
//    robot.distRearDrive(1, 5);
//    robot.distRightDrive(0.5, 90, 45);
//    robot.distRightDrive(0.3, -90, 45);
//    runtime.reset();
//    while (runtime.seconds() < 2) {
//      tf.takeALook();
//      telemetry.addData("tfdata ", tf.whichColumn());
//      telemetry.addData("tfconf ", tf.confidence());
//      telemetry.update();
//    }
//    /*
//    if (tfod != null) {
//        tfod.deactivate();
//    }
//    */
//    // run until the end of the match (driver presses STOP)
//    int blockPos = tf.whichColumn();
//
//    telemetry.update();
//    while (opModeIsActive()) {
//      telemetry.addData("Status", "Run Time: " + runtime.toString());
//      switch (currentState) {
//        case INITIALIZE:
//          if (opModeIsActive()) {
//            telemetry.addData("state", currentState.toString());
//            runtime.reset();
//          /*
//          if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
//              tfod.activate();
//          }
//          */
//            switch (blockPos) {
//              case 2:
//                currentState = AutoState.GOTOBLOCK1;
//                break;
//              case 1:
//                currentState = AutoState.GOTOBLOCK2;
//                break;
//              case 0:
//                currentState = AutoState.GOTOBLOCK3;
//                break;
//            }
//          }
//          break;
//
//        case GOTOBLOCK1:
//          if (opModeIsActive()) {
//            telemetry.addData("state", currentState.toString());
//            robot.distRightDrive(0.5, -90, 90);
//            robot.distRightDrive(0.3, 90, 90);
//            currentState = AutoState.GOFORWARD;
//          }
//          break;
//        case GOTOBLOCK2:
//          if (opModeIsActive()) {
//            telemetry.addData("state", currentState.toString());
//            robot.distRightDrive(0.5, -90, 70);
//            robot.distRightDrive(0.3, 90, 70);
//            currentState = AutoState.GOFORWARD;
//          }
//          break;
//        case GOTOBLOCK3:
//          if (opModeIsActive()) {
//            telemetry.addData("state", currentState.toString());
//            robot.distRightDrive(0.5, 90, 45);
//            //robot.distRightDrive(0.3, -90, 55);
//            currentState = AutoState.GOFORWARD;
//          }
//          break;
//        case GOFORWARD:
//          if (opModeIsActive()) {
//            telemetry.addData("state", currentState.toString());
//            robot.distRearDrive(0.75, 100);
//            driveTime.reset();
//            while (driveTime.seconds() < 1) {
//              robot.liftUp();
//              robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
//            }
//            while (driveTime.seconds() < 1.5) {
//              robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
//            }
//            robot.liftStop();
//            robot.setLinearSlideDirection(LinearSlideOperation.None, true);
//
//            currentState = AutoState.EXTENDSLIDE;
//          }
//          break;
//        case EXTENDSLIDE:
//          if (opModeIsActive()) {
//
//            telemetry.addData("state", currentState.toString());
//            driveTime.reset();
//            while (driveTime.seconds() < 1) {
//              robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
//            }
//            //while (driveTime.seconds() < 4 && !robot.slideSwitchSignaled()) {
//            //robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
//            //}
//            robot.setLinearSlideDirection(LinearSlideOperation.None, false);
//            driveTime.reset();
//            robot.rotateClaw(1);
//            robot.claw(1.0);
//            //while(driveTime.seconds() < 0.9) {
//            //robot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
//            //}
//            robot.setLinearSlideDirection(LinearSlideOperation.None, true);
//            driveTime.reset();
//            while (driveTime.seconds() < 1.5 && !robot.liftSwitchSignaled()) {
//              robot.liftDown();
//
//            }
//            robot.liftStop();
//
//            robot.setLinearSlideDirection(LinearSlideOperation.None, false);
//
//            currentState = AutoState.GRABBLOCK;
//
//            // distToLine(x, y, z);
//          }
//          break;
//        case GRABBLOCK:
//          if (opModeIsActive()) {
//
//            telemetry.addData("state", currentState.toString());
//
//            robot.claw(0.0);
//            sleep(500);
//            while (driveTime.seconds() < 0.25) {
//              robot.liftUp();
//            }
//
//            currentState = AutoState.STOP;
//            // distToLine(x, y, z);
//            robot.distRearDrive(0.75, 55);
//            robot.syncTurn(-90, 3);
//            driveTime.reset();
//
//            stop();
//          }
//          break;
//      }
//      telemetry.update();
//    }
//  }
//}
