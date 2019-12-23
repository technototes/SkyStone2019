//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//@TeleOp(name = "Direct Control Auto Grab")
//public class DirectControlAutoGrab extends LinearOpMode {
//  private static double FINEDRIVESPEED = 0.2;
//  private TTRobot robot;
//  private Controller control;
//  private Controller driver;
//
//  @Override
//  public void runOpMode() {
//    robot = new TTRobot(this, hardwareMap, telemetry);
//    // If you want telemetry, include a name as a string
//    // If you don't want telemetry, pass a null:
//    driver = new Controller(gamepad1, telemetry, "driver");
//    control = new Controller(gamepad2, telemetry, null);
//    telemetry.addLine("Hello!");
//    telemetry.update();
//    waitForStart();
//    while (opModeIsActive()) {
//
//      // Handle Grabber rotation
//      /*if (control.buttonA() == Button.Pressed) {
//        if (robot.getGrabberPosition() == GrabberPosition.Vertical) {
//          robot.snapGrabberPosition(GrabberPosition.Horizontal);
//        } else { // It'sHORIZONTAL!
//          robot.snapGrabberPosition(GrabberPosition.Vertical);
//        }
//      }*/
//      // Handle Grabber clutch
//      if(control.ltrigger() > robot.TRIGGERTHRESHOLD) {
//        robot.claw(0.4); // Open
//        telemetry.addLine("Open .4");
//      }else if (control.rtrigger() > robot.TRIGGERTHRESHOLD){
//        robot.claw(0.6); // CLosed
//        telemetry.addLine("Close .6");
//      }
//      // Grabber rotation
//      if(control.lbump() == Button.Pressed) {
//        robot.rotateClaw(0.4);
//        telemetry.addLine("Open 0.4");
//      }else if (control.rbump() == Button.Pressed){
//        robot.rotateClaw(0.6);
//        telemetry.addLine("Close 0.6");
//      }
//
//
//      if (control.buttonA() == Button.Pressed) {
//
//        robot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
//        while (robot.isLiftAtLowerLimit() == false) {
//          robot.liftDown();
//          robot.rotateClaw(0.6);
//
//        }
//        robot.claw(0.6);
//
//      }
//
//      if (control.buttonY() == Button.Pressed) {
//
//        robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
//
//        while (robot.isLiftAtLowerLimit()) {
//          robot.liftUp();
//          robot.rotateClaw(0.4);
//        }
//
//        robot.claw(0.6);
//      }
//
//      // redid this to work with magnetic limit switch
//      /*Direction dpad = control.dpad();
//      if(dpad.X == 1){
//          robot.lslide(LinearSlideOperation.Extend);
//      }else if(dpad.X == -1){
//          robot.lslide(LinearSlideOperation.Retract);
//      }*/
//      Direction slide = control.dpad();
//      /*
//      if (slide.isLeft()) {
//        robot.lslide(1);
//      }
//      else if (slide.isRight()) {
//        robot.lslide(-1);
//      }
//      else {
//        robot.lslide(0);
//      }
//      */
//      Direction dcontrols = driver.dpad();
//      if(dcontrols.isUp()){
//        robot.bpGrabber(1);
//      }else if(dcontrols.isDown()){
//        robot.bpGrabber(-1);
//      }else{
//        robot.bpGrabber(0);
//      }
//      if(dcontrols.isLeft()){
//        robot.capstone(-1);
//      }else if(dcontrols.isRight()){
//        robot.capstone(1);
//      }else{
//        robot.capstone(0);
//      }
//      // Lift control:
//      Direction dir = control.dpad();
//      if (dir.isUp()) {
//        robot.liftUp();
//      }
//      else if (dir.isDown()) {
//        robot.liftDown();
//      }
//      else {
//        robot.liftStop();
//      }
//
//      // Driver control:
//      Direction Dpad = driver.dpad();
//      Direction L = driver.lstick();
//      Direction R1 = driver.rstick();
//      Direction R2 = control.rstick();
//      Direction D = new Direction(0, 0);
//      Direction L2 = new Direction(0, 0);
//      if (Math.abs(R2.X) > robot.STICKDEADZONE) {
//        D.X = R2.X;
//      } else if (Math.abs(R1.X) > robot.STICKDEADZONE) {
//        D.X = R1.X;
//      }
//      if (Math.abs(L.X) > robot.STICKDEADZONE) {
//        L2.X = L.X;
//      }
//      if (Math.abs(L.Y) > robot.STICKDEADZONE) {
//        L2.Y = L.Y;
//      }
//
//      //Turbo Mode (insert Tristan happy face)
//      if ((control.rtrigger() == 1.0 || control.ltrigger() == 1.0 )) {
//        robot.speedSnail();
//      } else if ((driver.rtrigger() == 1.0 || driver.ltrigger() == 1.0 )) {
//        robot.speedTurbo();
//      } else {
//        robot.speedNormal();
//      }
//
//
//      robot.joystickDrive(L2, D, robot.gyroHeading());
//      /*if (control.buttonY() == Button.Pressed) {
//        robot.lslide(LinearSlideOperation.Extend);
//      } else if (control.buttonA() == Button.Pressed) {
//        robot.lslide(LinearSlideOperation.Retract);
//      } else {
//        // DO NOTHING, not "return;" :D
//      }*/
//
//
//      /*if(control.dpad().isDown()){
//        robot.joystickDrive(new Direction(-0,-FINEDRIVESPEED), new Direction(0,0), robot.gyroHeading());
//      }
//      if(control.dpad().isUp()){
//        robot.joystickDrive(new Direction(0,FINEDRIVESPEED), new Direction(0,0), robot.gyroHeading());
//      }
//      if(control.dpad().isLeft()){
//        robot.joystickDrive(new Direction(-FINEDRIVESPEED,0), new Direction(0,0), robot.gyroHeading());
//      }
//      if(control.dpad().isRight()){
//        robot.joystickDrive(new Direction(FINEDRIVESPEED,0), new Direction(0,0), robot.gyroHeading());
//      }*/
//      if (driver.buttonA() == Button.Pressed && driver.buttonB() == Button.Pressed) {
//        robot.joystickDrive(L2, D, 0);
//      }
//      else {
//        robot.joystickDrive(L2, D, robot.gyroHeading());
//      }
//      telemetry.update();
//
//    }
//  }
//}
