package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


public class ReadFromController extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    if (gamepad1.dpad_up) {
      telemetry.addLine("Pressing DPad Up");
      telemetry.update();
    }
    if (gamepad1.dpad_down) {
      telemetry.addLine("Pressing DPad Down");
      telemetry.update();
    }
    if (gamepad1.dpad_left) {
      telemetry.addLine("Pressing DPad Left");
      telemetry.update();
    }
    if (gamepad1.dpad_right) {
      telemetry.addLine("Pressing DPad Right");
    }

  }
}
