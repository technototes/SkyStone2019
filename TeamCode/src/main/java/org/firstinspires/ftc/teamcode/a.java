package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "test")
public class a extends LinearOpMode {
  private DigitalChannel lslideSwitch;
  private Controller control;
  private Controller driver;

  @Override
  public void runOpMode() {
    lslideSwitch = hardwareMap.get(DigitalChannel.class, "mLimitSwitch");

    waitForStart();
    while (opModeIsActive()) {

      telemetry.addLine(lslideSwitch.getState() ? "true" : "false");

    }
    telemetry.update();
  }
}