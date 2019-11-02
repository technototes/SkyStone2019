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
    lslideSwitch.setMode(DigitalChannel.Mode.INPUT);
    waitForStart();
    int i = 0;
    while (opModeIsActive()) {
      i++;
      telemetry.addData("Switch:", "%d: %s", i, lslideSwitch.getState() ? "true" : "false");
      sleep(10);
      telemetry.update();
    }
  }
}
