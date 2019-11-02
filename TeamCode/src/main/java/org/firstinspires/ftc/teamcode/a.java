package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "test")
public class a extends LinearOpMode {
  private DigitalChannel lslideSwitch;
  private DigitalChannel l11;
  private DigitalChannel l12;
  private DigitalChannel l13;
  private DigitalChannel l20;
  private DigitalChannel l21;
  private DigitalChannel l22;
  private DigitalChannel l23;

  private Controller control;
  private Controller driver;
  private String val(DigitalChannel chan) {
    return chan.getState() == true ? "true" : "false";
  }
  @Override
  public void runOpMode() {
    lslideSwitch = hardwareMap.get(DigitalChannel.class, "mLimitSwitch");
     l11 = hardwareMap.get(DigitalChannel.class, "limit11");
     l12 = hardwareMap.get(DigitalChannel.class, "limit12");
     l13 = hardwareMap.get(DigitalChannel.class, "limit13");
     l20 = hardwareMap.get(DigitalChannel.class, "limit20");
     l21 = hardwareMap.get(DigitalChannel.class, "limit21");
     l22 = hardwareMap.get(DigitalChannel.class, "limit22");
     l23 = hardwareMap.get(DigitalChannel.class, "limit23");

    lslideSwitch.setMode(DigitalChannel.Mode.INPUT);
    l11.setMode(DigitalChannel.Mode.INPUT);
    l12.setMode(DigitalChannel.Mode.INPUT);
    l13.setMode(DigitalChannel.Mode.INPUT);
    l20.setMode(DigitalChannel.Mode.INPUT);
    l21.setMode(DigitalChannel.Mode.INPUT);
    l22.setMode(DigitalChannel.Mode.INPUT);
    l23.setMode(DigitalChannel.Mode.INPUT);
    waitForStart();
    int i = 0;
    while (opModeIsActive()) {
      i++;
      telemetry.addData("slide Switch:", "%d: %s", i, val(lslideSwitch));
      telemetry.addData("l11 Switch:", "%d: %s", i, val(l11));
      telemetry.addData("l12 Switch:", "%d: %s", i, val(l12));
      telemetry.addData("l13 Switch:", "%d: %s", i, val(l13));
      telemetry.addData("l20 Switch:", "%d: %s", i, val(l20));
      telemetry.addData("l21 Switch:", "%d: %s", i, val(l21));
      telemetry.addData("l22 Switch:", "%d: %s", i, val(l22));
      telemetry.addData("l23 Switch:", "%d: %s", i, val(l23));
      sleep(10);
      telemetry.update();
    }
  }
}
