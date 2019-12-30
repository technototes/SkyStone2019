package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;

@ExtendWith(MockitoExtension.class)
class DirectControlTest {
  private MockRobot mockRobot;
  private TTRobot ttRobot;
  private DirectControl directControl = new DirectControl();
  private @Mock Gamepad gamepad1;
  private @Mock Gamepad gamepad2;
  private @Mock Telemetry telemetry;

  @BeforeEach
  void setUp() {
    mockRobot = new MockRobot();
    ttRobot = mockRobot.buildMockRobot(directControl, telemetry);

    directControl.SetTestRobot(ttRobot);
    directControl.gamepad1 = gamepad1;
    directControl.gamepad2 = gamepad2;
    directControl.telemetry = telemetry;
  }

  @Test
  void runOpMode() {
    Thread stopTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {}
          directControl.stop();
        }
      }
    );
    directControl.start();

    ElapsedTime runTime = new ElapsedTime();
    stopTimer.start();
    directControl.runOpMode();
    double msDuration = runTime.milliseconds();
    assertTrue(msDuration < 120, "msDuration (" + msDuration + ") less than 120");
    assertTrue(msDuration >= 100, "msDuration (" + msDuration + ") greater or equal to 100");
  }
}