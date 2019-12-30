package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.verify;

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
    stopTimer.start();
    directControl.runOpMode();
  }

  @Test
  void liftOverride() {
    Thread releaseButtonTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(50);
          } catch (InterruptedException ignored) {}

          assertEquals(0.5, mockRobot.lLiftPower);
          assertEquals(0.5, mockRobot.rLiftPower);
          gamepad2.left_trigger = 0.0f;
          gamepad2.right_trigger = 0.0f;
          gamepad2.right_bumper = false;
          gamepad2.left_bumper = false;
          gamepad2.x = false;
        }
      }
    );

    Thread stopTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(100);
          } catch (InterruptedException ignored) {}
          directControl.stop();
        }
      }
    );

    gamepad2.left_trigger = 1.0f;
    gamepad2.right_trigger = 1.0f;
    gamepad2.right_bumper = true;
    gamepad2.left_bumper = true;
    gamepad2.x = true;

    directControl.start();
    releaseButtonTimer.start();
    stopTimer.start();
    directControl.runOpMode();

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }
}