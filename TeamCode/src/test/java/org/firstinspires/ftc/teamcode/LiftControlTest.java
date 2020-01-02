package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class LiftControlTest {
  private MockRobot mockRobot;
  private MockOpMode opMode = new MockOpMode();
  private LiftControl liftControl;
  private static final double DOWN_POWER = 0.5;
  private static final double UP_POWER = -1.0;
  private static final int BRICK_HEIGHT = 1200;
  private static final int BASE_PLATE_HEIGHT = 800;

  @BeforeEach
  void setUp() {
    mockRobot = new MockRobot();
    liftControl = new LiftControl(opMode, mockRobot.lLiftMotor, mockRobot.rLiftMotor);
    opMode.start();
  }

  @Test
  void initialize() {
    // Ensure lift motors work together: they're facing opposite directions
    verify(mockRobot.lLiftMotor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    verify(mockRobot.rLiftMotor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    verify(mockRobot.lLiftMotor).setDirection(DcMotor.Direction.FORWARD);
    verify(mockRobot.rLiftMotor).setDirection(DcMotor.Direction.REVERSE);

    // Ensure the motors *stop* when we hit zero, not float around
    verify(mockRobot.lLiftMotor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    verify(mockRobot.rLiftMotor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  @Test
  void upAndStop() {
    liftControl.up();
    assertEquals(UP_POWER, mockRobot.lLiftPower);
    assertEquals(UP_POWER, mockRobot.rLiftPower);

    liftControl.stop();
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void upAndStopsAtMax() {
    mockRobot.setLiftPositions(2000, 2000);
    liftControl.up();
    assertEquals(UP_POWER, mockRobot.lLiftPower);
    assertEquals(UP_POWER, mockRobot.rLiftPower);

    // move motors to near maximum position and up is requested again
    mockRobot.setLiftPositions(6850, 6860);
    liftControl.up();

    // Motors should be stopped
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void downAndStop() {
    // Pretend the motors are above zero
    mockRobot.setLiftPositions(500, 500);

    liftControl.down();
    assertEquals(DOWN_POWER, mockRobot.lLiftPower);
    assertEquals(DOWN_POWER, mockRobot.rLiftPower);

    liftControl.stop();
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void downStopsAtZero() {
    // Pretend the motors are above zero
    mockRobot.setLiftPositions(500, 500);

    liftControl.down();
    assertEquals(DOWN_POWER, mockRobot.lLiftPower);
    assertEquals(DOWN_POWER, mockRobot.rLiftPower);

    // Pretend the motors are near zero and down is requested again
    mockRobot.setLiftPositions(10, 15);
    liftControl.down();

    // Motors should be stopped
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void liftBrick() {
    Thread stopTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(100);
          } catch (InterruptedException ignored) {}
          assertEquals(UP_POWER, mockRobot.lLiftPower);
          assertEquals(UP_POWER, mockRobot.rLiftPower);
          mockRobot.setLiftPositions(BRICK_HEIGHT + BASE_PLATE_HEIGHT, BRICK_HEIGHT + BASE_PLATE_HEIGHT);
        }
      }
    );

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);

    ElapsedTime runTime = new ElapsedTime();
    stopTimer.start();
    liftControl.LiftBrickWait(1);

    double msDuration = runTime.milliseconds();
    assertTrue(msDuration < 120, "msDuration (" + msDuration + ") less than 120");
    assertTrue(msDuration >= 95, "msDuration (" + msDuration + ") greater or equal to 95");

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void acquireBrick() {
    Thread stopTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(100);
          } catch (InterruptedException ignored) {}
          assertEquals(DOWN_POWER, mockRobot.lLiftPower);
          assertEquals(DOWN_POWER, mockRobot.rLiftPower);
          mockRobot.setLiftPositions(0, 0);
        }
      }
    );

    // Start above zero
    mockRobot.setLiftPositions(300, 300);
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);

    ElapsedTime runTime = new ElapsedTime();
    stopTimer.start();
    liftControl.AcquireBrickWait();

    double msDuration = runTime.milliseconds();
    assertTrue(msDuration < 120, "msDuration (" + msDuration + ") less than 120");
    assertTrue(msDuration >= 90, "msDuration (" + msDuration + ") greater or equal to 90");

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void setBrick() {
    Thread stopTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(100);
          } catch (InterruptedException ignored) {}
          assertEquals(DOWN_POWER, mockRobot.lLiftPower);
          assertEquals(DOWN_POWER, mockRobot.rLiftPower);
          mockRobot.setLiftPositions(1300, 1300);
        }
      }
    );

    // Start above one brick position
    mockRobot.setLiftPositions(1500, 1500);
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);

    ElapsedTime runTime = new ElapsedTime();
    stopTimer.start();
    liftControl.SetBrickWait();

    double msDuration = runTime.milliseconds();
    assertTrue(msDuration < 120, "msDuration (" + msDuration + ") less than 120");
    assertTrue(msDuration >= 95, "msDuration (" + msDuration + ") greater or equal to 95");

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }
}