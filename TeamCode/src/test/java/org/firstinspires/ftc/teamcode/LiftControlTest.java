package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class LiftControlTest {
  private MockRobot mockRobot;
  private @Mock LinearOpMode opMode;
  private LiftControl liftControl;
  private static final double DOWN_POWER = 0.5;
  private static final double UP_POWER = -1.0;

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
  void upAndAutoStop() throws InterruptedException {
    liftControl.up();
    assertEquals(UP_POWER, mockRobot.lLiftPower);
    assertEquals(UP_POWER, mockRobot.rLiftPower);

    Thread.sleep(400);
    assertEquals(UP_POWER, mockRobot.lLiftPower);
    assertEquals(UP_POWER, mockRobot.rLiftPower);

    Thread.sleep(120);
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void downAndStop() {
    // Pretend the motors are above zero
    Mockito.lenient().when(mockRobot.lLiftMotor.getCurrentPosition()).thenReturn(500);
    Mockito.lenient().when(mockRobot.rLiftMotor.getCurrentPosition()).thenReturn(500);

    liftControl.down();
    assertEquals(DOWN_POWER, mockRobot.lLiftPower);
    assertEquals(DOWN_POWER, mockRobot.rLiftPower);

    liftControl.stop();
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void downAndAutoStop() throws InterruptedException {
    // Pretend the motors are above zero
    mockRobot.setLiftPositions(500, 500);

    liftControl.down();
    assertEquals(DOWN_POWER, mockRobot.lLiftPower);
    assertEquals(DOWN_POWER, mockRobot.rLiftPower);

    Thread.sleep(400);
    assertEquals(DOWN_POWER, mockRobot.lLiftPower);
    assertEquals(DOWN_POWER, mockRobot.rLiftPower);

    Thread.sleep(120);
    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void liftBrickAsync() throws InterruptedException {
    Thread stopTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(100);
          } catch (InterruptedException ignored) {}
          mockRobot.setLiftPositions(1600, 1600);
        }
      }
    );

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);

    liftControl.LiftBrickAsync(1);
    stopTimer.start();

    Thread.sleep(50);
    assertEquals(UP_POWER, mockRobot.lLiftPower);
    assertEquals(UP_POWER, mockRobot.rLiftPower);

    Thread.sleep(100);
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
          mockRobot.setLiftPositions(1600, 1600);
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
    assertTrue(msDuration >= 100, "msDuration (" + msDuration + ") greater or equal to 100");

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void acquireBrickAsync() throws InterruptedException {
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

    liftControl.AcquireBrickAsync();
    stopTimer.start();

    Thread.sleep(50);
    assertEquals(DOWN_POWER, mockRobot.lLiftPower);
    assertEquals(DOWN_POWER, mockRobot.rLiftPower);

    Thread.sleep(100);
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
    assertTrue(msDuration >= 100, "msDuration (" + msDuration + ") greater or equal to 100");

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }

  @Test
  void setBrickAsync() throws InterruptedException {
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

    liftControl.SetBrickAsync();
    stopTimer.start();

    Thread.sleep(50);
    assertEquals(DOWN_POWER, mockRobot.lLiftPower);
    assertEquals(DOWN_POWER, mockRobot.rLiftPower);

    Thread.sleep(100);
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
    assertTrue(msDuration >= 100, "msDuration (" + msDuration + ") greater or equal to 100");

    assertEquals(0.0, mockRobot.lLiftPower);
    assertEquals(0.0, mockRobot.rLiftPower);
  }
}