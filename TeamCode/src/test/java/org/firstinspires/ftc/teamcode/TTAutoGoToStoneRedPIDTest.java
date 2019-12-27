package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.AdditionalMatchers.gt;
import static org.mockito.AdditionalMatchers.lt;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class TTAutoGoToStoneRedPIDTest {
  private MockRobot mockRobot;
  private TTRobot ttRobot;
  private Telemetry telemetry = mock(Telemetry.class);
  private TTAutoGoToStoneRedPID autoGoToStoneRedPID;

  @BeforeEach
  void setUp() {
    mockRobot = new MockRobot();
    autoGoToStoneRedPID = new TTAutoGoToStoneRedPID();
    autoGoToStoneRedPID.telemetry = telemetry;
    ttRobot = mockRobot.buildMockRobot(autoGoToStoneRedPID, telemetry);
  }

  @Test
  void distRearDrivePIDNone() {
    mockRobot.setRearRangePositionCm(20);
    autoGoToStoneRedPID.start();
    autoGoToStoneRedPID.distRearDrivePID(ttRobot,20);
    verify(mockRobot.flMotor).setPower(0.0);
    verify(mockRobot.frMotor).setPower(0.0);
    verify(mockRobot.rlMotor).setPower(0.0);
    verify(mockRobot.rrMotor).setPower(0.0);
    verify(mockRobot.flMotor, never()).setPower(gt(0.0));
    verify(mockRobot.frMotor, never()).setPower(gt(0.0));
    verify(mockRobot.rlMotor, never()).setPower(gt(0.0));
    verify(mockRobot.rrMotor, never()).setPower(gt(0.0));
    verify(mockRobot.flMotor, never()).setPower(lt(0.0));
    verify(mockRobot.frMotor, never()).setPower(lt(0.0));
    verify(mockRobot.rlMotor, never()).setPower(lt(0.0));
    verify(mockRobot.rrMotor, never()).setPower(lt(0.0));
  }

  @Test
  void distRearDrivePIDForward() {
    Thread stopTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {}
          mockRobot.setRearRangePositionCm(22);
        }
      }
    );

    mockRobot.setRearRangePositionCm(0);
    ElapsedTime runTime = new ElapsedTime();
    autoGoToStoneRedPID.start();
    stopTimer.start();
    autoGoToStoneRedPID.distRearDrivePID(ttRobot, 20);
    double msDuration = runTime.milliseconds();
    assertTrue(msDuration < 110, "msDuration less than 110");
    assertTrue(msDuration >= 100, "msDuration greater or equal to 100");
  }
}