package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class TTRobotTest {
  private MockRobot mockRobot;
  private TTRobot ttRobot;
  private @Mock LinearOpMode opMode;
  private Telemetry telemetry = mock(Telemetry.class);


  @org.junit.jupiter.api.Test
  void setLinearSlideDirection() {
    //when(lslideSwitch.getState()).thenReturn(true).thenReturn(false).thenReturn(true);
    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
    verify(mockRobot.lslideServo).setPower(0.0);

    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
    verify(mockRobot.lslideServo, times(2)).setPower(0.0);

    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
    verify(mockRobot.lslideServo).setPower(-0.5);
    verify(mockRobot.lslideSwitch, never()).getState();
  }


  @org.junit.jupiter.api.Test
  void rotateClaw() {
    final double left = 0;
    final double center = 0.3;
    final double right = 1;

    ttRobot.rotateClaw(true);
    verify(mockRobot.grabTurn).setPosition(right);
    verify(mockRobot.grabTurn, never()).setPosition(center);
    verify(mockRobot.grabTurn, never()).setPosition(left);

    ttRobot.rotateClaw(true);
    verify(mockRobot.grabTurn).setPosition(right);
    verify(mockRobot.grabTurn, never()).setPosition(center);
    verify(mockRobot.grabTurn, never()).setPosition(left);

    ttRobot.rotateClaw(false);
    verify(mockRobot.grabTurn).setPosition(right);
    verify(mockRobot.grabTurn).setPosition(center);
    verify(mockRobot.grabTurn, never()).setPosition(left);

    ttRobot.rotateClaw(true);
    verify(mockRobot.grabTurn, times(2)).setPosition(right);
    verify(mockRobot.grabTurn).setPosition(center);
    verify(mockRobot.grabTurn, never()).setPosition(left);

    ttRobot.rotateClaw(true);
    verify(mockRobot.grabTurn, times(2)).setPosition(right);
    verify(mockRobot.grabTurn).setPosition(center);
    verify(mockRobot.grabTurn, never()).setPosition(left);

    ttRobot.rotateClaw(false);
    verify(mockRobot.grabTurn, times(2)).setPosition(right);
    verify(mockRobot.grabTurn, times(2)).setPosition(center);
    verify(mockRobot.grabTurn, never()).setPosition(left);

    ttRobot.rotateClaw(false);
    verify(mockRobot.grabTurn, times(2)).setPosition(right);
    verify(mockRobot.grabTurn, times(2)).setPosition(center);
    verify(mockRobot.grabTurn).setPosition(left);

    ttRobot.rotateClaw(false);
    verify(mockRobot.grabTurn, times(2)).setPosition(right);
    verify(mockRobot.grabTurn, times(2)).setPosition(center);
    verify(mockRobot.grabTurn).setPosition(left);

    ttRobot.rotateClaw(true);
    verify(mockRobot.grabTurn, times(2)).setPosition(right);
    verify(mockRobot.grabTurn, times(3)).setPosition(center);
    verify(mockRobot.grabTurn).setPosition(left);

    ttRobot.rotateClaw(false);
    verify(mockRobot.grabTurn, times(2)).setPosition(right);
    verify(mockRobot.grabTurn, times(3)).setPosition(center);
    verify(mockRobot.grabTurn, times(2)).setPosition(left);

    ttRobot.rotateClaw(false);
    verify(mockRobot.grabTurn, times(2)).setPosition(right);
    verify(mockRobot.grabTurn, times(3)).setPosition(center);
    verify(mockRobot.grabTurn, times(2)).setPosition(left);
  }

  @BeforeEach
  void setUp() {
    mockRobot = new MockRobot();
    ttRobot = mockRobot.buildMockRobot(opMode, telemetry);
  }
}