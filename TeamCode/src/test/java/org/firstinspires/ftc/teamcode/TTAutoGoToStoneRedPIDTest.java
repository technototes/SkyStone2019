package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;

@ExtendWith(MockitoExtension.class)
class TTAutoGoToStoneRedPIDTest {
  private TTRobotTest ttRobotTest = new TTRobotTest();
  private TTRobot ttRobot = ttRobotTest.buildMockRobot();
  private DirectControl directControl = new DirectControl();

  @BeforeEach
  void setUp() {
  }

  @Test
  void distRearDrivePID() {
  }
}