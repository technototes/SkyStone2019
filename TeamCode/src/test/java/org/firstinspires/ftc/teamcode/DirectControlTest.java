package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class DirectControlTest {
  private TTRobotTest ttRobotTest = new TTRobotTest();
  private TTRobot ttRobot = ttRobotTest.buildMockRobot();
  private DirectControl directControl = new DirectControl();

  @BeforeEach
  void setUp() {
    directControl.SetTestRobot(ttRobot);
  }

  @Test
  void runOpMode() {
    directControl.start();
    directControl.runOpMode();
  }
}