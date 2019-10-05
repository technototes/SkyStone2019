package org.firstinspires.ftc.teamcode;

public class Stick {
  double angle() { return 0.0; }
  double force() { return 0.0; }
  Direction dir() {
    if (force() < .1) {
      return Direction.None;
    }
    double d = angle();
    // This is assuming we're reading in degrees from 'up', clockwise
    if (d < 45.0 || d > 315.0) {
      return Direction.Up;
    } else if (d < 135.0) {
      return Direction.Right;
    } else if (d < 225.0) {
      return Direction.Down;
    } else {
      // It's between 225 & 315, so it's go left
      return Direction.Left;
    }
  }
}
