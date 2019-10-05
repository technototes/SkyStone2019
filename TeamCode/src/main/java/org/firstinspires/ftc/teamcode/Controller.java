package org.firstinspires.ftc.teamcode;

public class Controller {
  Direction dpad() {
    return Direction.None;
  }
  Button buttonA() {
    return Button.Released;
  }
  Button buttonB() {
    return Button.Released;
  }
  Button buttonX() {
    return Button.Released;
  }
  Button buttonY() {
    return Button.Released;
  }
  public Stick leftStick;
  public Direction lstick() {
    return leftStick.dir();
  }
  public Stick rightStick;
  public Direction rstick() {
    return rightStick.dir();
  }
  Button lbump() {
    return Button.Released;
  }
  Button rbump() {
    return Button.Released;
  }
  double ltrigger() {
    return 0.0;
  }
  double rtrigger() {
    return 0.0;
  }
  Button back() {
    return Button.Released;
  }
  Button mode() {
    return Button.Released;
  }
  Button start() {
    return Button.Released;
  }
}
