package org.firstinspires.ftc.teamcode;

//States for a Button
public enum Button {
  Pressed,
  Released;
  //functions
  public boolean isPressed() {
    return this == Pressed;
  }

  public boolean isReleased() {
    return this == Released;
  }
}
