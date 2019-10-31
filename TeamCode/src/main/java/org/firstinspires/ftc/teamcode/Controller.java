package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Controller {
    private Gamepad pad = null;
    private Telemetry telemetry = null;
    private String name = null;

    public Controller(Gamepad p, Telemetry tel, String nm) {
        pad = p;
        name = nm;
    }

    Direction dpad() {
        if (pad.dpad_down) {
            return getStick("d", 0, 1);
        }
        if (pad.dpad_up) {
            return getStick("d", 0, -1);
        }
        if (pad.dpad_left) {
            return getStick("d", -1, 0);
        }
        if (pad.dpad_right) {
            return getStick("d", 1, 0);
        }
        return Direction.None;
    }

    Button buttonA() {
        if (pad.a) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }

    Button buttonB() {
        if (pad.b) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }

    Button buttonX() {
        if (pad.x) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }

    Button buttonY() {
        if (pad.y) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }

    public Direction lstick() {
        return getStick("l", pad.left_stick_x, pad.left_stick_y);
    }

    public Direction rstick() {
        return getStick("r", pad.right_stick_x, pad.right_stick_y);
    }

    Button lbump() {
        if (pad.left_bumper) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }

    private Direction getStick(String which, float X, float Y) {
        if (name != null) {
            //telemetry.addData(name + "-" + which + "> ", "X:%3.2f, Y:%3.2f", (double)X, (double)Y);
        }
        return new Direction(X, Y);
    }

    Button rbump() {
        if (pad.right_bumper) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }

    double ltrigger() {
        return (double) pad.left_trigger;
    }

    double rtrigger() {
        return (double) pad.right_trigger;
    }

    Button back() {
        if (pad.back) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }

    Button mode() {
        if (pad.guide) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }

    Button start() {
        if (pad.start) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
    }
}
