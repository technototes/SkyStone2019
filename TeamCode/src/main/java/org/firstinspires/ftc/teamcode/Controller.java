package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Controller {
    private Gamepad pad = null;
    private Telemetry telemetry = null;

    public Controller(Gamepad p, Telemetry tel) {
        pad = p;
    }

    Direction dpad() {
        if (pad.dpad_down) {
            return new Direction(0, 1);
        }
        if (pad.dpad_up) {
            return new Direction(0, -1);
        }
        if (pad.dpad_left) {
            return new Direction(-1, 0);
        }
        if (pad.dpad_right) {
            return new Direction(1, 0);
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
        return new Direction(pad.left_stick_x, pad.left_stick_y);
    }

    public Direction rstick() {
        return new Direction(pad.right_stick_x, pad.right_stick_y);
    }

    Button lbump() {
        if (pad.left_bumper) {
            return Button.Pressed;
        } else {
            return Button.Released;
        }
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
