package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "hebeMath")
 public class math extends OpMode {
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        double sum = gamepad1.right_stick_y - gamepad1.left_stick_y;

        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("button B", gamepad1.b);
        telemetry.addData("sum", sum);
    }
 }