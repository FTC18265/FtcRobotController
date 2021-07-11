package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class turboButton extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        boolean button = gamepad1.a;
        double stick = gamepad1.right_stick_y;

        if(button == false){
            telemetry.addData("Not pressed, ForwardSpeed: ", stick *= 0.5);
        }
        if (button == true){
            telemetry.addData("pressed, ForwardSpeed: ", stick);
        }
    }
}
