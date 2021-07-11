package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class crazyMode extends OpMode {
    boolean button = gamepad1.a;
    double gameStickX = gamepad1.right_stick_x;
    double gameStickY = gamepad1.right_stick_y;

    public void init() {

    }

    @Override
    public void loop() {
        if (button == true){
            gameStickX = gameStickY;
            gameStickY = gameStickX;

            telemetry.addData("CrazyMode On, x: ", gameStickX);
            telemetry.addData("y: ", gameStickY);
        }
        if (button == false){
            telemetry.addData("Normal, x:", gameStickX);
            telemetry.addData("y: ", gameStickY);
        }
    }

}
