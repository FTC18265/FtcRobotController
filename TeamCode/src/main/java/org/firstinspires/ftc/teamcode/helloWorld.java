package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "HelloHebe")
public class helloWorld extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","Hebe");
    }
    @Override
    public void loop() {

    }
}
