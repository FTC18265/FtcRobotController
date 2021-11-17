package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class practiceArray extends OpMode {
    programmingBoard2 programmingBoard2 = new programmingBoard2();
    int index = 0;
    double time = 0;
    String[]lyrics = {"What you know about rollin' down in the deep?",
            "When your brain goes numb, you can call that mental freeze",
            "When these people talk too much, put that shit in slow motion, yeah",
            "I feel like an astronaut in the ocean"};

    @Override
    public void init() {

    }
    public void start(){
        resetStartTime();
    }

    @Override
    public void loop() {
        if (getRuntime() > time +1){
            if (index < 4){
                time += 1;
                telemetry.addLine(lyrics[index]);
                index += 1;
            }
            else{
                index = 0;
            }
        }
    }
}
