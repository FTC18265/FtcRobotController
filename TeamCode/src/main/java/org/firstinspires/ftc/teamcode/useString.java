package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "hebeString")
public class useString extends OpMode {
    @Override
    public void init() {
         String myName = "Hebe He";
         int grade = 99;
         telemetry.addData("Hello", myName);
         telemetry.addData("grade:", grade);
         }
         @Override
         public void loop() {

         }
}
