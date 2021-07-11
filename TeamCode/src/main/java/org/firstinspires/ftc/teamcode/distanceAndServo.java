package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class distanceAndServo extends OpMode {
    programmingBoard2 programmingBoard2 = new programmingBoard2();

    @Override
    public void init() {

    }
    public  void start(){
        resetStartTime();
    }

    @Override
    public void loop() {
        while(programmingBoard2.getDistance(DistanceUnit.CM) > 10 || getRuntime() < 5){
            programmingBoard2.setMotorSpeed(0.3);
        }
        programmingBoard2.setServoPosition(gamepad1.left_trigger);

    }
}
