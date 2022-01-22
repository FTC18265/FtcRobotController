package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "servoTest")
public class ServoTest extends LinearOpMode {

    private Servo pusher;


    @Override
    public void runOpMode() {


        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setPosition(1);



        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (gamepad1.y) {
                    pusher.setPosition(0);
                }else{
                    pusher.setPosition(1);

                }


            }


        }

    }


}
