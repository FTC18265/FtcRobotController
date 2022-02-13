package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoTest")
public class ServoTest extends LinearOpMode {

    private Servo shippingArm;
    private double currenttime;
    private double lasttime;
    private double position = 1;

    @Override
    public void runOpMode() {
        shippingArm = hardwareMap.get(Servo.class, "shippingArm");
        shippingArm.setPosition(1);

        //1 -starting position

        waitForStart();

        while(opModeIsActive()){
            currenttime = getRuntime();
            if(gamepad1.a && currenttime - lasttime > 0.5){
                lasttime = currenttime;
                position = position - 0.05;
            }
            if(gamepad1.y && currenttime - lasttime > 0.5){
                lasttime = currenttime;
                position = position + 0.05;
            }

            if(position > 1){
                position = 1;
            }
            if(position < 0){
                position = 0;
            }

            shippingArm.setPosition(position);
            telemetry.addData("position", position);
            telemetry.update();
        }


    }


}