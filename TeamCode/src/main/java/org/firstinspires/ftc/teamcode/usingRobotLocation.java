package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class usingRobotLocation extends OpMode {
    robotLocation robotLocation = new robotLocation(0);

    @Override
    public void init() {
     robotLocation.setAngle(0);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            robotLocation.turn(0.1);
            }
        else if(gamepad1.b){
            robotLocation.turn(-0.1);
            }
        if(gamepad1.dpad_left){
            robotLocation.setX(-0.1);
        }
        else if(gamepad1.dpad_right){
            robotLocation.setX(0.1);
        }
        else if(gamepad1.dpad_up){
            robotLocation.setX(0.1);
        }
        else if(gamepad1.dpad_down){
            robotLocation.setX(-0.1);
        }

        telemetry.addData("Location", robotLocation);
        telemetry.addData("Heading", robotLocation.getHeading());
        telemetry.addData("double: ", robotLocation.doubleGetAngle());
        telemetry.addData("x: ", robotLocation.x);
        telemetry.addData("y: ", robotLocation.y);


    }
}