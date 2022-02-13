package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {
    private Rev2mDistanceSensor distancesensor;

    @Override
    public void runOpMode(){
        distancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "distancesensor");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("distance", distancesensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

}
