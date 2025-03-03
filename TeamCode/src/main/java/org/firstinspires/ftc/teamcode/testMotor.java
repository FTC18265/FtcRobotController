package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class testMotor extends testItem {
    private double speed;
    private DcMotor motor;
    public testMotor(String description, double speed, DcMotor motor) {
        super(description);
        this.speed = speed;
        this.motor = motor;
    }
    @Override

    public void run(boolean on, Telemetry telemetry) {
        if (on) {
            motor.setPower(speed);
        } else {
            motor.setPower(0.0);
        }
        telemetry.addData("Encoder:", motor.getCurrentPosition());
    }
}