package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumController {
    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;

    public void MecanumController(DcMotor topleft, DcMotor topright, DcMotor bottomleft, DcMotor bottomright){
        this.topleft = topleft;
        this.topright = topright;
        this.bottomleft = bottomleft;
        this.bottomright = bottomright;
    }

    public void init(){
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topleft.setDirection(DcMotorSimple.Direction.FORWARD);
        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMode(DcMotor.RunMode mode){
        topleft.setMode(mode);
        topright.setMode(mode);
        bottomleft.setMode(mode);
        bottomright.setMode(mode);
    }

    public void setPower(double power) {
        topleft.setPower(power);
        topright.setPower(power);
        bottomleft.setPower(power);
        bottomright.setPower(power);
    }

    public void stopAllMotors() {
        topleft.setPower(0);
        bottomleft.setPower(0);
        topright.setPower(0);
        bottomright.setPower(0);
    }
}
