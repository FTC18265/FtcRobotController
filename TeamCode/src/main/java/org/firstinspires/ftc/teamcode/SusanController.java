package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SusanController {
    private DcMotorEx susan;
    /*
    public static final double turning_NEW_P = 2.5;
    public static final double turning_NEW_I = 0.75;
    public static final double turning_NEW_D = 0.0;
    public static final double turning_NEW_F = 0;
     */
    public static final double turning_NEW_P = 2.75;
    public static final double turning_NEW_I = 1.5;
    public static final double turning_NEW_D = 0.0;
    public static final double turning_NEW_F = 0;
    private int degree = 1;
    public int position;

    public SusanController (DcMotorEx susan){
        this.susan = susan;
    }

    public void init (){
        susan.setDirection(DcMotorSimple.Direction.REVERSE);

        susan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int position = susan.getCurrentPosition();
        susan.setTargetPosition(position);
        susan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        susan.setPower(0);

        susan.setVelocityPIDFCoefficients(turning_NEW_P, turning_NEW_I, turning_NEW_D, turning_NEW_F);
        susan.setPositionPIDFCoefficients(1);
        //susan.setPositionPIDFCoefficients(1);
        susan.setTargetPositionTolerance(1);

        susan.setPower(0.5);

    }


    public void adjustLevel(int offset){
        degree = degree + offset;

        if (degree < -1){
            degree = -1;
        }
        if (degree > 1){
            degree = 1;
        }

        if (degree == 1){
            susan.setPower(0.5);
            susan.setTargetPosition(0);

        }
        else if (degree == -1){
            susan.setPower(0.5);
            susan.setTargetPosition(-1500);

        }

        else if (degree == 0){
            susan.setPower(0.5);
            susan.setTargetPosition(-750);

        }
    }

    public int getCurrentPosition(){
        return susan.getCurrentPosition();
    }

    public void microAjust(int offset){
        position = susan.getCurrentPosition();
        position = position + offset;

        if (position > 0){
            position = 0;
        }
        if (position < -1500){
            position = -1500;
        }

        susan.setTargetPosition(position);
    }

    public void autoLevel(int degree){
        if (degree == 1){
            susan.setTargetPosition(0);
        }
        if (degree == 0){
            susan.setTargetPosition(-700);
        }
        if (degree == -1){
            susan.setTargetPosition(-1400);
        }
    }

}
