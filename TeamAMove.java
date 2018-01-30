package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by JCTeamA on 12/20/2017.
 */

@TeleOp(name = "TeamAMove", group = "First")

public class TeamAMove extends OpMode {

    DcMotor FMLeft, FMRight, BMLeft, BMRight,Extension,Lift_Motor2,Lift_Motor;
    Double xmove, ymove;
    Servo Servo1,Servo2,Servo3,Servo4;


    @Override
    public void init()
    {
        telemetry.addData("Status:","See it yah");
        telemetry.addData("Status:","Yo it ready yere");
        telemetry.update();

        FMLeft = hardwareMap.dcMotor.get("FMLeft");
        FMRight = hardwareMap.dcMotor.get("FMRight");
        BMLeft = hardwareMap.dcMotor.get("BMLeft");
        BMRight = hardwareMap.dcMotor.get("BMRight");
        Servo1 = hardwareMap.servo.get("S1");
        Servo2 = hardwareMap.servo.get("S2");
        Servo3 = hardwareMap.servo.get("S3");
        Servo4 = hardwareMap.servo.get("S4");
        Extension = hardwareMap.dcMotor.get("Extension");
        Lift_Motor = hardwareMap.dcMotor.get("Lift_Motor");
        Lift_Motor2 = hardwareMap.dcMotor.get("Lift_Motor2");
        Servo2.setDirection(Servo.Direction.REVERSE);
        Servo4.setDirection(Servo.Direction.REVERSE);
        Lift_Motor2.setDirection(DcMotor.Direction.REVERSE);
        xmove = 1.0;
        ymove = 1.0;

    }
    @Override
    public void loop() {
        //Forward Backward
        if (gamepad1.left_stick_y >= 1)
        {
            FMLeft.setPower(ymove);
            BMLeft.setPower(ymove);
        }
        else if (gamepad1.left_stick_y <=0)
        {
            FMLeft.setPower(-ymove);
            BMLeft.setPower(-ymove);
        }
        if (gamepad1.right_stick_y >=1)
        {
            FMRight.setPower(ymove);
            BMRight.setPower(ymove);
        }
        else if (gamepad1.right_stick_y <=0)
        {
            FMRight.setPower(-ymove);
            BMRight.setPower(-ymove);
        }
        //Right, Left
        if (gamepad1.left_stick_x >=1)
        {
            FMLeft.setPower(xmove);
            BMLeft.setPower(-xmove);
        }
        else if (gamepad1.left_stick_x <= 0)
        {
            FMLeft.setPower(-xmove);
            BMLeft.setPower(xmove);
        }
        if (gamepad1.right_stick_x >=1)
        {
            BMRight.setPower(xmove);
            FMRight.setPower(-xmove);
        }
        else if (gamepad1.right_stick_x <=0)
        {
            BMRight.setPower(-xmove);
            FMRight.setPower(xmove);
        }
        //Absolute stop Movement
        if (gamepad1.right_trigger == 1)
        {
            FMRight.setPower(0);
            BMRight.setPower(0);
            FMLeft.setPower(0);
            BMLeft.setPower(0);
        }
        //Servo Movement
        if (gamepad1.x)
        {
            Servo1.setPosition(1.0);
            Servo2.setPosition(1.0);
        }
        else if (gamepad1.a)
        {
            Servo1.setPosition(0.0);
            Servo2.setPosition(0.0);
        }
        if (gamepad1.b)
        {
            Servo3.setPosition(1.0);
            Servo4.setPosition(1.0);
        }
        else if (gamepad1.y)
        {
            Servo3.setPosition(0.0);
            Servo4.setPosition(0.0);
        }
        //Lift Movement
       /* if (gamepad1.dpad_up)
        {
            Lift_Motor.setPower(ymove);
            Lift_Motor2.setPower(ymove);
        }
        else if (gamepad1.dpad_down)
        {
            Lift_Motor.setPower(-ymove);
            Lift_Motor2.setPower(-ymove);
        }*/
        // Extension Movement
      /*  if (gamepad1.right_bumper)
        {
            Extension.setPower(xmove);
        }
        else if (gamepad1.left_bumper)
        {
            Extension.setPower(-xmove);
        }*/
    }
}