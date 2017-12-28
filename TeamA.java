package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.BrokenBarrierException;

/**
 * Created by Aawon on 12/20/2017.
 */

@TeleOp(name = "Senior Team", group = "First")

public class TeamA extends OpMode {

    DcMotor FLeft;
    DcMotor FRight;

    DcMotor BLeft;
    DcMotor BRight;

    DcMotor Lift_Motor;
    DcMotor Extention;

    Servo Claw_1;
    Servo Claw_2;

    @Override
    public void init() {
        FLeft = hardwareMap.dcMotor.get("FLeft");
        FRight = hardwareMap.dcMotor.get("FRight");

        BLeft = hardwareMap.dcMotor.get("BLeft");
        BRight = hardwareMap.dcMotor.get("BRight");

        Lift_Motor = hardwareMap.dcMotor.get("Lift_Motor");

        Extention = hardwareMap.dcMotor.get("Extention");

        Claw_1 = hardwareMap.servo.get("Claw_1");
        Claw_2 = hardwareMap.servo.get("Claw_2");

        FRight.setDirection(DcMotor.Direction.REVERSE);
        BRight.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
    public void loop() {
        //Forward Backward
        if (gamepad1.left_stick_y >= 1)
        {
            FLeft.setPower(1.0);
            FRight.setPower(1.0);
        }
        else if (gamepad1.left_stick_y < 0)
        {
            BLeft.setPower(-1.0);
            BRight.setPower(-1.0);
        }
        //Right Left
        else if (gamepad1.x)
            {
            FRight.setPower(-1.0);
            BRight.setPower(1.0);

            FLeft.setPower(-1.0);
            BLeft.setPower(1.0);
        }
         else if (gamepad1.b)
        {
            FLeft.setPower(1.0);
            BLeft.setPower(-1.0);

            FRight.setPower(1.0);
            BRight.setPower(-1.0);
        }
        //Rotation
        else if (gamepad1.y)
        {
            FRight.setPower(1.0);
            BLeft.setPower(-1.0);
        }
        else if (gamepad1.a)
        {
            FLeft.setPower(1.0);
            BRight.setPower(1.0);
        }
        //Diagonal Movement
        else if (gamepad1.right_trigger>=1)
        {
            FRight.setPower(1.0);
            BLeft.setPower(1.0);
        }
        else if (gamepad1.left_trigger>=1)
        {
            FLeft.setPower(1.0);
            BRight.setPower(1.0);
        }
        else if (gamepad1.right_bumper)
        {
            FLeft.setPower(1.0);
            BRight.setPower(1.0);
        }
        else if (gamepad1.left_bumper)
        {
            FRight.setPower(1.0);
            BLeft.setPower(1.0);
        }
        }
}
