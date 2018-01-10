package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by JCTeamA on 12/20/2017.
 */

@TeleOp(name = "TeamA", group = "First")

public class TeamATeleOp extends OpMode {

    DcMotor FMLeft, FMRight, BMLeft, BMRight, Lift_Motor, Extension;
    Servo   Claw_1, Claw_2;
    Double xymove, zmove;


    @Override
    public void init()
    {
        telemetry.addData("Status:","Initialized");
        telemetry.addData("Status:","Ready to run!");
        telemetry.update();

        FMLeft = hardwareMap.dcMotor.get("FMLeft");
        FMRight = hardwareMap.dcMotor.get("FMRight");
        BMLeft = hardwareMap.dcMotor.get("BMLeft");
        BMRight = hardwareMap.dcMotor.get("BMRight");
        xymove = 1.0;
        zmove = 0.5;

        Lift_Motor = hardwareMap.dcMotor.get("Lift_Motor");

        Extension = hardwareMap.dcMotor.get("Extension");

        Claw_1 = hardwareMap.servo.get("Claw_1");
        Claw_2 = hardwareMap.servo.get("Claw_2");

        FMRight.setDirection(DcMotor.Direction.REVERSE);
        BMRight.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
    public void loop() {
        //Forward Backward
        if (gamepad1.left_stick_y >= 1)
        {
            FMLeft.setPower(xymove);
            FMRight.setPower(xymove);
        }
        else if (gamepad1.left_stick_y < 0)
        {
            BMLeft.setPower(-xymove);
            BMRight.setPower(-xymove);
        }
        //Right Left
        else if (gamepad1.left_stick_x>=1)
        {
            FMRight.setPower(-xymove);
            BMRight.setPower(xymove);

            FMLeft.setPower(-xymove);
            BMLeft.setPower(xymove);
        }
        else if (gamepad1.left_stick_x < 0)
        {
            FMLeft.setPower(xymove);
            BMLeft.setPower(-xymove);

            FMRight.setPower(xymove);
            BMRight.setPower(-xymove);
        }
        //Rotation
        else if (gamepad1.b)
        {
            FMRight.setPower(xymove);
            BMLeft.setPower(-xymove);
        }
        else if (gamepad1.a)
        {
            FMLeft.setPower(xymove);
            BMRight.setPower(xymove);
        }
        //Diagonal Movement
        else if (gamepad1.right_trigger>=1)
        {
            FMRight.setPower(xymove);
            BMLeft.setPower(xymove);
        }
        else if (gamepad1.left_trigger>=1)
        {
            FMLeft.setPower(xymove);
            BMRight.setPower(xymove);
        }
        else if (gamepad1.right_bumper)
        {
            FMLeft.setPower(xymove);
            BMRight.setPower(xymove);
        }
        else if (gamepad1.left_bumper)
        {
            FMRight.setPower(xymove);
            BMLeft.setPower(xymove);
        }
        //Gamepad two servo movement
        if (gamepad2.b)
        {
            Claw_1.setPosition(xymove);
            Claw_2.setPosition(xymove);
        }
        else if (gamepad2.a)
        {
            Claw_1.setPosition(0.0);
            Claw_2.setPosition(0.0);
        }
        //Lift Movement
        if (gamepad2.dpad_up)
        {
            Lift_Motor.setPower(zmove);
        }
        else if (gamepad2.dpad_down)
        {
            Lift_Motor.setPower(-zmove);
        }
        // Extension Movement
        if (gamepad2.x)
        {
            Extension.setPower(zmove);
        }
        else if (gamepad2.y)
        {
            Extension.setPower(-zmove);
        }
    }
}
