package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



/**
 * Created by Aaron on 12/20/2017.
 */

@TeleOp(name = "Team_B", group = "First")
public class Team_B extends OpMode {

    DcMotor A1, A2, A3, A4, Gear_centre;
    Servo Extension_A, Extension_B,B1, B2;

    double Fast,Slow;

    @Override
    public void init() {
        telemetry.addData("Status:", "It Loading");
        telemetry.addData("Status:", "Soon Start!");
        telemetry.update();

        A1 = hardwareMap.dcMotor.get("Drive1");
        A2 = hardwareMap.dcMotor.get("Drive2");

        A3 = hardwareMap.dcMotor.get("Drive3");
        A4 = hardwareMap.dcMotor.get("Drive4");

        Gear_centre= hardwareMap.dcMotor.get("GCentre");

        Extension_A = hardwareMap.servo.get("ExtensionT");
        Extension_B = hardwareMap.servo.get("ExtensionB");

        B1 = hardwareMap.servo.get("B_1");
        B2 = hardwareMap.servo.get("B_2");

        Fast = 1.0;
        Slow = 0.5;
    }
    @Override
    public void loop() {
        //Gamepad One
        // Forward Backward
        if (gamepad1.left_stick_y >= 1)
        {

            A1.setPower(gamepad1.left_stick_y);
            A2.setPower(gamepad1.left_stick_y);
        }
        else if (gamepad1.left_stick_y <= 0)
        {

            A3.setPower(-gamepad1.left_stick_y);
            A4.setPower(-gamepad1.left_stick_y);
        }
        //Right Left
        else if (gamepad1.left_stick_x >= 0)
        {
            A2.setPower(-gamepad1.left_stick_x);
            A4.setPower(gamepad1.left_stick_x);

            A1.setPower(-gamepad1.left_stick_x);
            A3.setPower(gamepad1.left_stick_x);
        }
        else if (gamepad1.left_stick_x <= 0)
        {
            A1.setPower(gamepad1.left_stick_x);
            A3.setPower(-gamepad1.left_stick_x);

            A2.setPower(gamepad1.left_stick_x);
            A4.setPower(gamepad1.left_stick_x);
        }
        //Rotation
        else if (gamepad1.b) {

            A2.setPower(Slow);
            A3.setPower(-Slow);
        }
        else if (gamepad1.a) {

            A1.setPower(Slow);
            A4.setPower(Slow);
        }
        //Diagonal Movement
        else if (gamepad1.right_trigger >= 1) {
            A2.setPower(gamepad1.right_trigger);
            A3.setPower(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger >= 1) {
            A1.setPower(gamepad1.left_trigger);
            A4.setPower(gamepad1.left_trigger);
        }
        else if (gamepad1.right_bumper) {
            A1.setPower(Fast);
            A4.setPower(Fast);
        }
        else if (gamepad1.left_bumper) {
            A2.setPower(Fast);
            A3.setPower(Fast);
        }
        // Gamepad Two
        // Pull-in movement
        if (gamepad1.b) {
            B1.setPosition(1.0);
            B2.setPosition(1.0);
        }
        else if (gamepad1.a){
            B1.setPosition(0.0);
            B2.setPosition(0.0);
        }
        //Extension_1 Movement
        if (gamepad1.y)
        {
            Extension_A.setPosition(1.0);
        }
        else if (gamepad1.x)
        {
            Extension_A.setPosition(0.0);
        }
        // Extension Movement
        if (gamepad1.y)
        {
            Extension_B.setPosition(1.0);
        }
        else if (gamepad1.x)
        {
            Extension_B.setPosition(0.0);
        }
        //Gears Movement
        if (gamepad1.b)
        {
            Gear_centre.setPower(Fast);
    }


    }
}