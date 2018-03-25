package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by chava on 18/02/2018.
 *
 * Created for the use of testing
 */
@TeleOp(name = "WorldTesting", group = "Champs")
public class WorldsTesting extends LinearOpMode {
    DcMotor Fright, Fleft, Bright, Bleft, Intake1,Intake2,Lift;
    Servo ClawR, ClawL;
    Double mode, FrightP, FleftP, BrightP, BleftP, Xinput, Yinput;
    int x;

    public void movement() {
/*
Here, gamepad 1's Left stick is used as a graph like feature where it uses the its respective x and y values.
 */
        Xinput = (double) gamepad1.left_stick_x;
        Yinput = (double) gamepad1.left_stick_y;
        //use of a Range function to limit the values that enter the variables that give power to the motor
        //which allows for movement in most directions
        FrightP = Range.clip(Yinput + (+Xinput), -1.0, 1.0);

        FleftP = Range.clip(Yinput + (+Xinput), -1.0, 1.0);

        BrightP = Range.clip(Yinput + (-Xinput), -1.0, 1.0);

        BleftP = Range.clip(Yinput + (-Xinput), -1.0, 1.0);
        // Here limits the power entering the motor even further to create a "perfection" like movement

        // Here the power gets applied to the motors

        Fright.setPower(-FrightP); //FrightP, BrightP, FleftP, FleftP
        Fleft.setPower(BleftP);
        Bright.setPower(-BrightP);
        Bleft.setPower(FleftP);
        //This statement uses the right trigger of the gamepad to allow for rotational movement using the x axis of the stick.
        if (gamepad1.right_stick_x != 0) {
            mode = (double) gamepad1.right_stick_x;
            Fright.setPower(mode);
            Fleft.setPower(mode);
            Bright.setPower(mode);
            Bleft.setPower(mode);
        }
    }

    public void Intake() {
        if (gamepad1.right_trigger != 0) {
            Intake1.setPower(gamepad1.right_trigger);
            Intake2.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger != 0) {
            Intake1.setPower(-gamepad1.left_trigger);
            Intake2.setPower(-gamepad1.left_trigger);
        } else {
            Intake1.setPower(0.0);
            Intake2.setPower(0.0);
        }
    }

    public void Servos() {
        if (gamepad2.a) {
            close();
        }
        if (gamepad2.a == Boolean.FALSE && gamepad2.x == Boolean.FALSE) {
            open();
        }
    }

    public void close() {
        ClawL.setPosition(0.51);
        ClawR.setPosition(0.50);
        x = 0;
    }

    public void open() {
        ClawR.setPosition(0.56);
        ClawL.setPosition(0.58);
        x = 1;
    }

    public void Lift()
    {
        Lift.setPower(gamepad2.left_stick_y);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Fright = hardwareMap.dcMotor.get("Fright");
        Fleft = hardwareMap.dcMotor.get("Fleft");
        Bright = hardwareMap.dcMotor.get("Bright");
        Bleft = hardwareMap.dcMotor.get("Bleft");
        ClawL = hardwareMap.servo.get("ClawL");
        ClawR = hardwareMap.servo.get("ClawR");
        Lift = hardwareMap.dcMotor.get("Lift");
        ClawL.setDirection(Servo.Direction.REVERSE);
        Intake1 = hardwareMap.dcMotor.get("Intake");
        Intake2 = hardwareMap.dcMotor.get("Intake");
        mode = 0.0;
        Fleft.setDirection(DcMotorSimple.Direction.FORWARD);
        Bleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Whenever you're ready", "***");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            movement();

            Intake();

            Servos();

            Lift();

        }
    }

}
