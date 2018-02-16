package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by chava on 05/02/2018.
 */

@TeleOp (name = "TeamAGaza", group = "First")
public class TeamAGaza extends OpMode{

    DcMotor M1, M2, M3, M4, Lift, Extension;
    Servo S1,S2,S3,S4;
    double Xinput, Yinput, M1Power, M2Power, M3Power, M4Power;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
// Certain details are commented out due to the condition of the bot
        telemetry.addData("Status:","See it yah");
        telemetry.addData("Status:","Yo it ready yere");
        telemetry.update();

        M1 = hardwareMap.dcMotor.get("M1");
        M2 = hardwareMap.dcMotor.get("M2");
        M3 = hardwareMap.dcMotor.get("M3");
        M4 = hardwareMap.dcMotor.get("M4");
        //Extension = hardwareMap.dcMotor.get("Ext");
        Lift = hardwareMap.dcMotor.get("Lift");
        S1 = hardwareMap.servo.get("S1");
        S2 = hardwareMap.servo.get("S2");
        S3 = hardwareMap.servo.get("S3");
        S4 = hardwareMap.servo.get("S4");
        S2.setDirection(Servo.Direction.REVERSE);
        S4.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void start(){
        telemetry.addData("Status:","Vroom Vroom Bredda");
        telemetry.addData("Pos1:",S1.getPosition());
        telemetry.addData("Pos2:",S2.getPosition());
        telemetry.addData("Pos3:",S3.getPosition());
        telemetry.addData("Pos4:",S4.getPosition());
        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {

        Movement();

        ServoClaw();

        ServoUpDown();

        //Ext();

        Lifting();
    }
    @Override
    public void stop()
    {
        M1.setPower(0.0);
        M2.setPower(0.0);
        M3.setPower(0.0);
        M4.setPower(0.0);
    }

    public void Movement()
    {
        Xinput = gamepad1.left_stick_x;
        Yinput = gamepad1.left_stick_y;
        // Little math and a Range setter to make sure that the wheels get the correct value and move ergonomically.
        M1Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

        M2Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

        M3Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

        M4Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

        M1.setPower(M1Power);
        M2.setPower(M4Power);
        M3.setPower(M3Power);
        M4.setPower(M2Power);

        runtime.reset();
        //Resetting the runtime to make sure that the while loop runs efficiently.
        while (gamepad1.right_stick_x != 0  && (runtime.seconds() < 4.5))
        {
            M1.setPower(-gamepad1.right_stick_x);
            M2.setPower(-gamepad1.right_stick_x);
            M3.setPower(gamepad1.right_stick_x);
            M4.setPower(gamepad1.right_stick_x);
        }
        runtime.reset();
    }

    public void ServoClaw()
    {
        if (gamepad2.x)
        {
            S1.setPosition(0.3);
            S2.setPosition(0.3);
            telemetry.addData("Pos:",S1.getPosition());
            telemetry.addData("Pos:",S2.getPosition());
            telemetry.update();
        }
        if (gamepad2.a)
        {
            S1.setPosition(0.0);
            S2.setPosition(0.0);
            telemetry.addData("Pos:",S1.getPosition());
            telemetry.addData("Pos:",S2.getPosition());
            telemetry.update();
        }
    }

   /** public void Ext()
    {
        runtime.reset();
        while (gamepad1.left_trigger >= 1 && gamepad2.b && (runtime.seconds() < 4.5))
        {
            Extension.setPower(0.3);
        }
        runtime.reset();
        while (gamepad1.right_trigger >= 1 && gamepad2.y && (runtime.seconds() < 4.5))
        {
            Extension.setPower(-0.3);
        }
        runtime.reset();
    }
    */

    public void ServoUpDown()
    {
        if (gamepad2.left_bumper)
        {
            runtime.reset();
            while ((runtime.seconds() < 0.6)) {
                S3.setPosition(1.0);
                S4.setPosition(1.0);
                Xinput = gamepad1.left_stick_x;
                Yinput = gamepad1.left_stick_y;
                // Little math and a Range setter to make sure that the wheels get the correct value and move ergonomically.
                M1Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

                M2Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

                M3Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

                M4Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

                M1.setPower(M1Power);
                M2.setPower(M4Power);
                M3.setPower(M3Power);
                M4.setPower(M2Power);
            }
            S3.setPosition(0.5);
            S4.setPosition(0.5);
        }
        else if (gamepad2.right_bumper)
        {
            runtime.reset();
            while ((runtime.seconds() < 0.6)) {
                S3.setPosition(0.0);
                S4.setPosition(0.0);
                Xinput = gamepad1.left_stick_x;
                Yinput = gamepad1.left_stick_y;
                // Little math and a Range setter to make sure that the wheels get the correct value and move ergonomically.
                M1Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

                M2Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

                M3Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

                M4Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

                M1.setPower(M1Power);
                M2.setPower(M4Power);
                M3.setPower(M3Power);
                M4.setPower(M2Power);
            }
            S3.setPosition(0.5);
            S4.setPosition(0.5);
        }
    }
    public void Lifting()
    {
        while (gamepad2.dpad_up && runtime.seconds() < 4.5)
        {
            Lift.setPower(0.6);
            telemetry.addData("Yo", "Watch out it might pop");
            telemetry.update();
            Xinput = gamepad1.left_stick_x;
            Yinput = gamepad1.left_stick_y;
            // Little math and a Range setter to make sure that the wheels get the correct value and move ergonomically.
            M1Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

            M2Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

            M3Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

            M4Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;
            M1.setPower(M1Power);
            M2.setPower(M4Power);
            M3.setPower(M3Power);
            M4.setPower(M2Power);
        }
        while (gamepad2.dpad_down && runtime.seconds() < 4.5)
        {
            Lift.setPower(-0.6);
            telemetry.addData("Yo", "Watch out it might pop");
            telemetry.update();
            Xinput = gamepad1.left_stick_x;
            Yinput = gamepad1.left_stick_y;
            // Little math and a Range setter to make sure that the wheels get the correct value and move ergonomically.
            M1Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

            M2Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

            M3Power   = Range.clip(Yinput + (-Xinput), -0.7, 0.7) ;

            M4Power   = Range.clip(Yinput + Xinput, -0.7, 0.7) ;

            M1.setPower(M1Power);
            M2.setPower(M4Power);
            M3.setPower(M3Power);
            M4.setPower(M2Power);
        }
        runtime.reset();
        Lift.setPower(0.0);
    }
}

