package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by chava
 */
@Autonomous (name = "LocationMapping")
public class LocationMapping extends LinearOpMode {
    private DcMotor Fright, Fleft, Bright, Bleft;
    private Servo TailTop,TailBase, ClawL,ClawR;
    private ColorSensor Tip, Left,Right;
    private ElapsedTime runtime = new ElapsedTime();
    private enum Columns{Left, Center, Right}
    Columns Column;
    private enum Leg {CryptoLocate, ColumnLocate,Placing}
    Leg LEG;
    private enum Alliance{Red, Blue}
    Alliance Alliance;
    private Boolean CorrectColorRight, CorrectColorLeft, CorrectColorTip;
    private double R1,G1,B1,A1,
            R2,G2,B2,A2,
            R3,G3,B3,A3;

    @Override
    public void runOpMode() throws InterruptedException {
        Fright = hardwareMap.dcMotor.get("Fright");
        Fleft = hardwareMap.dcMotor.get("Fleft");
        Bright = hardwareMap.dcMotor.get("Bright");
        Bleft = hardwareMap.dcMotor.get("Bleft");
        TailBase = hardwareMap.servo.get("TailBase");
        TailTop = hardwareMap.servo.get("TailTop");
        Tip = hardwareMap.colorSensor.get("Tip");
        Left = hardwareMap.colorSensor.get("Left");
        Right = hardwareMap.colorSensor.get("Right");
        ClawL = hardwareMap.servo.get("ClawL");
        ClawR = hardwareMap.servo.get("ClawR");
        CorrectColorLeft = false;
        CorrectColorRight = false;
        CorrectColorTip = false;
        waitForStart();
        while (opModeIsActive()) {

            //TestLocate();
            Column = Columns.Left;
            CryptoLocate();
            //Rotate(1.0,200);
        }
    }


    private void TipMapper()
    {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double ScaleF = 255;
        runtime.reset();
        while (runtime.seconds() < 0.2){
            StopMovement();
            Color.RGBToHSV((int) (Tip.red() * ScaleF), (int) (Tip.green() * ScaleF), (int) (Tip.blue() * ScaleF), hsvValues);
            A3 = Tip.alpha();
            R3 = Tip.red();
            G3 = Tip.green();
            B3 = Tip.blue();
            telemetry.addData("Alpha tip", Tip.alpha());
            telemetry.addData("Red tip", Tip.red());
            telemetry.addData("Blue tip", Tip.blue());
            telemetry.addData("Green tip", Tip.green());
            telemetry.update();
            if (G3 < R3 || G3 < B3)
            {CorrectColorTip = true;}
            else{CorrectColorTip = false;}
        }
        runtime.reset();
        while (runtime.seconds() <0.3) {
            StopMovement();
            telemetry.addData("Bot", CorrectColorTip);
            telemetry.update();
        }
    }

    private void BackMappers()
    {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double ScaleF = 255;
        runtime.reset();
        while (runtime.seconds() < 0.2) {
            //Calculation of the RGB values of the sensors
            Color.RGBToHSV((int) (Right.red() * ScaleF), (int) (Right.green() * ScaleF), (int) (Right.blue() * ScaleF), hsvValues);
            Color.RGBToHSV((int) (Left.red() * ScaleF), (int) (Left.green() * ScaleF), (int) (Left.blue() * ScaleF), hsvValues);
            telemetry.addData("Right Alpha", Right.alpha());
            telemetry.addData("Left Alpha", Left.alpha());
            telemetry.addData("Right Red", Right.red());
            telemetry.addData("Left Red", Left.red());
            telemetry.addData("Right Green", Right.green());
            telemetry.addData("Left Green", Left.green());
            telemetry.addData("Right Blue", Right.blue());
            telemetry.addData("Left Blue", Left.blue());
            telemetry.update();
            A1 = Right.alpha();
            A2 = Left.alpha();
            R1 = Right.red();
            R2 = Left.red();
            G1 = Right.green();
            G2 = Left.green();
            B1 = Right.blue();
            B2 = Left.blue();

            if (G1 < R1 || G1 < B1) {CorrectColorRight = true;}
            if (G2 < R2 || G2 < B2 ) {CorrectColorLeft = true;}
            else {
                CorrectColorLeft = false;
                CorrectColorRight = false;
            }
            while (runtime.seconds() <0.3) {
                telemetry.addData("Bot", CorrectColorTip);
                telemetry.update();
            }
        }
    }

    private void CryptoReset(){
        CryptoLocate();
    }

    private void CrytoLocateP2()
    {
        while  (!CorrectColorRight || !CorrectColorTip){
            if (Column == Columns.Left) {
                BackMappers();
                if (CorrectColorRight && CorrectColorTip) {
                    break;
                }
                telemetry.addData("Bot", "Locating Left.....");
                telemetry.update();
                Strafe(0.3, 900);
                TipMapper();
                BackMappers();
            }
        }
        }

    private void CryptoLocate(){

        if (Column == Columns.Center) {
            BackMappers();
            // Locating the CryptoBox
            runtime.reset();
            do {
                if (CorrectColorRight || CorrectColorLeft){
                    break;
                }
                telemetry.addData("Bot", "Locating Center.....");
                telemetry.update();
                Strafe(0.3,900);
                BackMappers();
                if (runtime.seconds() >= 4.5 && !CorrectColorRight || !CorrectColorLeft) {
                    CryptoReset();
                }
            }
            while  (!CorrectColorRight || !CorrectColorLeft);
        }
        if (Column == Columns.Left) {
            BackMappers();
            // Locating the CryptoBox
            runtime.reset();
            do {
                if (CorrectColorRight && CorrectColorTip){
                    break;
                }
                telemetry.addData("Bot", "Locating Left.....");
                telemetry.update();
               Strafe(0.3,900);
                TipMapper();
                BackMappers();
            }
            while  (!CorrectColorRight || !CorrectColorTip);
        }
        if (Column == Columns.Right) {
            BackMappers();
            // Locating the CryptoBox
            runtime.reset();
            do {
                if (CorrectColorLeft || CorrectColorTip){
                    break;
                }
                telemetry.addData("Bot", "Locating Left.....");
                telemetry.update();
                Strafe(0.3,900);
                TipMapper();
                BackMappers();
                if (runtime.seconds() >= 3.5 && !CorrectColorLeft || !CorrectColorTip) {
                    CryptoReset();
                }
            }
            while  (!CorrectColorLeft || !CorrectColorTip);
        }
    }


    private void TestLocate()
    {
        TipMapper();
        runtime.reset();
        while (!CorrectColorTip)
        {
            ForwardMove(0.2,500);
            TipMapper();
            if (runtime.seconds() >= 4.5 && !CorrectColorTip) {
                runtime.reset();
            }
        }
        StopMovement();
        Rotate(0.3,1700);
        StopMovement();
    }

    private void ForwardMove(double Speed, long Time)
    {
        telemetry.addData("Bot","Forward/Backward");
        telemetry.update();
        Fright.setPower(-Speed);
        Fleft.setPower(-Speed);
        Bright.setPower(-Speed);
        Bleft.setPower(-Speed);
        sleep(Time);
    }

    private void StopMovement()
    {
        Fright.setPower(0);
        Fleft.setPower(0);
        Bright.setPower(0);
        Bleft.setPower(0);
    }



    public void Strafe(double Speed, long Time)
    {
        Fright.setPower(Speed);
        Fleft.setPower(-Speed);
        Bright.setPower(-Speed);
        Bleft.setPower(Speed);
        sleep(Time);
    }

    public void Rotate(double Speed, long Time)
    {
        Fright.setPower(Speed);
        Fleft.setPower(-Speed);
        Bright.setPower(Speed);
        Bleft.setPower(-Speed);
        sleep(Time);
    }

    public void LeftPointRotate(double Speed, long Time)
    {

    }

    public void RightPointRotate(double Speed, long Time)
    {

    }

    public void clawsopen ()
    {
        ClawL.setPosition(0.6);
        ClawR.setPosition(0.6);
    }

    public void clawsclose ()
    {
        ClawR.setPosition(0.41);
        ClawL.setPosition(0.41);//0.35
    }
}
