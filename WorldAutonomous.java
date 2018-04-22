package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TestPhoneJewel;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "World Autonomous",group = "Autonomous")
public class WorldAutonomous extends LinearOpMode {
    private DcMotor Fright, Fleft, Bright, Bleft,Intake1,Intake2,Lift;
    private Servo TailTop,TailBase, ClawL,ClawR,Cubepush;
    private ColorSensor Tip, Left,Right;
    private enum Columns{Left, Center, Right}
                Columns Column;
    private enum Leg {Pad,FirstCube,CryptoLocate, SecondCube, end}
                Leg leg;
    private enum Alliances{Red, Blue}
                Alliances Alliance;
    private Boolean CorrectColorRight, CorrectColorLeft, CorrectColorTip;
    private double R1,G1,B1,A1,
            R2,G2,B2,A2,
            R3,G3,B3,A3;
    public static final String TAG = "Vuforia Navigation Sample";
    double tX; // X value extracted from our the offset of the target relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    private ElapsedTime runtime =new ElapsedTime();
    double rX; // X value extracted from the rotational components of the target relative to the robot
    double rY; // Same as above but for Y
    double rZ;
    VuforiaLocalizer vuforia; // Cypher
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    int ColourValue;
    private TestPhoneJewel PhoneJewel;

    @Override
    public void runOpMode() {
        /*
         * Motors
         */
        Fright = hardwareMap.dcMotor.get("Fright");
        Fleft = hardwareMap.dcMotor.get("Fleft");
        Bright = hardwareMap.dcMotor.get("Bright");
        Bleft = hardwareMap.dcMotor.get("Bleft");
        Fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fleft .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bleft .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fright.setDirection(DcMotorSimple.Direction.FORWARD);
        Bright.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake1 = hardwareMap.dcMotor.get("Intake1");
        Intake2 = hardwareMap.dcMotor.get("Intake2");
        Lift = hardwareMap.dcMotor.get("Lift");
        /*
         * Servos
         */
        ClawL = hardwareMap.servo.get("ClawL");
        ClawR = hardwareMap.servo.get("ClawR");
        TailBase = hardwareMap.servo.get("TailBase");
        TailTop = hardwareMap.servo.get("TailTop");
        ClawL.setDirection(Servo.Direction.REVERSE);
        Cubepush = hardwareMap.servo.get("CubePush");
        /*
        * Sensors
         */
        Tip = hardwareMap.colorSensor.get("Tip");
        Left = hardwareMap.colorSensor.get("Left");
        Right = hardwareMap.colorSensor.get("Right");
        /*
        *Vuforia
         */
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZ6iROb/////AAAAmbUjFCICnExkvCKlbfObnYaOwEvmr1RdpVLJTf2eUj7f+U+o1ZWfoh7HntvIglNem/JufobpZAS7Qymzxr6PIb1oNIU7KXs6N2yFi1JZKMRLzNyjbcv55dJJ5OqT86eJZgWAiGN4cUDHayWtv/3Cy4piSoEYWL40dJ18w5pFAiyg3A5fgUYUUIFg8FP4yRy1nwAqyUkO3/EYUzQ7YWTkyv98y3wnvMLS3BzGwkV8Lot09bA0KISw7aXASLg3J+GQad6DIqqrvrLqgUP/Bk/mXATFoIpr1h97O0RzdEdCEshYer92PTSvi3CkmBgwRwrQFsOYwKFWu2xhZo1WpNXTXuDLuyLq9Ic3IL1N+Sz49CZk";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        /*
         *General
         */
        CorrectColorLeft = false;
        CorrectColorRight = false;
        CorrectColorTip = false;


        telemetry.addData("Initialized","");
        telemetry.update();
        waitForStart();
        ColourValue = PhoneJewel.Vision();
        VuMark();
        AllianceDection();
        leg = Leg.Pad;
        while (opModeIsActive()){
            JewelTailBaseandTop();
            if (leg == Leg.Pad){
                    close();
                   // Lift.setPower(-1);
                   // sleep(500);
                    ForwardMove(.3,1100);
                    leg = Leg.FirstCube;
            }
                else if (leg == Leg.FirstCube) {
                        Rotate(1,-950);
                        ForwardMove(0.3,1120);
                        leg = Leg.CryptoLocate;
                }
                    else if (leg == Leg.CryptoLocate){
                        CryptoLocate();
                       // TipStrafe();

                    }
                        else if (leg == Leg.end){
                            requestOpModeStop();
                        }
        }

    }
//Forward move with and without Encoders
    private void ForwardMove(double Speed,int distance)
    {
        ResetEnc();
        RuntoEnc();
        Fright.setTargetPosition(-distance);
        Fleft .setTargetPosition(distance);
        Bright.setTargetPosition(-distance);
        Bleft .setTargetPosition(distance);
        Fright.setPower(Speed);
        Fleft.setPower(Speed);
        Bright.setPower(Speed);
        Bleft.setPower(Speed);
        MotorBusy();
    }
    private void ForwardMoveNoEnc(double Speed, long Time)
    {
        Fright.setPower(-Speed);
        Fleft.setPower(-Speed);
        Bright.setPower(-Speed);
        Bleft.setPower(-Speed);
        sleep(Time);
    }
//Encoder functions (Reset, run to position)
    private void ResetEnc(){
        Fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fleft .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bleft .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void RuntoEnc(){
        Fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fleft .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bleft .setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
//stop all motor function
    private void StopBot(){
        Fright.setPower(0);
        Fleft.setPower(0);
        Bright.setPower(0);
        Bleft.setPower(0);
        sleep(300);
    }
//Strafe
    private void Strafe(double Speed, int distance)
    {
        ResetEnc();
        RuntoEnc();
        Fright.setTargetPosition(-distance);
        Fleft .setTargetPosition(-distance);
        Bright.setTargetPosition(distance);
        Bleft .setTargetPosition(distance);
        Fright.setPower(-Speed);
        Fleft.setPower(-Speed);
        Bright.setPower(Speed);
        Bleft.setPower(Speed);
        MotorBusy();
    }

    public void StrafewithoutEnc(double Speed, long time){
        Fright.setPower(-Speed);
        Fleft.setPower(-Speed);
        Bright.setPower(Speed);
        Bleft.setPower(Speed);
    }
//Motor Busy For encoders to run until position
    private void MotorBusy(){
        while (Fright.isBusy() && Fleft.isBusy() && Bright.isBusy() && Bleft.isBusy()){
            Bleft.getCurrentPosition();
            telemetry.addData("Front Right",Fright.getCurrentPosition());
            telemetry.addData("Front Left", Fleft .getCurrentPosition());
            telemetry.addData("Back Right",Bright.getCurrentPosition());
            telemetry.addData("Back Left",Bleft.getCurrentPosition());
            telemetry.update();
        }
        StopBot();
    }
//Rotate with and without encoders
    private void Rotate(double Speed, int distance)
    {
        ResetEnc();
        RuntoEnc();
        Fright.setTargetPosition(distance);
        Fleft .setTargetPosition(distance);
        Bright.setTargetPosition(distance);
        Bleft .setTargetPosition(distance);
        Fright.setPower(Speed);
        Fleft.setPower(Speed);
        Bright.setPower(Speed);
        Bleft.setPower(Speed);
        MotorBusy();
    }

    private void RotateNoEnc(double Speed, long Time)
    {
        Fright.setPower(Speed);
        Fleft.setPower(-Speed);
        Bright.setPower(Speed);
        Bleft.setPower(-Speed);
        sleep(Time);
    }
//Servos
    private void close() {
        ClawL.setPosition(0.51);
        ClawR.setPosition(0.48);
    }

    private void open() {
        ClawR.setPosition(0.59);
        ClawL.setPosition(0.56);
    }
//Color Senser
    private void BackMappers()
    {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double ScaleF = 255;
        //Calculation of the RGB values of the sensors
        Color.RGBToHSV((int) (Right.red() * ScaleF), (int) (Right.green() * ScaleF), (int) (Right.blue() * ScaleF), hsvValues);
        Color.RGBToHSV((int) (Left.red() * ScaleF), (int) (Left.green() * ScaleF), (int) (Left.blue() * ScaleF), hsvValues);
        A1 = Right.alpha();
        A2 = Left.alpha();
        R1 = Right.red();
        R2 = Left.red();
        G1 = Right.green();
        G2 = Left.green();
        B1 = Right.blue();
        B2 = Left.blue();

        if (G1 < R1 || G1 < B1) {CorrectColorRight = true;}
        else if (G2 < R2 || G2 < B2 ) {CorrectColorLeft = true;}
        else {
            CorrectColorLeft = false;
            CorrectColorRight = false;
        }
    }

    private void TipMapper()
    {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double ScaleF = 255;
        Color.RGBToHSV((int) (Tip.red() * ScaleF), (int) (Tip.green() * ScaleF), (int) (Tip.blue() * ScaleF), hsvValues);
        A3 = Tip.alpha();
        R3 = Tip.red();
        G3 = Tip.green();
        B3 = Tip.blue();

        if (G3 < R3 || G3 < B3)
        {CorrectColorTip = true;}
            else{CorrectColorTip = false;}
    }

    private void AllianceDection()
    {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double ScaleF = 255;
        Color.RGBToHSV((int) (Tip.red() * ScaleF), (int) (Tip.green() * ScaleF), (int) (Tip.blue() * ScaleF), hsvValues);
        A3 = Tip.alpha();
        R3 = Tip.red();
        G3 = Tip.green();
        B3 = Tip.blue();

        if (G3 < R3 && B3<R3) {
            Alliance = Alliances.Red;
        }
        if( G3 < B3 && B3<R3) {
            Alliance = Alliances.Blue;
        }

    }
//Vuforia
    private void VuMark()
    {

        relicTrackables.activate(); // Activate Vuforia
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                tX = trans.get(0);
                tY = trans.get(1);
                tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot. NOTE: VERY IMPORTANT IF BASING MOVEMENT OFF OF THE IMAGE!!!!
                rX = rot.firstAngle;
                rY = rot.secondAngle;
                rZ = rot.thirdAngle;
            }
            if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);
                Column = Columns.Left;

            } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);
                Column = Columns.Right;

            } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);
                Column = Columns.Center;

            }
        } else {
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.update();
    }
//Locating Cryptobox
    private void CryptoLocate(){

        if (Column == Columns.Left) {
            TipMapper();
            BackMappers();
            // Locating the CryptoBox
            runtime.reset();
            if (!CorrectColorRight || !CorrectColorTip) {
                telemetry.addData("Right", CorrectColorRight);
                telemetry.addData("Tip", CorrectColorTip);
                telemetry.update();
                StrafewithoutEnc(0.35,200);
            }
                 if (CorrectColorTip)
                     {
                         ForwardMove(0.3,200);
                         open();
                         Intake1.setPower(1.0);
                         Intake2.setPower(1.0);
                         Cubepush.setPosition(0.1);
                         ForwardMove(0.2,-250);
                         sleep(200);
                         Cubepush.setPosition(0.56);
                         StopBot();
                         requestOpModeStop();
                         leg = Leg.end;
                    }
        }
        if (Column == Columns.Right){
            TipMapper();
            BackMappers();
            // Locating the CryptoBox
            runtime.reset();
            if (!CorrectColorLeft || !CorrectColorTip) {
                telemetry.addData("Left", CorrectColorLeft);
                telemetry.addData("Tip", CorrectColorTip);
                telemetry.update();
                StrafewithoutEnc(0.35,200);
            }
            if (CorrectColorRight && CorrectColorTip)
            {
                ForwardMove(0.3,200);
                open();
                Intake1.setPower(1.0);
                Intake2.setPower(1.0);
                Cubepush.setPosition(0.1);
                ForwardMove(0.2,-250);
                sleep(200);
                Cubepush.setPosition(0.56);
                StopBot();
                requestOpModeStop();
                leg = Leg.end;
            }
        }
        if (Column == Columns.Center){
            BackMappers();
            // Locating the CryptoBox
            runtime.reset();
            if (!CorrectColorLeft || !CorrectColorRight) {
                telemetry.addData("Left", CorrectColorLeft);
                telemetry.addData("Right", CorrectColorRight);
                telemetry.update();
                StrafewithoutEnc(0.35,200);
            }
            if (CorrectColorRight && CorrectColorTip)
            {
                ForwardMove(0.3,200);
                open();
                Intake1.setPower(1.0);
                Intake2.setPower(1.0);
                Cubepush.setPosition(0.1);
                ForwardMove(0.2,-250);
                sleep(200);
                Cubepush.setPosition(0.56);
                StopBot();
                requestOpModeStop();
                leg = Leg.end;
            }
        }
    }
//Jewel Knockoff
    private void JewelTailBaseandTop(){
        TailBase.setPosition(0.5);
        TailTop.setPosition(0.5);
        if (ColourValue == 1 && Alliance == Alliances.Red) {
            TailTop.setPosition(0.0);
            sleep(100);
            TailBase.setPosition(.5);
        }
            else if (ColourValue == 2 && Alliance == Alliances.Blue)
            {
                TailTop.setPosition(0.0);
                sleep(100);
                TailBase.setPosition(.5);
            }
                else{
                     TailTop.setPosition(0.0);
                     sleep(100);
                     TailBase.setPosition(1);
        }
        TailBase.setPosition(0.5);
        TailTop.setPosition(0.5);
    }

}
