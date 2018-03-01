package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

/**
 * Created by chava on 21/02/2018.
 *
 * Official Team 3981 - Autonomous Code for the Relic Recovery Challenge 2018
 */


@Autonomous (name = "RevAuto", group = "Regionals")
public class RevAuto extends LinearOpMode {


    DcMotor Fright,Fleft,Bright,Bleft, Lift, Ext;
    Servo ClawR,ClawL,RelicGrab,JewelPunch;
    int  Coloumn, TeamColour, JewelColour;
    Boolean Sensing;
    public static final String TAG = "Vuforia Navigation Sample";
    int team1r,team1b,team1g,team2r,team2b,team2g;
    double tX; // X value extracted from our the offset of the traget relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    private ElapsedTime runtime =new ElapsedTime();
    double rX; // X value extracted from the rotational components of the tartget relitive to the robot
    double rY; // Same as above but for Y
    double rZ;
    VuforiaLocalizer vuforia; // Cypher
    ColorSensor Jewel, Team1, Team2;

    @Override
    public void runOpMode() throws InterruptedException {

        Fright = hardwareMap.dcMotor.get("Fright");
        Fleft = hardwareMap.dcMotor.get("Fleft");
        Bright = hardwareMap.dcMotor.get("Bright");
        Bleft = hardwareMap.dcMotor.get("Bleft");
        JewelPunch = hardwareMap.servo.get("JewelPunch");
        ClawL = hardwareMap.servo.get("ClawL");
        ClawR = hardwareMap.servo.get("ClawR");
        Lift = hardwareMap.dcMotor.get("Lift");
        Fleft.setDirection(DcMotorSimple.Direction.FORWARD);
        Bleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        ClawL.setDirection(Servo.Direction.REVERSE);
        Jewel = hardwareMap.get(ColorSensor.class, "Jewel");
        Team1 = hardwareMap.get(ColorSensor.class, "Team1");
        //Team Colours 1 = Red
        //             2 = Blue

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double ScaleF = 255;



        telemetry.addData("Robot", "Ready");
        telemetry.update();
        waitForStart();
        clawsclose();
        Lift.setPower(-0.5);
        sleep(330);
        Lift.setPower(0.0);
        JewelPunch.setPosition(0.9);
        Jewel.enableLed(true);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            Color.RGBToHSV((int) (Team1.red() * ScaleF), (int) (Team1.green() * ScaleF), (int) (Team1.blue() * ScaleF), hsvValues);
            telemetry.addData("Alpha", Team1.alpha());
            telemetry.addData("Red", Team1.red());
            telemetry.addData("Green", Team1.green());
            telemetry.addData("Blue", Team1.blue());
            telemetry.update();
            team1r = Team1.red();
            team1b = Team1.blue();
            team1g = Team1.green();
        }
        if (team1r > team1b && team1r > team1g) {
            TeamColour = 1;
        } else if (team1b > team1r && team1b > team1g) {
            TeamColour = 2;
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Team Colour", TeamColour);
            telemetry.update();
        }
        Jewel();
        if (TeamColour == 1) {
            ForwardMove(-0.5, 650);
            JewelPunch.setPosition(0.2);
            Turnleft(1.0,492);
            ForwardMove(0.3,650);
            clawsopen();
            ForwardMove(-0.2,100);
        } else{
            ForwardMove(0.5, 650);
        JewelPunch.setPosition(0.2);
            Turnleft(1.0,492);
            ForwardMove(0.3,650);
            clawsopen();
            ForwardMove(-0.2,100);
    }
        sleep(600);
        ForwardMove(0.2,900);

       /* runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 4.5)
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AZ6iROb/////AAAAmbUjFCICnExkvCKlbfObnYaOwEvmr1RdpVLJTf2eUj7f+U+o1ZWfoh7HntvIglNem/JufobpZAS7Qymzxr6PIb1oNIU7KXs6N2yFi1JZKMRLzNyjbcv55dJJ5OqT86eJZgWAiGN4cUDHayWtv/3Cy4piSoEYWL40dJ18w5pFAiyg3A5fgUYUUIFg8FP4yRy1nwAqyUkO3/EYUzQ7YWTkyv98y3wnvMLS3BzGwkV8Lot09bA0KISw7aXASLg3J+GQad6DIqqrvrLqgUP/Bk/mXATFoIpr1h97O0RzdEdCEshYer92PTSvi3CkmBgwRwrQFsOYwKFWu2xhZo1WpNXTXuDLuyLq9Ic3IL1N+Sz49CZk";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");

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
                    Coloumn = 1;
                    placement();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
                    telemetry.addData("VuMark is", "Right");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    Coloumn = 3;
                    placement();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
                    telemetry.addData("VuMark is", "Center");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    Coloumn = 2;
                    placement();
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }*/


    }
//Functions used in the code to either move the robot, stop the robot or operate the servos.
    public void ForwardMove(double Power, long Time)
    {
        telemetry.addData("Movement","Forward/Backward");
        telemetry.update();
        Fright.setPower(Power);
        Fleft.setPower(-Power);
        Bright.setPower(Power);
        Bleft.setPower(-Power);
        sleep(Time);
    }

    public void LeftHorizontal(double Power, long Time)
    {
        telemetry.addData("Robot","Strafe");
        telemetry.update();
        Fright.setPower(Power);
        Fleft.setPower(Power);
        Bright.setPower(-Power);
        Bleft.setPower(-Power);
        sleep(Time);
    }

    public void RightHorizontal (double Power, long Time)
    {
        telemetry.addData("Robot","Strafe");
        telemetry.update();
        Fright.setPower(-Power);
        Fleft.setPower(-Power);
        Bright.setPower(Power);
        Bleft.setPower(Power);
        sleep(Time);
    }

    public void Turnright(double Power, long Time)
    {
        Fright.setPower(-Power);
        Fleft.setPower(-Power);
        Bright.setPower(-Power);
        Bleft.setPower(-Power);
        sleep(Time);
        telemetry.addData("Robot","Turning");
        telemetry.update();
    }

    public void Turnleft(double Power, long Time)
    {
        telemetry.addData("Robot","Turning");
        telemetry.update();
        Fright.setPower(Power);
        Fleft.setPower(Power);
        Bright.setPower(Power);
        Bleft.setPower(Power);
        sleep(Time);
    }

    public void Stopmovement()
    {
        telemetry.addData("Robot","Stop");
        telemetry.update();
        Fright.setPower(0.0);
        Fleft.setPower(0.0);
        Bright.setPower(0.0);
        Bleft.setPower(0.0);
    }

    public void clawsopen ()
    {
        ClawL.setPosition(0.55);
        ClawR.setPosition(0.55);//0.5
    }


    public void Jewel()
    {

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double ScaleF = 255;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <1.5) {
            Color.RGBToHSV((int) (Jewel.red() * ScaleF), (int) (Jewel.green() * ScaleF), (int) (Jewel.blue() * ScaleF), hsvValues);
            team2r = Jewel.red();
            team2b = Jewel.blue();
            team2g = Jewel.green();
        }
        if (team2r > team2b && team2r > team2g)
        {
            JewelColour = 2;
        }else
        if (team2b > team2r && team2b > team2g)
        {
            JewelColour = 1;
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <1.1) {
            telemetry.addData("Jewel Colour", JewelColour);
            telemetry.update();
        }

        if (TeamColour == 1 && JewelColour == 1)
        {
            Turnright(0.2,750);
            JewelPunch.setPosition(0.2);
            Turnleft(0.2, 750);

        }else if (TeamColour == 2 && JewelColour == 1 || TeamColour == 1 && JewelColour == 2)
        {
            telemetry.addData("Function","Else");
            telemetry.update();
            Turnleft(0.2, 750);
            JewelPunch.setPosition(0.2);
            Turnright(0.2, 750);
        }
    }

    public void clawsclose ()
    {
        ClawR.setPosition(0.41);
        ClawL.setPosition(0.41);//0.35
    }

    public void placement()
    {
        if (Coloumn == 1)
        {
            if (TeamColour == 1) {
                ForwardMove(-0.5, 650);
                JewelPunch.setPosition(0.2);
                Turnright(1.0,492);
                ForwardMove(0.3,650);
                clawsopen();
                ForwardMove(-0.2,100);
            } else{
                ForwardMove(0.5, 650);
                JewelPunch.setPosition(0.2);
                Turnleft(1.0,492);
                ForwardMove(0.3,650);
                clawsopen();
                ForwardMove(-0.2,100);
            }
        }
        if (Coloumn == 2)
        {
            if (TeamColour == 1) {
                ForwardMove(-0.5, 750);
                JewelPunch.setPosition(0.2);
                Turnright(1.0,492);
                ForwardMove(0.3,650);
                clawsopen();
                ForwardMove(-0.2,100);
            } else{
                ForwardMove(0.5, 750);
                JewelPunch.setPosition(0.2);
                Turnleft(1.0,492);
                ForwardMove(0.3,650);
                clawsopen();
                ForwardMove(-0.2,100);
            }
        }
        if (Coloumn == 3)
        {
            if (TeamColour == 1) {
                ForwardMove(-0.5, 850);
                JewelPunch.setPosition(0.2);
                Turnright(1.0,492);
                ForwardMove(0.3,850);
                clawsopen();
                ForwardMove(-0.2,100);
            } else{
                ForwardMove(0.5, 850);
                JewelPunch.setPosition(0.2);
                Turnleft(1.0,492);
                ForwardMove(0.3,850);
                clawsopen();
                ForwardMove(-0.2,100);
            }
        }
    }


}
