package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
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

@Autonomous(name="TeamACode", group="Pushbot")
//@Disabled
public class AutonomousCode extends LinearOpMode {
    DcMotor M1,M2,M3,M4;
    private ElapsedTime     runtime = new ElapsedTime();
    public static final String TAG = "Vuforia Navigation Sample";
    double tX; // X value extracted from our the offset of the traget relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    // -----------------------------------
    double rX; // X value extracted from the rotational components of the tartget relitive to the robot
    double rY; // Same as above but for Y
    double rZ; // Same as above but for Z

    VuforiaLocalizer vuforia;
    static final double     Forwardsp = 0.6;
    static final double     Turnsp    = 0.5;
    public ModernRoboticsI2cColorSensor bottomsenser = null;
    @Override
    public void runOpMode() {

        bottomsenser = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "TeamSense");
        M1 = hardwareMap.dcMotor.get("M1");
        M2 = hardwareMap.dcMotor.get("M2");
        M3 = hardwareMap.dcMotor.get("M3");
        M4 = hardwareMap.dcMotor.get("M4");
        M2.setDirection(DcMotorSimple.Direction.REVERSE);
        M4.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();

        //
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            M1.setPower(Forwardsp);
            M2.setPower(Forwardsp);
            M3.setPower(Forwardsp);
            M4.setPower(Forwardsp);
        }

        //Turn the robot to the cypher
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            M1.setPower(-Turnsp);
            M2.setPower(Turnsp);
            M3.setPower(Turnsp);
            M4.setPower(-Turnsp);
        }

        runtime.reset();

        //Move the robot to the cypher so it can decode it
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            M1.setPower(Forwardsp);
            M2.setPower(Forwardsp);
            M3.setPower(Forwardsp);
            M4.setPower(Forwardsp);
        }
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            M1.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            M4.setPower(0);
            camera();
        }

        //Drive Backwards
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            M1.setPower(-Turnsp);
            M2.setPower(Turnsp);
            M3.setPower(Turnsp);
            M4.setPower(-Turnsp);
            sleep(3000);
            M1.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            M4.setPower(0);
        }
        runtime.reset();

        /* Here an if statement will be place showing the relationship between the different cypher patterns
        so that the robot can know how long to move forward for to go to the location.
        The code comparing the cyphers will be place above the if statement in the colour sensor.
         */
        while (opModeIsActive() && (runtime.seconds() < 0.8))

            //This is the end used to stop at while on the balance pad
            M1.setPower(0);
        M2.setPower(0);
        M3.setPower(0);
        M4.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }


    public void left()
    {
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            M1.setPower(Forwardsp);
            M2.setPower(Forwardsp);
            M3.setPower(Forwardsp);
            M4.setPower(Forwardsp);
        }
        M1.setPower(0);
        M2.setPower(0);
        M3.setPower(0);
        M4.setPower(0);
    }

    public void right()
    {
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            M1.setPower(Forwardsp);
            M2.setPower(Forwardsp);
            M3.setPower(Forwardsp);
            M4.setPower(Forwardsp);
        }
        M1.setPower(0);
        M2.setPower(0);
        M3.setPower(0);
        M4.setPower(0);
    }

    public void center()
    {
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            M1.setPower(Forwardsp);
            M2.setPower(Forwardsp);
            M3.setPower(Forwardsp);
            M4.setPower(Forwardsp);
        }
        M1.setPower(0);
        M2.setPower(0);
        M3.setPower(0);
        M4.setPower(0);
    }

    public void camera ()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        waitForStart();

        relicTrackables.activate(); // Activate Vuforia

        while (opModeIsActive()) {
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
                    left();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
                    telemetry.addData("VuMark is", "Right");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    right();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
                    telemetry.addData("VuMark is", "Center");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    center();
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
    }

    protected void coloursense() throws InterruptedException
    {
        bottomsenser.enableLed(true);
    }
}