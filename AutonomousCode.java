package org.firstinspires.ftc.robotcontroller.internal;
/* The Following codes have not been added as yet:
        The Colour sensor code
        The "Code to knock off the ball"
        The go on the balance pad
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="TeamACode", group="Pushbot")
@Disabled
public class AutonomousCode extends LinearOpMode {

    HardwarePushbot         robot   = new HardwarePushbot();
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     Forwardsp = 0.6;
    static final double     Turnsp    = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        robot.leftDrive.setPower(Turnsp);
        robot.rightDrive.setPower(-Turnsp);
        runtime.reset();

        //Turn the robot to the cypher
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            robot.leftDrive.setPower(-Turnsp);
            robot.rightDrive.setPower(Turnsp);
            runtime.reset();
        }
        robot.leftDrive.setPower(Forwardsp);
        robot.rightDrive.setPower(Forwardsp);
        runtime.reset();

        //Move the robot to the cypher so it can decode it
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            robot.leftDrive.setPower(Forwardsp);
            robot.rightDrive.setPower(Forwardsp);
        }

        //Drive Backwards for 1 Second
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            robot.leftDrive.setPower(-Forwardsp);
            robot.rightDrive.setPower(-Forwardsp);
            runtime.reset();
        }

        //Turning the robot to head for the correct column
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            robot.leftDrive.setPower(Forwardsp);
            robot.rightDrive.setPower(-Forwardsp);
        }

        /* Here an if statement will be place showing the relationship between the different cypher patterns
        so that the robot can know how long to move forward for to go to the location.
        The code comparing the cyphers will be place above the if statement in the colour sensor.
         */
        while (opModeIsActive() && (runtime.seconds() < 0))

        //This is the end used to stop at while on the balance pad
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
