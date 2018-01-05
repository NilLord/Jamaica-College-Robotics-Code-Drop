package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Colin Campbell on 1/5/2018.
 */

public class TeamB extends OpMode {

    DcMotor FmLeft, FmRight, BmLeft, BmRight, MLift;
    Double foward = 1.0, back = -1.0;

    @Override
    public void init() {
        // FMLeft = Front Motor Left
        FmLeft = (DcMotor) hardwareMap.get("FmLeft");
        FmRight = (DcMotor) hardwareMap.get("FmRight");
        BmLeft = (DcMotor) hardwareMap.get("BmLeft");
        BmRight = (DcMotor) hardwareMap.get("BmRight");
    }

    @Override
    public void loop() {
        // To move foward begins here
        if (gamepad1.left_stick_y >= 1 )
        {
            FmRight.setPower(foward);
            FmLeft.setPower(foward);
            BmRight.setPower(foward);
            BmLeft.setPower(foward);
        }
        // To Move foward ends here

        // To reverse begins here
        if (gamepad1.left_stick_y <= -1 )
        {
            FmRight.setPower(back);
            FmLeft.setPower(back);
            BmRight.setPower(back);
            BmLeft.setPower(back);
        }
        // To reverse ends here

        // To move right begins here
        if (gamepad1.left_stick_x >= 1)
        {
            FmLeft.setPower(foward);
            BmLeft.setPower(back);
            FmRight.setPower(back);
            BmRight.setPower(foward);
        }
        // To move right ends here

        // To move left begins here
        if (gamepad1.left_stick_x <= -1)
        {
            FmLeft.setPower(back);
            BmLeft.setPower(foward);
            FmRight.setPower(foward);
            BmRight.setPower(back);
        }
        // To move left ends here

        /*****************************/
        //  Combinations are used for diagonal movements //
        /// To  move foward right
        if (gamepad1.left_stick_y >= 1 && gamepad1.right_stick_x <= -1)
        {
            FmLeft.setPower(foward);
            BmRight.setPower(foward);
        }

        /// to move Backwards left
        if (gamepad1.left_stick_y <= -1 && gamepad1.right_stick_x >= 1)
        {
            FmLeft.setPower(back);
            BmRight.setPower(back);
        }

        // To move foward left
        if (gamepad1.left_stick_y >= 1 && gamepad1.right_stick_x <= -1)
        {
           FmRight.setPower(foward);
           BmLeft.setPower(foward);
        }

        // To move backwards left
        if (gamepad1.left_stick_y <= -1 && gamepad1.right_stick_x >= 1)
        {
            FmRight.setPower(foward);
            BmLeft.setPower(foward);
        }
        // rotate right
        if (gamepad1.right_bumper)
        {
            FmRight.setPower(back);
            BmRight.setPower(back);
            FmLeft.setPower(foward);
            BmLeft.setPower(foward);
        }

        // Rotate left
        if (gamepad1.left_bumper)
        {
            FmRight.setPower(foward);
            BmRight.setPower(foward);
            FmLeft.setPower(back);
            BmLeft.setPower(back);
        }

        ////// Lift mechanism
        if (gamepad1.dpad_up)
        {
            MLift.setPower(foward);
        }

        if (gamepad1.dpad_down)
        {
            MLift.setPower(back);
        }
    }

}
