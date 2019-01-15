package org.firstinspires.ftc.teamcode.organizedImplementation.Robot.Opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*

*Whenever code is updated, summarize what has been changed here*

1/14/19 - created base for code

 */

@TeleOp (name = "Main TeleOp", group = "TeleOp")
//@Disabled
public class MainTeleOp extends LinearOpMode {

    // teleop variables

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // mapping

        telemetry.addData("Status", "Mapping Complete");
        telemetry.update();

        // setting directions & initial positions

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {

        }
    }
}