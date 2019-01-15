package org.firstinspires.ftc.teamcode.organizedImplementation.Robot.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

/*

*Whenever code is updated, summarize what has been changed here*

1/14/19 - created base for code

 */

@Autonomous (name = "Depot Autonomous", group = "Autonomous")
//@Disabled
public class DepotAutonomous extends LinearOpMode {

    // autonomous variables

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // mapping

        telemetry.addData("Status", "Mapping Complete");
        telemetry.update();

        // setting directions & initial positions

        waitForStart();
        runtime.reset();

        // start

        /*

        // example

        rightMotor.setPower(rightSpeed);
        leftMotor.setPower(leftSpeed);
        while(opModeIsActive() && runtime.seconds() < 0.2){
            telemetry.addData("Status", "forward unlatching movement", runtime.seconds());
            telemetry.update();
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        sleep(500);

        runtime.reset();

         */

        telemetry.addData("Status", "Autonomous Finished", runtime.seconds());
        telemetry.update();
    }
}