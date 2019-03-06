package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "encoders", group = "TeleOp")
//@Disabled
public class Encoders extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double GEAR_RATIO = 1.5;
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);

    private boolean there = false;

    public void runOpMode(){
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        waitForStart();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()) {
            telemetry.addData("Position:", leftMotor.getCurrentPosition());
            telemetry.update();
            leftMotor.setTargetPosition((int) (40 * COUNTS_PER_INCH));
            rightMotor.setTargetPosition((int) (40 * COUNTS_PER_INCH / 2));

            leftMotor.setPower(0.1);
            rightMotor.setPower(0.05);
            // testing

        }

    }
}
