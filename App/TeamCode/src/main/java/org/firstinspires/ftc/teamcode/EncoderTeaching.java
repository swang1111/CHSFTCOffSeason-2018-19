package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class EncoderTeaching extends LinearOpMode {

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    private static final double COUNTS_PER_MOTOR_REV = 512;
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_INCHES = 4;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);

    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {

            int newLeftDistance = leftMotor.getCurrentPosition() + (int) (48 * COUNTS_PER_INCH);
            int newRightDistance = rightMotor.getCurrentPosition() + (int) (48 * COUNTS_PER_INCH);

            leftMotor.setTargetPosition(newLeftDistance);
            rightMotor.setTargetPosition(newRightDistance);

            leftMotor.setPower(1);
            rightMotor.setPower(1);

        }

    }

}
