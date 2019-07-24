package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "encoders", group = "TeleOp")
//@Disabled
public class Encoders extends LinearOpMode {

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private static final double COUNTS_PER_MOTOR_REV = 288;
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_INCHES = 4;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);

    private boolean there = false;

    public void runOpMode(){

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        tr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        tl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(opModeIsActive()) {



            if(gamepad1.y) {
                tl_motor.setTargetPosition(tl_motor.getCurrentPosition() + (int) (48 * COUNTS_PER_INCH));
                tr_motor.setTargetPosition(tr_motor.getCurrentPosition() + (int) (48 * COUNTS_PER_INCH));
                bl_motor.setTargetPosition(bl_motor.getCurrentPosition() + (int) (48 * COUNTS_PER_INCH));
                br_motor.setTargetPosition(br_motor.getCurrentPosition() + (int) (48 * COUNTS_PER_INCH));
                tl_motor.setPower(1);
                tr_motor.setPower(1);
                bl_motor.setPower(1);
                br_motor.setPower(1);
            }else if (gamepad1.b) {
                tl_motor.setTargetPosition(tl_motor.getCurrentPosition() - (int) (48 * COUNTS_PER_INCH));
                tr_motor.setTargetPosition(tr_motor.getCurrentPosition() + (int) (48 * COUNTS_PER_INCH));
                bl_motor.setTargetPosition(bl_motor.getCurrentPosition() + (int) (48 * COUNTS_PER_INCH));
                br_motor.setTargetPosition(br_motor.getCurrentPosition() - (int) (48 * COUNTS_PER_INCH));
                tl_motor.setPower(1);
                tr_motor.setPower(1);
                bl_motor.setPower(1);
                br_motor.setPower(1);
            }else if (gamepad1.a) {
                tl_motor.setTargetPosition(tl_motor.getCurrentPosition() + (int) (-48 * COUNTS_PER_INCH));
                tr_motor.setTargetPosition(tr_motor.getCurrentPosition() + (int) (-48 * COUNTS_PER_INCH));
                bl_motor.setTargetPosition(bl_motor.getCurrentPosition() + (int) (-48 * COUNTS_PER_INCH));
                br_motor.setTargetPosition(br_motor.getCurrentPosition() + (int) (-48 * COUNTS_PER_INCH));
                tl_motor.setPower(1);
                tr_motor.setPower(1);
                bl_motor.setPower(1);
                br_motor.setPower(1);
            }else if (gamepad1.x) {
                tr_motor.setTargetPosition(tr_motor.getCurrentPosition() - (int) (48 * COUNTS_PER_INCH));
                tl_motor.setTargetPosition((int) (48 * COUNTS_PER_INCH));
                br_motor.setTargetPosition((int) (48 * COUNTS_PER_INCH));
                bl_motor.setTargetPosition(bl_motor.getCurrentPosition() - (int) (48 * COUNTS_PER_INCH));
                tl_motor.setPower(1);
                tr_motor.setPower(1);
                bl_motor.setPower(1);
                br_motor.setPower(1);
            }






            telemetry.addData("Encoder tl", tl_motor.getCurrentPosition());
            telemetry.addData("Encoder tr", tr_motor.getCurrentPosition());
            telemetry.addData("Encoder bl", bl_motor.getCurrentPosition());
            telemetry.addData("Encoder br", br_motor.getCurrentPosition());
            telemetry.update();

        }

    }
}
