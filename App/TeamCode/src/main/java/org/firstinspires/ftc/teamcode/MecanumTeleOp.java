package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    public void runOpMode() {

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        tr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        tr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {

            double xPos = -gamepad1.right_stick_x;
            double yPos = -gamepad1.right_stick_y;
            double rot = -gamepad1.left_stick_x;

            double scalar = Math.hypot(yPos, xPos);
            double largestPower = 1;

            double tlPower = scalar * (xPos + yPos + rot);
            double trPower = scalar * (xPos - yPos - rot);
            double blPower = scalar * (xPos - yPos + rot);
            double brPower = scalar * (xPos + yPos - rot);

            double[] motorPower = {tlPower, trPower, blPower, brPower};

            largestPower = findLargest(motorPower);

            if(largestPower > 1 || largestPower < -1) {
                scalar = Math.hypot(yPos, xPos) / Math.abs(largestPower);
            }else {
                scalar = 1;
            }

            tlPower = scalar * (xPos + yPos + rot);
            trPower = scalar * (xPos - yPos - rot);
            blPower = scalar * (xPos - yPos + rot);
            brPower = scalar * (xPos + yPos - rot);

            tl_motor.setPower(tlPower);
            tr_motor.setPower(trPower);
            bl_motor.setPower(blPower);
            br_motor.setPower(brPower);

        }
    }

    public double findLargest(double[] motorPower) {
        double largest = motorPower[0];
        for(double power : motorPower) {
            if(power > largest) {
                largest = power;
            }
        }
        return largest;
    }
}