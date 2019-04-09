package org.firstinspires.ftc.teamcode.organizedImplementation.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "MechanumTeleOp", group = "TeleOp")
//@Disabled
public class MechanumTeleOp extends LinearOpMode {

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    public void runOpMode() {
        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        waitForStart();

        while(opModeIsActive()) {
            double xPos = gamepad1.right_stick_x;
            double yPos = gamepad1.right_stick_y;
            double tan = yPos/xPos;
            double rSpeed = (Math.sqrt(2) * (1 - tan))/(2 * (1 + tan));

            double speed = Math.hypot(xPos, yPos);
            if(yPos >= 0 && xPos > 0) {
                tr_motor.setPower(speed);
                bl_motor.setPower(speed);
                tl_motor.setPower(speed * -rSpeed);
                br_motor.setPower(speed * -rSpeed);
            }else if(yPos >= 0 && xPos < 0) {
                tl_motor.setPower(speed);
                br_motor.setPower(speed);
                tr_motor.setPower(speed * -rSpeed);
                bl_motor.setPower(speed * -rSpeed);
            }else if(yPos <= 0 && xPos > 0) {
                tl_motor.setPower(-speed);
                br_motor.setPower(-speed);
                tr_motor.setPower(speed * rSpeed);
                bl_motor.setPower(speed * rSpeed);
            }else if(yPos <= 0 && xPos < 0) {
                tr_motor.setPower(-speed);
                bl_motor.setPower(-speed);
                tl_motor.setPower(speed * rSpeed);
                br_motor.setPower(speed * rSpeed);
            }else if(yPos > 0 && xPos == 0) {
                tl_motor.setPower(speed);
                br_motor.setPower(speed);
                tr_motor.setPower(speed);
                bl_motor.setPower(speed);
            }else if(yPos < 0 && xPos == 0) {
                tl_motor.setPower(-speed);
                br_motor.setPower(-speed);
                tr_motor.setPower(-speed);
                bl_motor.setPower(-speed);
            }else {
                tl_motor.setPower(0);
                br_motor.setPower(0);
                tr_motor.setPower(0);
                bl_motor.setPower(0);
            }
            telemetry.addData("Speed 1", tl_motor.getPower());
            telemetry.addData("Speed 2", tr_motor.getPower());
            telemetry.addData("Angle", Math.atan(tan));
        }

    }

}
