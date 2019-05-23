package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Mechanum Test", group = "TeleOp")
//@Disabled
public class MechanumTest extends LinearOpMode {

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    public void runOpMode() {

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        tl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bl_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.x) {
                tl_motor.setPower(1);
            }else {
                tl_motor.setPower(0);
            }
            if(gamepad1.y){
                tr_motor.setPower(1);
            }else {
                tr_motor.setPower(0);
            }
            if(gamepad1.a){
                bl_motor.setPower(1);

                    telemetry.addLine("A");


            }else {
                bl_motor.setPower(0);
            }
            telemetry.addData("Power", bl_motor.getPower());
            telemetry.update();
            if(gamepad1.b){
                br_motor.setPower(1);
            }else {
                br_motor.setPower(0);
            }

        }

    }

}
