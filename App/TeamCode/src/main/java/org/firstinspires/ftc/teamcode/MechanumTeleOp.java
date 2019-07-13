package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "Mechanum TeleOp", group = "TeleOp") // go organize it
//@Disabled
public class MechanumTeleOp extends LinearOpMode {

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        boolean robotPerspective = true;
        boolean enableDpad = false;
        boolean isTurning = false;
        double gyroAngle = 0;


        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.y) robotPerspective = !robotPerspective;


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double xPos = -gamepad1.right_stick_x;
            double yPos = -gamepad1.right_stick_y;
            double tan = 0;
            double rSpeed;
            double speed;
            double angle = 0;

            if (robotPerspective) {

                tan = yPos / xPos;

                if(tan == -1) {
                    rSpeed = 0;
                }else {
                    rSpeed = (tan - 1) / (tan + 1);
                    // previous formula is rSpeed = (Math.sqrt(2) * (1 - tan)) / (2 * (1 + tan));
                }

                speed = Math.hypot(xPos, yPos);

                if (yPos >= 0 && xPos > 0) {

                    enableDpad = false;
                    tr_motor.setPower(speed);
                    bl_motor.setPower(speed);
                    tl_motor.setPower(speed * rSpeed);
                    br_motor.setPower(speed * rSpeed);

                } else if (yPos >= 0 && xPos < 0) {

                    enableDpad = false;
                    tl_motor.setPower(speed);
                    br_motor.setPower(speed);
                    tr_motor.setPower(speed * rSpeed);
                    bl_motor.setPower(speed * rSpeed);

                } else if (yPos <= 0 && xPos > 0) {

                    enableDpad = false;
                    tl_motor.setPower(-speed);
                    br_motor.setPower(-speed);
                    tr_motor.setPower(speed * -rSpeed);
                    bl_motor.setPower(speed * -rSpeed);

                } else if (yPos <= 0 && xPos < 0) {

                    enableDpad = false;
                    tr_motor.setPower(-speed);
                    bl_motor.setPower(-speed);
                    tl_motor.setPower(speed * -rSpeed);
                    br_motor.setPower(speed * -rSpeed);

                } else if (yPos > 0 && xPos == 0) {

                    enableDpad = false;
                    tl_motor.setPower(speed);
                    br_motor.setPower(speed);
                    tr_motor.setPower(speed);
                    bl_motor.setPower(speed);

                } else if (yPos < 0 && xPos == 0) {

                    enableDpad = false;
                    tl_motor.setPower(-speed);
                    br_motor.setPower(-speed);
                    tr_motor.setPower(-speed);
                    bl_motor.setPower(-speed);

                } else {
                    if(!isTurning) {
                        tl_motor.setPower(0);
                        br_motor.setPower(0);
                        tr_motor.setPower(0);
                        bl_motor.setPower(0);
                    }
                }

                telemetry.addData("Speed 1", speed);
                telemetry.addData("Speed 2", rSpeed);
                telemetry.addData("Angle", Math.toDegrees(Math.atan2(yPos, xPos)));
                telemetry.addData("Robot Perspective", robotPerspective);


            }
            else {
                gyroAngle = angles.firstAngle;
                angle = gyroAngle + Math.toDegrees(Math.atan2(yPos, xPos));
                while (angle >= 360) {
                    angle -= 360;
                }
                while (angle < 0) {
                    angle += 360;
                }
                tan = Math.tan(Math.toRadians(angle));
                if (tan == -1) {
                    rSpeed = 0;
                } else {
                    rSpeed = (tan - 1) / (tan + 1);
                }

                speed = Math.hypot(xPos, yPos);

                double initialSpeed = speed;

                if((speed * rSpeed > 1) || (speed * rSpeed < -1)) {
                    telemetry.addLine("1");
                    speed /= Math.abs(initialSpeed * rSpeed);
                    rSpeed /= Math.abs(initialSpeed * rSpeed);
                }

                if((xPos != 0) || (yPos != 0)) {

                    telemetry.addLine("hi");

                    if (angle >= 0 && angle < 90) {

                        enableDpad = false;
                        tr_motor.setPower(speed);
                        bl_motor.setPower(speed);
                        tl_motor.setPower(initialSpeed * rSpeed);
                        br_motor.setPower(initialSpeed * rSpeed);

                    } else if (angle > 90 && angle <= 180) {

                        enableDpad = false;
                        tl_motor.setPower(speed);
                        br_motor.setPower(speed);
                        tr_motor.setPower(initialSpeed * rSpeed);
                        bl_motor.setPower(initialSpeed * rSpeed);

                    } else if (angle > 270 && angle < 360) {

                        enableDpad = false;
                        tl_motor.setPower(-speed);
                        br_motor.setPower(-speed);
                        tr_motor.setPower(initialSpeed * -rSpeed);
                        bl_motor.setPower(initialSpeed * -rSpeed);

                    } else if (angle > 180 && angle < 270) {

                        enableDpad = false;
                        tr_motor.setPower(-speed);
                        bl_motor.setPower(-speed);
                        tl_motor.setPower(initialSpeed * -rSpeed);
                        br_motor.setPower(initialSpeed * -rSpeed);

                    } else if (angle == 90) {

                        enableDpad = false;
                        tl_motor.setPower(speed);
                        br_motor.setPower(speed);
                        tr_motor.setPower(speed);
                        bl_motor.setPower(speed);

                    } else if (angle == 270) {

                        enableDpad = false;
                        tl_motor.setPower(-speed);
                        br_motor.setPower(-speed);
                        tr_motor.setPower(-speed);
                        bl_motor.setPower(-speed);

                    } else {

                        if (!isTurning) {
                            tl_motor.setPower(0);
                            br_motor.setPower(0);
                            tr_motor.setPower(0);
                            bl_motor.setPower(0);
                        }

                    }//
                }else{
                    if (!isTurning) {
                        tl_motor.setPower(0);
                        br_motor.setPower(0);
                        tr_motor.setPower(0);
                        bl_motor.setPower(0);
                    }
                }
                telemetry.addData("Speed 1", speed);
                telemetry.addData("Speed 2", rSpeed);
                telemetry.addData("Angle", angle);
                telemetry.addData("Gyro angle:", gyroAngle);
                telemetry.addData("Tan", tan);
                telemetry.addData("Robot Perspective (gamepad1.y)", robotPerspective);


            }

            //turn code is very sketchy

            double turnX = gamepad1.left_stick_x;
            double turnY = gamepad1.left_stick_y;
            double turnAngle;



            turnAngle = gamepad1.left_stick_x;

            if(turnAngle != 0) {
                isTurning = true;
            }else {
                isTurning = false;
            }


            tr_motor.setPower(tr_motor.getPower() - turnAngle);
            br_motor.setPower(br_motor.getPower() - turnAngle);
            tl_motor.setPower(tl_motor.getPower() + turnAngle);
            bl_motor.setPower(bl_motor.getPower() + turnAngle);



            if(gamepad1.x) enableDpad = !enableDpad;

            if(enableDpad) {

                //double dpadAngle =  angles.firstAngle;

                while(gamepad1.dpad_up) {
                    tr_motor.setPower(1);
                    tl_motor.setPower(1);
                    bl_motor.setPower(1);
                    br_motor.setPower(1);
                }
                while(gamepad1.dpad_down) {
                    tr_motor.setPower(-1);
                    tl_motor.setPower(-1);
                    bl_motor.setPower(-1);
                    br_motor.setPower(-1);
                }
                while(gamepad1.dpad_right) {
                    tr_motor.setPower(1);
                    tl_motor.setPower(-1);
                    bl_motor.setPower(1);
                    br_motor.setPower(-1);
                }
                while(gamepad1.dpad_left) {
                    tr_motor.setPower(-1);
                    tl_motor.setPower(1);
                    bl_motor.setPower(-1);
                    br_motor.setPower(1);
                }
                tr_motor.setPower(0);
                tl_motor.setPower(0);
                bl_motor.setPower(0);
                br_motor.setPower(0);
            }

            telemetry.addData("Turn Angle:", turnAngle);
            telemetry.addData("D-pad Enabled (gamepad1.x):", enableDpad);
            telemetry.addData("Right Stick X: ", xPos);
            telemetry.addData("Right Stick Y: ", yPos);
            telemetry.update();
        }

    }

}