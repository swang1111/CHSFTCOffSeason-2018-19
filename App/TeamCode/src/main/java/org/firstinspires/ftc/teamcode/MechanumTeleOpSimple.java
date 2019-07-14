package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Mechanum TeleOp Simple", group = "TeleOp") // go organize it
//@Disabled
public class MechanumTeleOpSimple extends LinearOpMode {

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


        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.y) robotPerspective = !robotPerspective;


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double xPos = -gamepad1.right_stick_x;
            double yPos = -gamepad1.right_stick_y;
            double rot = -gamepad1.left_stick_x;
            double gyroAngle = angles.firstAngle;
            double adjustedAngle = 0;
            double scalar = Math.hypot(yPos, xPos);
            double largestPower = 1;

            if (robotPerspective) {

                largestPower = findLargest(
                        scalar * (yPos - xPos + rot),
                        scalar * (yPos + xPos - rot),
                        scalar * (yPos + xPos + rot),
                        scalar * (yPos - xPos - rot)
                        );

                if(largestPower > 1 || largestPower < -1) {
                    scalar = Math.hypot(yPos, xPos) / Math.abs(largestPower);
                }else {
                    scalar = 1;
                }


                tl_motor.setPower(scalar * (yPos - xPos + rot));
                tr_motor.setPower(scalar * (yPos + xPos - rot));
                bl_motor.setPower(scalar * (yPos + xPos + rot));
                br_motor.setPower(scalar * (yPos - xPos - rot));



            }
            else {

                adjustedAngle = gyroAngle + Math.toDegrees(Math.atan2(yPos, xPos));
                while (adjustedAngle >= 360) {
                    adjustedAngle -= 360;
                }
                while (adjustedAngle < 0) {
                    adjustedAngle += 360;
                }
                adjustedAngle *= (2 * Math.PI/360);

                largestPower = findLargest(
                        scalar * (Math.sin(adjustedAngle) - Math.cos(adjustedAngle) + rot),
                        scalar * (Math.sin(adjustedAngle) + Math.cos(adjustedAngle) - rot),
                        scalar * (Math.sin(adjustedAngle) + Math.cos(adjustedAngle) + rot),
                        scalar * (Math.sin(adjustedAngle) - Math.cos(adjustedAngle) - rot)
                        );

                if(largestPower > 1 || largestPower < -1) {
                    scalar = Math.hypot(yPos, xPos) / Math.abs(largestPower);
                }else {
                    scalar = 1;
                }

                tl_motor.setPower(scalar * (Math.sin(adjustedAngle) - Math.cos(adjustedAngle) + rot));
                tr_motor.setPower(scalar * (Math.sin(adjustedAngle) + Math.cos(adjustedAngle) - rot));
                bl_motor.setPower(scalar * (Math.sin(adjustedAngle) + Math.cos(adjustedAngle) + rot));
                br_motor.setPower(scalar * (Math.sin(adjustedAngle) - Math.cos(adjustedAngle) - rot));


            }



            if(gamepad1.x) enableDpad = !enableDpad;

            if(enableDpad) {

                double dpadAngle = angles.firstAngle;
                double error = 0;

                while(gamepad1.dpad_up || gamepad1.dpad_left ||
                        gamepad1.dpad_right || gamepad1.dpad_down) {

                    error = (angles.firstAngle - dpadAngle) / 1.5;


                    if (gamepad1.dpad_up) {

                        tr_motor.setPower(Range.clip(1 - error, 1, -1));
                        tl_motor.setPower(Range.clip(1 + error, 1, -1));
                        bl_motor.setPower(Range.clip(1 + error, 1, -1));
                        br_motor.setPower(Range.clip(1 - error, 1, -1));
                    }
                    if (gamepad1.dpad_down) {

                        tr_motor.setPower(Range.clip(-1 - error, 1, -1));
                        tl_motor.setPower(Range.clip(-1 + error, 1, -1));
                        bl_motor.setPower(Range.clip(-1 + error, 1, -1));
                        br_motor.setPower(Range.clip(-1 - error, 1, -1));
                    }
                    if (gamepad1.dpad_right) {

                        tr_motor.setPower(Range.clip(1 + error, 1, -1));
                        tl_motor.setPower(Range.clip(-1 - error, 1, -1));
                        bl_motor.setPower(Range.clip(1 - error, 1, -1));
                        br_motor.setPower(Range.clip(-1 + error, 1, -1));
                    }
                    if (gamepad1.dpad_left) {
                        tr_motor.setPower(Range.clip(-1 + error, 1, -1));
                        tl_motor.setPower(Range.clip(1 - error, 1, -1));
                        bl_motor.setPower(Range.clip(-1 - error, 1, -1));
                        br_motor.setPower(Range.clip(1 + error, 1, -1));
                    }
                }
                tr_motor.setPower(0);
                tl_motor.setPower(0);
                bl_motor.setPower(0);
                br_motor.setPower(0);

            }


        }

    }
    public double findLargest(double p1, double p2, double p3, double p4) {
        double largest = p1;
        if(p2 > largest) {
            largest = p2;
        }
        if(p3 > largest) {
           largest = p3;
        }
        if(p4 > largest) {
          largest = p4;
        }
        return largest;
    }


}