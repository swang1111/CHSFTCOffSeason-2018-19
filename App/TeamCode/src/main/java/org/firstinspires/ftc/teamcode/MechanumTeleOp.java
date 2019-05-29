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

@TeleOp (name = "Mechanum TeleOp", group = "TeleOp")
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

        tl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bl_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean robotPerspective = true;


        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.y) robotPerspective = !robotPerspective;


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double xPos = gamepad1.right_stick_x;
            double yPos = gamepad1.right_stick_y;
            double tan;
            double rSpeed;
            double speed;

            if (robotPerspective) {

                tan = yPos / xPos;

                if(tan == -1) {
                    rSpeed = 0;
                }else {
                    rSpeed = (tan -1)/(tan+1);
                    // previous formula is rSpeed = (Math.sqrt(2) * (1 - tan)) / (2 * (1 + tan));
                }

                speed = Math.hypot(xPos, yPos);

                if (yPos >= 0 && xPos > 0) {

                    tr_motor.setPower(speed);
                    bl_motor.setPower(speed);
                    tl_motor.setPower(speed * -rSpeed);
                    br_motor.setPower(speed * -rSpeed);

                } else if (yPos >= 0 && xPos < 0) {

                    tl_motor.setPower(speed);
                    br_motor.setPower(speed);
                    tr_motor.setPower(speed * -rSpeed);
                    bl_motor.setPower(speed * -rSpeed);

                } else if (yPos <= 0 && xPos > 0) {

                    tl_motor.setPower(-speed);
                    br_motor.setPower(-speed);
                    tr_motor.setPower(speed * rSpeed);
                    bl_motor.setPower(speed * rSpeed);

                } else if (yPos <= 0 && xPos < 0) {

                    tr_motor.setPower(-speed);
                    bl_motor.setPower(-speed);
                    tl_motor.setPower(speed * rSpeed);
                    br_motor.setPower(speed * rSpeed);

                } else if (yPos > 0 && xPos == 0) {

                    tl_motor.setPower(speed);
                    br_motor.setPower(speed);
                    tr_motor.setPower(speed);
                    bl_motor.setPower(speed);

                } else if (yPos < 0 && xPos == 0) {

                    tl_motor.setPower(-speed);
                    br_motor.setPower(-speed);
                    tr_motor.setPower(-speed);
                    bl_motor.setPower(-speed);

                } else {

                    tl_motor.setPower(0);
                    br_motor.setPower(0);
                    tr_motor.setPower(0);
                    bl_motor.setPower(0);

                }

                telemetry.addData("Speed 1", tl_motor.getPower());
                telemetry.addData("Speed 2", tr_motor.getPower());
                telemetry.addData("Angle", Math.toDegrees(Math.atan2(yPos, xPos)));
                telemetry.addData("Robot Perspective", robotPerspective);
                telemetry.update();

            }
            else {
                double angle = angles.firstAngle + Math.toDegrees(Math.atan2(yPos, xPos));

                while(angle >= 360){
                    angle -= 360;
                }
                while(angle < 0) {
                    angle += 360;
                }

                tan = Math.tan(Math.toRadians(angle));

                if(tan == -1) {
                    rSpeed = 0;
                }else {
                    rSpeed = (tan -1)/(tan+1);
                    // previous formula is rSpeed = (Math.sqrt(2) * (1 - tan)) / (2 * (1 + tan));
                }

                speed = Math.hypot(xPos, yPos);

                if (angle >=0 && angle < 90) {

                    tr_motor.setPower(speed);
                    bl_motor.setPower(speed);
                    tl_motor.setPower(speed * rSpeed);
                    br_motor.setPower(speed * rSpeed);

                } else if (angle > 90 && angle <= 180) {

                    tl_motor.setPower(speed);
                    br_motor.setPower(speed);
                    tr_motor.setPower(speed * rSpeed);
                    bl_motor.setPower(speed * rSpeed);

                } else if (angle > 270 && angle < 360) {

                    tl_motor.setPower(-speed);
                    br_motor.setPower(-speed);
                    tr_motor.setPower(speed * rSpeed);
                    bl_motor.setPower(speed * rSpeed);

                } else if (angle > 180 && angle < 270) {

                    tr_motor.setPower(-speed);
                    bl_motor.setPower(-speed);
                    tl_motor.setPower(speed * rSpeed);
                    br_motor.setPower(speed * rSpeed);

                } else if (angle == 90) {

                    tl_motor.setPower(speed);
                    br_motor.setPower(speed);
                    tr_motor.setPower(speed);
                    bl_motor.setPower(speed);

                } else if (angle == 270) {

                    tl_motor.setPower(-speed);
                    br_motor.setPower(-speed);
                    tr_motor.setPower(-speed);
                    bl_motor.setPower(-speed);

                } else {

                    tl_motor.setPower(0);
                    br_motor.setPower(0);
                    tr_motor.setPower(0);
                    bl_motor.setPower(0);

                }

                telemetry.addData("Speed 1", tl_motor.getPower());
                telemetry.addData("Speed 2", tr_motor.getPower());
                telemetry.addData("Angle", Math.toDegrees(Math.atan2(yPos, xPos)));
                telemetry.addData("Robot Perspective", robotPerspective);
                telemetry.update();

            }

            //turn code is very sketchy

            double turnX = gamepad1.left_stick_x;
            double turnY = gamepad1.left_stick_y;
            double turnAngle;

            if(turnX == 0) {
                turnAngle = 0;
            }else {
                turnAngle = Math.atan2(turnY, turnX) - 90;
            }
            
            double speedMultiplier = 0.1;

            while(turnAngle >= 360){
                turnAngle -= 360;
            }
            while(turnAngle < 0) {
                turnAngle += 360;
            }
            if(turnAngle > 180) {
                turnAngle -= 360;
            }
            if(turnAngle < -180) {
                turnAngle += 360;
            }
            tr_motor.setPower(tr_motor.getPower() + turnAngle * speedMultiplier/180);
            br_motor.setPower(br_motor.getPower() + turnAngle * speedMultiplier/180);
            tl_motor.setPower(tl_motor.getPower() + -turnAngle * speedMultiplier/180);
            bl_motor.setPower(bl_motor.getPower() + -turnAngle * speedMultiplier/180);
        }

    }

}
