# Mecanum Usage Guide

Mecanum wheels use rollers to apply force and movement at a 45 ̊ angle, allowing for movement in any direction. In robotics, this is useful as it allows sensors to simultaneously track a target and move in any direction.

Here is a diagram of the directional forces of the wheels:

![Screen Shot 2019-08-05 at 7 29 37 PM](https://user-images.githubusercontent.com/43021436/62506989-9187b900-b7b7-11e9-8a4d-52d96275aae9.png)

  To move forward, you need to power the wheels forward. This causes the left and right forces to be equal and thus cancel. To move right, the top-left and bottom-right need to be powered forward, and the top-right and bottom-left to be powered backward. This causes the forward and backward forces to cancel, leaving only a rightward force (making the robot move to the right). To turn to the right, the left motors need to be powered forward, while the right motors need to be powered backward. We can create a table like the one below:

```

            Forward (y)     Right (x)     Turn Right (r)

Top Left        +               +             +

Top Right       +               –             –

Bottom Left     +               –             +

Bottom Right    +               +             –

```

Using this, we can create an algorithm to set the motor powers.

```

tlPower = x + y + r
trPower = x - y - r
blPower = x - y + r
brPower = x + y - r

```

If you don't initially see why this is so, the y component on each of the wheels is motorPower/(√2). Summing up the y components for all the wheels results in (2√2)y. Similarly summing up the x components (note that you need to subtract tlPower and blPower because the power is in the left direction and not the right).

While the motorPower(s) may be set to a value greater than one, the motor cannot actually run at a higher speed. To fix this we need to scale the motorPower(s) such that they all are between -1 and 1.

[Link to file containing the code below](https://github.com/swang1111/CHSFTCOffSeason-2018-19/blob/master/App/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumTeleOp.java)

```java

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
            
```
