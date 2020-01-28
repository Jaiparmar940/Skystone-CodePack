//Created by Jaivir Parmar of Team 17557 Kinetix

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "strafetest", group = "")
public class test extends LinearOpMode {

    //Created by Jaivir Parmar of Team 17557 Kinetix

    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;
   // private ColorSensor cs;
    private BNO055IMU imu;
   // private Servo arm;


    @Override
    public void runOpMode() {
        int HI;
        Orientation angles;
        double correction;
        Acceleration gravity;
        int curPos;


        int stonePos = 0;

        // change the following names so they match your configuration:

        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
       // cs = hardwareMap.colorSensor.get("cs"); (FOR USE WITH COLOR SENSOR SKYSTONE DETECTION)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
       // arm = hardwareMap.servo.get("arm");
        // lift = hardwareMap.dcMotor.get("lift");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);


        initIMU();
        waitForStart();
        if (opModeIsActive()) {

            posCheck();

            //set a power and direction for strafe. Since we have a wicked fast drivetrain, 25% is a good test speed

            fl.setPower(0.25);
            bl.setPower(-0.25);
            fr.setPower(-0.25);
            br.setPower(0.25);
            while (opModeIsActive()) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

               // HI = Color.rgb(cs.red(), cs.green(), cs.blue()); (FOR USE WITH COLOR SENSOR SKYSTONE DETECTION)


                    /*the correction variable is set to keep the robot centered at 0 while strafing to the right.
                   If you need to change this direction or angle, then follow the following steps:

                   1.) Determine the angle you want the robot to strafe on. The following code strafes on 0, but
                   we'll change it to -90 for this instance.

                   2.) Determine the direction you want the robot to go in. The wheels turning inwards represents
                   the direction of travel. If we want to strafe right, we'll have the right wheels turn inwards (FR- BR+).
                   Focus on the 2 wheels turning inwards. If we add more power to the wheel front right wheel,
                   the robot will adjust clockwise. If we add more power to the back right wheel, the robot will adjust
                   counterclockwise.


                   3.) Adjust the correction variable below so at the optimal angle, it would be zero. For instance the
                   code reads:

                    correction = Math.abs(Math.abs(angles.firstAngle - 0) * 0.015);

                    If we want to center at -90 degrees, then we need to add 90 (where it subtracts 0)
                    We do this as the robot would be at -90, and by adding 90, it would bring the correction power to 0
                    So if its at -90, it would add 90 and see that no adjustment is needed.
                    We then make sure that the value is the transformed into the absolute value, because, well
                    nobody wants to deal with negatives. If the angle is -91 and 90 is added, the result would be
                    negative and would tell the motor to slow down, which certainly would not be right.

                    We finally add a reducing constant in the formula to limit the adjustment speed, because if we didn't,
                    and the robot was off course by 1 degree, it would calculate a necessary adjustment of 100% power to
                    a single motor.

                    This code is exponential in that it largely amplifies the power needed to adjust based on how drastic
                    the robot is off course


                     */

                if (angles.firstAngle <= 0 && opModeIsActive()) { //change the angle here if necessary to match your desired center direction

                    correction = Math.abs(Math.abs(angles.firstAngle - 0) * 0.015);

                    gravity = imu.getGravity();
                    br.setPower(0.25 + correction);
                    if (angles.firstAngle >= 0) { //change the angle here if necessary to match your desired center direction
                        br.setPower(0.25);
                    }
                }
                if (angles.firstAngle >= 0 && opModeIsActive()) { //change the angle here if necessary to match your desired center direction
                    correction = Math.abs(Math.abs(angles.firstAngle - 0) * 0.015);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    fr.setPower(-0.25 - correction);
                    if (angles.firstAngle <= 0) { //change the angle here if necessary to match your desired center direction
                        fr.setPower(-0.25);

                    }
                }
               /* FOR USE WITH COLOR SENSOR SKYSTONE DETECTION:

               if ((cs.red() * cs.green()) / Math.pow(cs.blue(), 2) <= 3) {
                    fl.setPower(0);
                    bl.setPower(0);
                    fr.setPower(0);
                    br.setPower(0);
                    arm.setPosition(0);
                    telemetry.addData("curPos", fl.getCurrentPosition());
                    curPos = fl.getCurrentPosition();
                    if (fl.getCurrentPosition() > 600) {
                        stonePos = 3;
                    }
                    if (fl.getCurrentPosition() <= 600 && fl.getCurrentPosition() >= 350) {
                        stonePos = 2;
                    }
                    if (fl.getCurrentPosition() < 300) {
                        stonePos = 1;
                    }
                    telemetry.addData("stonePos", stonePos);
                    telemetry.update();
                    break;
                }

                */
            }
        }
    }

    /* What's likely causing some of your strafe issues is your encoders. To test this, run two opmodes, one that strafes
    just time and power, and another using the run to position function. We don't trust encoders, so we've made a special
    code that gets the best of both worlds. The encoders mode is set to RUN_WITHOUT_ENCODERS, but the position is still
    accounted for in the posCheck() function below. We then set the motor power and create an if statement that stops the
    robot if the current position is greater than a number (the target position), without using encoders. Make sure brake
    function is enable for fast stopping.

    Here is a sample code of running to a position while using encoder positions, but not the encoder intervention:

    posCheck();

    // posCheck() is also known as the following lines of code:

    //        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


      fl.setPower(0.25);
      bl.setPower(-0.25);
      fr.setPower(-0.25);
      br.setPower(0.25);
      while (opModeIsActive()) {

       if(fl.getCurrentPosition() > 900) { //900 represents target encoder position
       fl.setPower(0);
      bl.setPower(0);
      fr.setPower(0);
      br.setPower(0);
       break;}

     */
    private void initIMU() {


        BNO055IMU.Parameters imuParameters;
        double adjPower;

        imuParameters = new BNO055IMU.Parameters();
        adjPower = 150;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
    }

    private void posCheck() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

//Created by Jaivir Parmar of Team 17557 Kinetix