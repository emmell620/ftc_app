package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * An empty op mode serving as a template for custom OpModes
 */
public class K9Ultra extends OpMode {

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.
    final static double ARM_MIN_RANGE = 0.20;
    final static double ARM_MAX_RANGE = 0.90;
    final static double CLAW_MIN_RANGE = 0.20;
    final static double CLAW_MAX_RANGE = 0.7;

    // position of the arm servo.
    double armPosition;

    // amount to change the arm servo position.
    double armDelta = 0.1;

    // position of the claw servo
    double clawPosition;

    // amount to change the claw servo position by
    double clawDelta = 0.1;

    DcMotor motorRight;
    DcMotor motorLeft;
    Servo claw;
    Servo arm;

    // Light sensor stuff
    LightSensor light;
    double lightValue;

    // Ultrasonic sensor stuff
    LightSensor ears;       // should be UltraSonic
    // how close are we to an obstacle
    double howClose;

    /*
    * Constructor
    */
    public K9Ultra() {

    }

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */
    @Override
    public void start() {
        /*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        arm = hardwareMap.servo.get("servo_1");
        claw = hardwareMap.servo.get("servo_6");

        light = hardwareMap.lightSensor.get("light");
        light.enableLed(true);
        ears = hardwareMap.lightSensor.get("ears");

        // assign the starting position of the wrist and claw
        armPosition = 0.2;
        clawPosition = 0.2;

    }


    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        // write the values to the motors
        motorRight.setPower(right);
        motorLeft.setPower(left);

        // update the position of the arm and claw (head and mouth)
        //raise arm - left bumper
        if (gamepad1.left_bumper)
        {
            armPosition += armDelta;
        }
        //lower arm - left trigger
        if (gamepad1.left_trigger > 0.25)
        {
            armPosition -= armDelta;
        }

        //open claw - right bumper
        if (gamepad1.right_bumper)
        {
            clawPosition += clawDelta;
        }
        //close claw - right trigger
        if (gamepad1.right_trigger > 0.25)
        {
            clawPosition -= clawDelta;
        }

        // read the value of the Light and Ultrasonic Sensors
        lightValue = light.getLightLevel();
        howClose = ears.getLightLevel();

        // clip the position values so that they never exceed their allowed range.
        armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
        clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

        // write position values to the wrist and claw servo
        arm.setPosition(armPosition);
        claw.setPosition(clawPosition);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

        telemetry.addData("0Text", "*** Robot Data***");
        telemetry.addData("1arm", "arm:  " + String.format("%.2f", armPosition));
        telemetry.addData("2claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("3left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("4right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("5ears", "ears:  " + String.format("%.2f", howClose));
        telemetry.addData("6light", "light: " + String.format("%.2f", lightValue));

    }

    /*
    * Code to run when the op mode is first disabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
    */
    @Override
    public void stop() {

    }
    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)
    {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0)
        {
            index = -index;
        } else if (index > 16)
        {
            index = 16;
        }

        double dScale;
        if (dVal < 0)
        {
            dScale = -scaleArray[index];
        } else
        {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
