/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class NxtTeleOp_TankDrive extends OpMode
{

    // position of the claw servo
    double clawPosition;

    // amount to change the claw servo position by
    double clawDelta = 0.01;

    // position of the wrist servo
    double wristPosition;

    // amount to change the wrist servo position by
    double wristDelta = 0.01;

    DcMotorController.DeviceMode devMode;
    DcMotorController wheelController;
    DcMotor motorRight;
    DcMotor motorLeft;

    Servo claw;
    Servo wrist;

    int numOpLoops = 1;

    /**
     * Constructor
     */
    public NxtTeleOp_TankDrive()
    {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void start()
    {

        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        claw = hardwareMap.servo.get("servo_6"); // channel 6
        wrist = hardwareMap.servo.get("servo_1"); // channel 1

        wheelController = hardwareMap.dcMotorController.get("wheels");
        devMode = DcMotorController.DeviceMode.WRITE_ONLY;

        motorRight.setDirection(DcMotor.Direction.REVERSE);
        //motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // set the mode
        // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        wristPosition = 0.6;
        clawPosition = 0.5;
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop()
    {

        // The op mode should only use "write" methods (setPower, setChannelMode, etc) while in
        // WRITE_ONLY mode or SWITCHING_TO_WRITE_MODE
        if (allowedToWrite())
        {
        /*
         * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

            // I left this here because I'm not sure what it does yet - JM
            if (gamepad1.dpad_left)
            {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }
            if (gamepad1.dpad_right)
            {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                motorRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            }

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
                wristPosition += wristPosition;
            }
            //lower arm - left trigger
            if (gamepad1.left_trigger > 0.25)
            {
                wristPosition -= wristPosition;
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

            // clip the position values so that they never exceed 0..1
            wristPosition = Range.clip(wristPosition, 0, 1);
            clawPosition = Range.clip(clawPosition, 0, 1);

            // write position values to the wrist and claw servo
            wrist.setPosition(wristPosition);
            claw.setPosition(clawPosition);
        }

        // To read any values from the NXT controllers, we need to switch into READ_ONLY mode.
        // It takes time for the hardware to switch, so you can't switch modes within one loop of the
        // op mode. Every 17th loop, this op mode switches to READ_ONLY mode, and gets the current power.
        if (numOpLoops % 17 == 0)
        {
            // Note: If you are using the NxtDcMotorController, you need to switch into "read" mode
            // before doing a read, and into "write" mode before doing a write. This is because
            // the NxtDcMotorController is on the I2C interface, and can only do one at a time. If you are
            // using the USBDcMotorController, there is no need to switch, because USB can handle reads
            // and writes without changing modes. The NxtDcMotorControllers start up in "write" mode.
            // This method does nothing on USB devices, but is needed on Nxt devices.
            wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        }

        // Every 17 loops, switch to read mode so we can read data from the NXT device.
        // Only necessary on NXT devices.
        if (numOpLoops % 19 == 0)
        {

            // Update the reads after some loops, when the command has successfully propagated through.
            telemetry.addData("Text", "free flow text");
            telemetry.addData("left motor", motorLeft.getPower());
            telemetry.addData("right motor", motorRight.getPower());
            telemetry.addData("RunMode: ", motorLeft.getChannelMode().toString());

            // Only needed on Nxt devices, but not on USB devices
            wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

            // Reset the loop
            numOpLoops = 0;
        }

        // Update the current devMode
        devMode = wheelController.getMotorControllerDeviceMode();
        numOpLoops++;

    }

    /*
     * Code to run when the op mode is first disabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop()
    {

    }

    // If the device is in either of these two modes, the op mode is allowed to write to the HW.
    private boolean allowedToWrite()
    {
        return (devMode == DcMotorController.DeviceMode.WRITE_ONLY);
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
