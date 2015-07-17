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

import com.jacobamason.FTCRC_Extensions.Drive;
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
public class NxtTeleOp extends OpMode
{

    // position of the claw servo
    double clawPosition;

    // amount to change the claw servo position by
    double clawDelta = 0.05;

    // position of the wrist servo
    double wristPosition;

    // amount to change the wrist servo position by
    double wristDelta = 0.05;

    DcMotorController.DeviceMode devMode;
    DcMotorController wheelController;
    DcMotor motorRight;
    DcMotor motorLeft;

    Servo claw;
    Servo wrist;

    int numOpLoops = 1;
    Drive driver = new Drive(this);

    /**
     * Constructor
     */
    public NxtTeleOp()
    {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void start()
    {

        motorRight = hardwareMap.dcMotor.get("right");
        motorLeft = hardwareMap.dcMotor.get("left");
        claw = hardwareMap.servo.get("claw"); // channel 6
        wrist = hardwareMap.servo.get("arm"); // channel 1

        wheelController = hardwareMap.dcMotorController.get("motor_controller");
        devMode = DcMotorController.DeviceMode.WRITE_ONLY;

//        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // set the mode
        // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        wristPosition = 0.1;
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
     * Gamepad 1 controls the motors via the left stick, and it controls the wrist/claw via the a,b,
     * x, y buttons
     */

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

            driver.tank_drive(motorLeft, motorRight);

            // update the position of the wrist
            if (gamepad1.a)
            {
                wristPosition -= wristDelta;
            }

            if (gamepad1.y)
            {
                wristPosition += wristDelta;
            }

            // update the position of the claw
            if (gamepad1.x)
            {
                clawPosition -= clawDelta;
            }

            if (gamepad1.b)
            {
                clawPosition += clawDelta;
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

}
