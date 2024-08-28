/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.SliceMatrixF;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop TankV2", group="Robot")
//@Disabled
public class TeleopTankControlsV2 extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    public DcMotor  leftfrontDrive   = null;
    public DcMotor  rightfrontDrive  = null;
    public DcMotor  leftbackDrive   = null;
    public DcMotor  rightbackDrive  = null;
    public DcMotor  Lift     = null;
    public DcMotor   Airplane  = null;

    static final int       ARM_BOTTOM = 10;

    static final int        ARM_MIDDLE = 500;

    static final int       ARM_TOP = 1900;
    private double       ARM_POWER = 0.5;

    int armState = 1;
    boolean isDirection = false;
    // false is stop/down, true is up

    public Servo    Vert     = null;
    public Servo    Claw     = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftfrontDrive  = hardwareMap.get(DcMotor.class, "LF");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "RF");
        leftbackDrive  = hardwareMap.get(DcMotor.class, "LB");
        rightbackDrive = hardwareMap.get(DcMotor.class, "RB");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
//        Arm = hardwareMap.get(DcMotor.class, "Arm");

        Vert = hardwareMap.get(Servo.class, "Vert");
        Airplane = hardwareMap.get(DcMotor.class, "Plane");
        Claw = hardwareMap.get(Servo.class, "Claw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.REVERSE);
        Airplane.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        runArmTask();
        runDriveTask();
        runServoTask();
        runClawTask();
        runDriveArm();
        runAirplaneTask();


        // GAMEPAD 1 CONTROLS

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)

    }

    public void runDriveTask () {
        double left;
        double right;
        double driveSpeed = .5;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        leftfrontDrive.setPower(left);
        rightfrontDrive.setPower(right);
        leftbackDrive.setPower(left);
        rightbackDrive.setPower(right);

        if(gamepad1.right_trigger > 0) {
            leftfrontDrive.setPower(driveSpeed);
            rightfrontDrive.setPower(-driveSpeed);
            leftbackDrive.setPower(-driveSpeed);
            rightbackDrive.setPower(driveSpeed);
        }

        if(gamepad1.left_trigger > 0) {
            leftfrontDrive.setPower(-driveSpeed);
            rightfrontDrive.setPower(driveSpeed);
            leftbackDrive.setPower(driveSpeed);
            rightbackDrive.setPower(-driveSpeed);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    public void runAirplaneTask () {

        if(gamepad1.y) {
            Airplane.setPower(0.5);
        }
        else {
            Airplane.setPower(0);
        }
    }
    public void runServoTask () {
        if (gamepad2.right_bumper) {
            Vert.setPosition(.2);
        } else if (gamepad2.left_bumper) {
            Vert.setPosition(.3);
        } else {
            Vert.setPosition(0.5);
        }
        }
    public void runClawTask (){
            if (gamepad2.x) {
                Claw.setPosition(0.5);
            }
            else if (gamepad2.b) {
                Claw.setPosition(0.4);
            }
        }
    public void runDriveArm () {
        if (gamepad1.x) {
            Lift.setTargetPosition(ARM_BOTTOM);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setPower(1);
            while (Lift.isBusy()) {
                telemetry.addData("SlideDown", "");
                telemetry.update();
            }
            Lift.setPower(0);
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void runArmTask (){
        int currentPosition = Lift.getCurrentPosition();
        Lift.getCurrentPosition();
        telemetry.addData("Current pos:","%d",currentPosition);
        switch (armState) {
            case 1:
                if (gamepad2.dpad_up) {
                    // move to middle
                    // set position 500, set RTP, setPower
                    isDirection=true;
                    Lift.setTargetPosition(ARM_MIDDLE);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift.setPower(ARM_POWER);
                    armState = 2;
                } else {
                    //nothing and telemetry
                    telemetry.addData("CASE:","1");
                }
                break;
            case 2:
                if (Lift.isBusy()) {
                    //motor moving do nothing
                    telemetry.addData("CASE:","2 is busy");
                } else if (!Lift.isBusy() && isDirection) {
                    // bottom to middle
                    armState = 3;
                    //set RUE, setPower(zero)
                    Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Lift.setPower(0);
                    telemetry.addData("CASE:","2-->3");
                } else if (!Lift.isBusy() && !isDirection) {
                    // middle to bottom
                    armState = 1;
                    // set RUE, setPower(zero)
                    Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Lift.setPower(0);
                    telemetry.addData("CASE:","2-->1");
                }
                break;
            case 3:
                if (gamepad2.dpad_up) {
                    // set position 1900, RTP, setPower
                    isDirection=true;
                    Lift.setTargetPosition(ARM_TOP);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift.setPower(ARM_POWER);
                    armState = 4;
                } else if (gamepad2.dpad_down) {
                    //set position 10, RTP, setPower
                    isDirection=false;
                    Lift.setTargetPosition(ARM_BOTTOM);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift.setPower(ARM_POWER);
                    armState = 2;
                } else {
                    //do nothing and telemetry
                    telemetry.addData("CASE:","3");
                }
                break;
            case 4:
                if (Lift.isBusy()) {
                    //motor moving do nothing
                    telemetry.addData("CASE:","4 is busy");
                } else if (!Lift.isBusy() && isDirection) {
                    // middle to top
                    armState = 5;
                    //set RUE, setPower(zero)
                    Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Lift.setPower(0);
                    telemetry.addData("CASE:","4-->5");
                } else if (!Lift.isBusy() && !isDirection) {
                    // top to middle
                    armState = 3;
                    // set RUE, setPower(zero)
                    Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Lift.setPower(0);
                    telemetry.addData("CASE:","4-->3");
                }
                break;
            case 5:
                if (gamepad2.dpad_down) {
                    // set position 500, RTP, setPower
                    isDirection=false;
                    Lift.setTargetPosition(ARM_MIDDLE);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift.setPower(ARM_POWER);
                    armState = 4;
                } else {
                    //do nothing and telemetry
                    telemetry.addData("CASE:","5");

                }
                break;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
