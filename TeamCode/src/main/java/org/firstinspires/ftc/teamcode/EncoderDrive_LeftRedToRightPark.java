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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="LeftRed-->RightPark", group="Robot")
//@Disabled
public class EncoderDrive_LeftRedToRightPark extends LinearOpMode {
RobotHardwareV1 robot = new RobotHardwareV1(this);
    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77952755906 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1.00;
    static final double     NORMAL_SPEED            = 0.4;
    static final double     TURN_SPEED              = 0.4;

    @Override
    public void runOpMode() {

        robot.init();

        robot.leftfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d,%7d,%7d,%7d",
                          robot.leftfrontDrive.getCurrentPosition(),
                          robot.rightfrontDrive.getCurrentPosition(),
                          robot.leftbackDrive.getCurrentPosition(),
                          robot.rightbackDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(NORMAL_SPEED, 5 , 5 , 5.0);  // S1: Go straight a little
        encoderDrive(TURN_SPEED,   15, -15, 4.0);  // S2: Turn 90 degrees
        encoderDrive(DRIVE_SPEED, 96, 96, 4.0);  // S3: Go straight to end in right park

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftfrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightfrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftbackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightbackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.leftfrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightfrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftbackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightbackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftfrontDrive.setPower(speed);
            robot.rightfrontDrive.setPower(speed);
            robot.leftbackDrive.setPower(speed);
            robot.rightbackDrive.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   //(robot.leftfrontDrive.isBusy() )) {
                   (robot.leftfrontDrive.isBusy() || robot.rightfrontDrive.isBusy() || robot.leftbackDrive.isBusy() || robot.rightbackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " at %7d,%7d,%7d,%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d,%7d,%7d,%7d",
                        robot.leftfrontDrive.getCurrentPosition(),
                        robot.rightfrontDrive.getCurrentPosition(),
                        robot.leftbackDrive.getCurrentPosition(),
                        robot.rightbackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.all_stop();

            // Turn off RUN_TO_POSITION
            robot.leftfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
