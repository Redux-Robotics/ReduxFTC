/* Copyright (c) 2024 Redux Robotics. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.reduxrobotics.ftc.sensors.canandgyro.ReduxAnalogCanandgyro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * This OpMode shows how to use a Redux Robotics Canandgyro that has been connected to an analog
 * port to read yaw.
 *
 * <p> It assumes that the sensor is configured with a name of "canandgyro".
 *
 * <p> The theory of operation is that the Canandgyro will output its current absolute yaw angle
 * through the analog port as a voltage between 0 to 3.3 volts. On boot it will assume a starting
 * value of 1.65 volts to represent a yaw of 0 degrees. Rotating the device clockwise will raise the
 *  voltage towards 3.3 volts (positive 180 degrees) and rotating it counter-clockwise will lower
 * the voltage towards 0 volts (negative 180 degrees).
 * <p>
 *
 * <p> As we cannot directly communicate with the gyro to re-zero the emitted yaw, the
 * {@link ReduxAnalogCanandgyro} class wraps this raw readng and applies a software-defined zero
 * offset that can be set on opmode start via
 * {@link ReduxAnalogCanandgyro#setYaw(double, AngleUnit)} or {@link ReduxAnalogCanandgyro#zero()}
 * as during match setup the robot's heading can be assumed to be a specific angle.
 * <p>
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
//@Disabled
@TeleOp(name = "Sensor: Redux Canandgyro (Analog)", group = "Sensor")
public class SensorReduxAnalogCanandgyro extends LinearOpMode {

  ReduxAnalogCanandgyro analogCanandgyro;  // Hardware Device Object

  @Override
  public void runOpMode() {

    // get a reference to our canandgyro
    analogCanandgyro = hardwareMap.get(ReduxAnalogCanandgyro.class, "canandgyro");

    // Updates the zero point right as the opmode starts and before the robot may move.
    // This is recommended at the start of every opmode to clear the effect of any drift or shift in
    // robot pose accumulated from before-match setup.
    analogCanandgyro.zero();
    // wait for the start button to be pressed.
    waitForStart();

    boolean xPrevPressed = false;
    boolean yPrevPressed = false;

    // while the OpMode is active, loop and read the yaw angle.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      if (!xPrevPressed && gamepad1.x) {
        // Using setYaw we can set the offset of the gyro to an absolute angle.
        // When the X button is pressed, this will set our current yaw to 30 degrees.
        analogCanandgyro.setYaw(30, AngleUnit.DEGREES);
      }
      xPrevPressed = gamepad1.x;

      if (!yPrevPressed && gamepad1.y) {
        // We can also adjust the angle relative to itself as well.
        // When the Y button is pressed, we offset the zero point by 15 degrees.
        analogCanandgyro.setYaw(analogCanandgyro.getYaw(AngleUnit.DEGREES) + 15, AngleUnit.DEGREES);
      }
      yPrevPressed = gamepad1.y;

      // send the info back to driver station using telemetry function.
      telemetry.addData("Yaw (degrees)", analogCanandgyro.getYaw(AngleUnit.DEGREES));
      telemetry.update();
    }
  }
}
