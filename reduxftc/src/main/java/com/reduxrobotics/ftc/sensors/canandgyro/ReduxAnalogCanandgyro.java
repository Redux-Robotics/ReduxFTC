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
package com.reduxrobotics.ftc.sensors.canandgyro;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.HashSet;
import java.util.Set;

/**
 * Device class for a Redux Robotics Canandgyro connected via an analog port.
 * @author guineawheek@reduxrobotics.com
 * @author j5155 ftcjdhs@gmail.com
 */
@ExportClassToBlocks
@AnalogSensorType
@DeviceProperties(
    name = "Redux Canandgyro (Analog)",
    xmlTag = "ReduxAnalogCanandgyro",
    compatibleControlSystems = {ControlSystem.REV_HUB},
    description = "a Redux Robotics Canandgyro connected via the analog port"
)
public class ReduxAnalogCanandgyro implements AnalogSensor, OrientationSensor, HardwareDevice, IMU {
  private final AnalogInputController controller;
  private final int channel;
  private double zeroOffset;

  // The canandgyro itself will only output a maximum of MAX_VOLTAGE volts.
  private static final double MAX_VOLTAGE = 3.3;

  private double oldYaw = 0; // degrees
  private final ElapsedTime oldYawAge = new ElapsedTime();
  private double filteredYawVel = 0; // degrees/sec
  private final ElapsedTime filteredYawVelAge = new ElapsedTime();

  private static final double lowpassSmoothing = 10.0; // https://phrogz.net/js/framerate-independent-low-pass-filter.html



  /**
   * Constructor
   *
   * @param controller AnalogInput controller this channel is attached to
   * @param channel    channel on the analog input controller
   */
  public ReduxAnalogCanandgyro(AnalogInputController controller, int channel) {
    this.controller = controller;
    this.channel = channel;
    this.zeroOffset = MAX_VOLTAGE / 2.0;
  }

  /**
   * Fetches the yaw reading in the specified {@link AngleUnit}.
   *
   * @param unit the unit the angle reading should use.
   * @return the angle reading in the specified units
   */
  @ExportToBlocks(
      heading = "read Canandgyro yaw angle",
      comment = "Fetches the yaw reading in the specified units of angle (degrees or radians)",
      tooltip = "Fetches the yaw heading of the gyro",
      parameterLabels = { "angleUnit" }
  )
  public double getYaw(AngleUnit unit) {
    double yaw = getRawYaw() - zeroOffset;
    updateLowpassFilter(yaw);
    return unit.fromDegrees(yaw);
  }

  /**
   * Get yaw directly from the voltage without the zero correction
   * @return raw yaw in degrees
   */
  private double getRawYaw() {
    double rawVoltage = readRawVoltage();
    // deadband detection
    if (rawVoltage <= 0.005) {
      return 0;
    } else if (rawVoltage >= 3.295) {
      return 360;
    } else {
      // corrects for error and nonlinearity in voltage read by the hub
      return rawVoltage * 108.95105635 - 179.64903577;
    }
  }

  /**
   * Sets the yaw angle to be the specified angle.
   * <p>
   * This is accomplished by changing the in-software zero point such that the currently read
   * value equals the specified yaw angle.
   * </p>
   *
   * @param unit the unit the new yaw value is in
   */
  @ExportToBlocks(
      heading = "set Canandgyro yaw angle",
      comment = ("Sets the yaw angle to be the specified angle.\n"
          + "This is accomplished by changing the in-software zero point such that the "
          + "currently read value equals the specified yaw angle."
      ),
      tooltip = "Sets the yaw heading of the gyro",
      parameterLabels = { "newAngle", "angleUnit" }
  )
  public void setYaw(double newAngle, AngleUnit unit) {
    newAngle = unit.toDegrees(newAngle);
    resetLowpassFilter(newAngle);
    zeroOffset = getRawYaw() - newAngle;
  }

  /**
   * Zeros the Canandgyro (in software).
   * <p>
   * As an analog port is a one-way bus from device to robot, this zero point is computed
   * and stored in software and does not persist. It is recommended this is called right at the
   * start of an opmode.
   * </p>
   */
  @ExportToBlocks(
      heading = "zero Canandgyro yaw",
      comment = ("Zeros the Canandgyro (in software).\n"
          + "As an analog port is a one-way bus from device to robot, this zero point is "
          + "computed and stored in software and does not persist. It is recommended this "
          + "is called right at the start of an opmode."
      ),
      tooltip = "Zeros the Canandgyro",
      parameterLabels = { "newAngle", "angleUnit" }
  )
  public void zero() {
    setYaw(0, AngleUnit.DEGREES);
  }

  /**
   * Fetches the zero offset subtracted from the analog reading to produce a re-zeroed yaw.
   * <p>
   * You can think of {@link #getYaw} being computed as
   * <pre>
   * wrapAngleToPlusMinus180Degrees(raw voltage * (voltage to angle factor) - zeroOffset)
   * </pre>
   * </p>
   * @param unit angle unit to fetch the zero offset in
   * @return zero offset in the specified units
   */
  @ExportToBlocks(
      heading = "get Canandgyro zero offset",
      comment = ("Fetches the zero offset that is subtracted from the analog reading to "
          + "produce a re-zeroed yaw.\n"
          + "You can think of getYaw being computed as \n"
          + "wrapAngleToPlusMinus180Degrees(raw voltage * (voltage to angle factor) - zeroOffset)"
      ),
      tooltip = "Fetches the Canandgyro zero offset subtracted from the raw reading",
      parameterLabels = { "angleUnit" }
  )
  public double getZeroOffset(AngleUnit unit) {
      return unit.fromDegrees(zeroOffset);
  }

  /**
   * Sets the zero offset subtracted from the analog reading.
   * <p>
   * You can think of {@link #getYaw} being computed as
   * <pre>
   * wrapAngleToPlusMinus180Degrees(raw voltage * (voltage to angle factor) - zeroOffset)
   * </pre>
   * </p>
   * @param newOffset the new zero offset
   * @param unit the units of the new zero offset
   */
  @ExportToBlocks(
      heading = "set Canandgyro zero offset",
      comment = ("Sets the Canandgyro zero offset subtracted from the analog reading.\n"
      + "You can think of getYaw being computed as \n"
      + "wrapAngleToPlusMinus180Degrees(raw voltage * (voltage to angle factor) - zeroOffset)"
      ),
      tooltip = "Sets the zero offset that is subtracted from the analog reading.",
      parameterLabels = { "newOffset", "angleUnit" }
  )
  public void setZeroOffset(double newOffset, AngleUnit unit) {
    zeroOffset = unit.toDegrees(newOffset);
  }


  /**
   * Reads the raw voltage output straight from the control/expansion hub.
   *
   * @return voltage from [0, 3.3) volts
   */
  @ExportToBlocks(
      heading = "read raw voltage",
      comment = "Read the raw voltage straight from the control/expansion hub.",
      tooltip = "Read the raw voltage straight from the control/expansion hub."
  )
  @Override
  public double readRawVoltage() {
    return controller.getAnalogInputVoltage(channel);
  }

  @Override
  public Set<Axis> getAngularOrientationAxes() {
    HashSet<Axis> axisSet = new HashSet<>();
    axisSet.add(Axis.Z);
    return axisSet;
  }

  @Override
  public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
    return new Orientation(
            AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit,
            (float) getYaw(angleUnit), 0, 0, System.nanoTime())
            .toAxesReference(reference)
            .toAxesOrder(order);
  }

  @Override
  public Manufacturer getManufacturer() {
    return Manufacturer.Other;
  }

  @Override
  public String getDeviceName() {
    return "Redux Robotics Analog-connected Canandgyro";
  }

  @Override
  public String getConnectionInfo() {
    return controller.getConnectionInfo() + "; analog port " + channel;
  }

  @Override
  public int getVersion() {
    return 1;
  }

  @Override
  public void resetDeviceConfigurationForOpMode() {
  }

  @Override
  public void close() {
  }

  /**
   * This does nothing with the Canandgyro; it's only here to allow existing IMU uses to work.
   */
  @Override
  public boolean initialize(Parameters parameters) {
    return true;
  }

  /**
   * Resets the robot's yaw angle to 0
   */
  @Override
  public void resetYaw() {
    zero();
  }

  /**
   * @return A {@link YawPitchRollAngles} object representing the current orientation of the robot
   * in the Robot Coordinate System, relative to the robot's position the last time that
   * {@link #resetYaw()} was called, as if the robot was perfectly level at that time.
   */
  @Override
  public YawPitchRollAngles getRobotYawPitchRollAngles() {
    // Blindly using System.nanoTime() here isn't ideal, but no better option, I think
    return new YawPitchRollAngles(AngleUnit.DEGREES,getYaw(AngleUnit.DEGREES),0.0,0.0,System.nanoTime());
  }

  /**
   * @return An {@link Orientation} object representing the current orientation of the robot
   * in the Robot Coordinate System, relative to the robot's position the last time
   * that {@link #resetYaw()} was called, as if the robot was perfectly level at that time.
   * <p><p>
   * The {@link Orientation} class provides many ways to represent the robot's orientation,
   * which is helpful for advanced use cases. Most teams should use {@link #getRobotYawPitchRollAngles()}.
   */
  @Override
  public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
    // Blindly using System.nanoTime() here isn't ideal, but no better option, I think
    return new Orientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,(float) getYaw(AngleUnit.DEGREES), 0f , 0f, System.nanoTime())
            .toAxesReference(reference)
            .toAxesOrder(order)
            .toAngleUnit(angleUnit);
  }

  /**
   * @return A {@link Quaternion} object representing the current orientation of the robot
   * in the Robot Coordinate System, relative to the robot's position the last time
   * that {@link #resetYaw()} was called, as if the robot was perfectly level at that time.
   * <p><p>
   * Quaternions provide an advanced way to access orientation data that will work well
   * for any orientation of the robot, even where other types of orientation data would
   * encounter issues such as gimbal lock.
   */
  @Override
  public Quaternion getRobotOrientationAsQuaternion() {
    // is this math right?
    double yaw = getYaw(AngleUnit.RADIANS); // this has to be radians
    float qx = (float) (Math.cos(yaw/2) - Math.sin(yaw/2));
    float qy = (float) (Math.cos(yaw/2) + Math.sin(yaw/2));
    float qz = (float) (Math.sin(yaw/2) - Math.cos(yaw/2));
    float qw = (float) (Math.cos(yaw/2) + Math.sin(yaw/2));
    return new Quaternion(qw, qx, qy, qz, System.nanoTime());
  }

  /**
   * @param unit the unit to return the velocity in
   * @return The angular velocity of the robot (how fast it's turning around the three axes) in units per second
   */
  @Override
  public AngularVelocity getRobotAngularVelocity(AngleUnit unit) {
    return new AngularVelocity(unit,0f,0f,(float) getYawVelocity(unit),System.nanoTime());
  }

  /**
   * @param unit the unit to return the velocity in
   * @return The yaw velocity of the robot in units per second
   */
  public double getYawVelocity(AngleUnit unit) {
    if (oldYawAge.milliseconds() > 1) { // only read if necessary (is this value big enough?)
      getYaw(AngleUnit.DEGREES); // the output value doesn't matter, just doing this to trigger the filter
    }
    double yawVelocity = filteredYawVel;
    return unit.fromDegrees(yawVelocity);
  }

  /**
   * @param yaw current yaw in degrees
   * @return yaw velocity in degrees/sec
   */
  private double getRawYawVelocity(double yaw) {
    double yawVel = (yaw - oldYaw) / oldYawAge.seconds();
    oldYaw = yaw;
    oldYawAge.reset();
    return yawVel;
  }

  /**
   * Updates the lowpass filter based on an input in degrees
   * @param yaw input yaw in degrees
   */
  private void updateLowpassFilter(double yaw) {
    if (yaw != oldYaw) { // ensure this is a different loop with different bulk read data,
      // then update the low-pass filter
      //(is milliseconds the right unit?)
      filteredYawVel += filteredYawVelAge.milliseconds() * (getRawYawVelocity(yaw) - filteredYawVel) / lowpassSmoothing;
      filteredYawVelAge.reset();
    }
  }

  private void resetLowpassFilter(double setValue) {
    filteredYawVel = 0;
    oldYaw = setValue;
    filteredYawVelAge.reset();
    oldYawAge.reset();
  }
}
