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

import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashSet;
import java.util.Set;

/**
 * Device class for a Redux Robotics Canandgyro connected via an analog port.
 * @author guineawheek@reduxrobotics.com
 */
@ExportClassToBlocks
@AnalogSensorType
@DeviceProperties(
    name = "Redux Canandgyro (Analog)",
    xmlTag = "ReduxAnalogCanandgyro",
    builtIn = false,
    compatibleControlSystems = {ControlSystem.REV_HUB},
    description = "a Redux Robotics Canandgyro connected via the analog port"
)
public class ReduxAnalogCanandgyro implements AnalogSensor, OrientationSensor, HardwareDevice {
  private AnalogInputController controller = null;
  private int channel = -1;
  private double zeroOffset = 0;

  // The canandgyro itself will only output a maximum of MAX_VOLTAGE volts.
  private static final double MAX_VOLTAGE = 3.3;

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
    switch (unit) {
      case DEGREES:
        return AngleUnit.normalizeDegrees(readZeroedVoltage() / MAX_VOLTAGE * 360.0);
      case RADIANS:
        return AngleUnit.normalizeRadians(readZeroedVoltage() / MAX_VOLTAGE * (Math.PI * 2));
    }
    return 0;
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
    if (unit == AngleUnit.RADIANS) {
      newAngle = AngleUnit.RADIANS.toDegrees(newAngle);
    } else {
      newAngle = AngleUnit.normalizeDegrees(newAngle);
    }
    newAngle *= MAX_VOLTAGE / 360.0;
    zeroOffset = readRawVoltage() - newAngle;
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
   * Fetches the zero offset that is subtracted from the analog reading to produce a re-zeroed yaw.
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
    if (unit == AngleUnit.DEGREES) {
      return AngleUnit.normalizeDegrees(zeroOffset / MAX_VOLTAGE * 360);
    } else {
      return AngleUnit.normalizeRadians(zeroOffset / MAX_VOLTAGE * 2 * Math.PI);
    }
  }

  /**
   * Sets the zero offset that is subtracted from the analog reading.
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
    if (unit == AngleUnit.DEGREES) {
      newOffset = AngleUnit.normalizeDegrees(newOffset) / 360;
    } else {
      newOffset = AngleUnit.normalizeRadians(newOffset) / (2 * Math.PI);
    }
    zeroOffset = newOffset * MAX_VOLTAGE;

  }

  /**
   * Reads the raw voltage output with the zero offset applied.
   *
   * @return voltage from [0, 3.3) volts
   */
  @ExportToBlocks(
      heading = "read zeroed voltage",
      comment = "Read the raw voltage with the zero offset applied.",
      tooltip = "Read the raw voltage with the zero offset applied."
  )
  public double readZeroedVoltage() {
    double zeroed = readRawVoltage() - zeroOffset;
    while (zeroed < 0.0) {
      zeroed += MAX_VOLTAGE;
    }
    while (zeroed >= MAX_VOLTAGE) {
      zeroed -= MAX_VOLTAGE;
    }
    return zeroed;
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
}
