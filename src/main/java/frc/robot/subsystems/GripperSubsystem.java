// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {
  private CANSparkMax _motor;
  private RelativeEncoder _encoder;
  private SparkMaxPIDController _controller;
  private double _setpoint;
  private double _prevSetpoint;

  /** Creates a new ExampleSubsystem. */
  public GripperSubsystem() {
    _motor = new CANSparkMax(Constants.Gripper.kGripperCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    _motor.setInverted(false);
    _motor.setSmartCurrentLimit(Constants.Gripper.kCurrentLimit);
    _motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    _motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    _motor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Gripper.kSoftLimitForward);
    _motor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Gripper.kSoftLimitReverse);

    _encoder = _motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    _controller = _motor.getPIDController();
    PIDGains.setSparkMaxGains(_controller, Constants.Gripper.kPositionPIDGains);

    _motor.burnFlash();

    _setpoint = Constants.Gripper.kClosePosition;
  }

  public boolean isSafe() {
    return _encoder.getPosition() > Constants.Gripper.kSafePosition;
  }

  public void openGripper() {
    _setpoint = Constants.Gripper.kOpenPosition;
  }

  public void closeGripper() {
    _setpoint = Constants.Gripper.kClosePosition;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    if (_setpoint != _prevSetpoint) {
      _controller.setReference(_setpoint, CANSparkMax.ControlType.kPosition);
    }
    _prevSetpoint = _setpoint;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Setpoint", () -> _setpoint, (val) -> _setpoint = val);
    builder.addDoubleProperty("Position", () -> _encoder.getPosition(), null);
  }
}
