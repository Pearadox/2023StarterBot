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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax _motor;
  private RelativeEncoder _encoder;
  private SparkMaxPIDController _controller;
  private double _setpoint;

  private TrapezoidProfile _profile;
  private Timer _timer;
  
  private TrapezoidProfile.State _targetState;
  private double _feedforward;
  private double _manualValue;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    _motor = new CANSparkMax(Constants.Arm.kArmCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    _motor.setInverted(false);
    _motor.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    _motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    _motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    _motor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Arm.kSoftLimitForward);
    _motor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Arm.kSoftLimitReverse);

    _encoder = _motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    _encoder.setPositionConversionFactor(Constants.Arm.kPositionFactor);
    _encoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);

    _controller = _motor.getPIDController();
    PIDGains.setSparkMaxGains(_controller, Constants.Arm.kArmPositionGains);

    _motor.burnFlash();

    _setpoint = Constants.Arm.kHomePosition;

    _timer = new Timer();
    _timer.start();
    _timer.reset();

    updateMotionProfile();
  }

  public void setTargetPosition(double setpoint, GripperSubsystem _gripper) {
    if (_setpoint != setpoint) {
      _setpoint = setpoint;
      updateMotionProfile();
    }
  }

  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(_encoder.getPosition(), _encoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(_setpoint, 0.0);
    _profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint, goal, state);
    _timer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = _timer.get();
    if (_profile.isFinished(elapsedTime)) {
      _targetState = new TrapezoidProfile.State(_setpoint, 0.0);
    }
    else {
      _targetState = _profile.calculate(elapsedTime);
    }

    _feedforward = Constants.Arm.kArmFeedforward.calculate(_encoder.getPosition()+Constants.Arm.kArmZeroCosineOffset, _targetState.velocity);
    _controller.setReference(_targetState.position, CANSparkMax.ControlType.kPosition, 0, _feedforward);
  }

  public void runManual(double _power) {
    //reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
    _setpoint = _encoder.getPosition();
    _targetState = new TrapezoidProfile.State(_setpoint, 0.0);
    _profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint, _targetState, _targetState);
    //update the _feedforward variable with the newly zero target velocity
    _feedforward = Constants.Arm.kArmFeedforward.calculate(_encoder.getPosition()+Constants.Arm.kArmZeroCosineOffset, _targetState.velocity);
    _motor.set(_power + (_feedforward / 12.0));
    _manualValue = _power;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() { // This method will be called once per scheduler run during simulation
    
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Final Setpoint",  () -> _setpoint, null);
    builder.addDoubleProperty("Position", () -> _encoder.getPosition(), null);
    builder.addDoubleProperty("Applied Output", () -> _motor.getAppliedOutput(), null);
    builder.addDoubleProperty("Elapsed Time", () -> _timer.get(), null);
    /*builder.addDoubleProperty("Target Position", () -> _targetState.position, null);
    builder.addDoubleProperty("Target Velocity", () -> _targetState.velocity, null);*/
    builder.addDoubleProperty("Feedforward", () -> _feedforward, null);
    builder.addDoubleProperty("Manual Value", () -> _manualValue, null);
    //builder.addDoubleProperty("Setpoint", () -> _setpoint, (val) -> _setpoint = val);
    //builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    //addChild("Controller", _controller);
  }
}
