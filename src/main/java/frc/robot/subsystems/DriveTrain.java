// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.JoystickConstants;
import frc.lib.drivers.*;

public class DriveTrain extends SubsystemBase {
  private PearadoxSparkMax _frontLeft = new PearadoxSparkMax(DriveTrainConstants.FRONTLEFT, MotorType.kBrushless, IdleMode.kBrake, Constants.DriveTrainConstants.kCurrentLimit, Constants.DriveTrainConstants.kFrontLeftInverted);
  private PearadoxSparkMax _frontRight = new PearadoxSparkMax(DriveTrainConstants.FRONTRIGHT, MotorType.kBrushless, IdleMode.kBrake, Constants.DriveTrainConstants.kCurrentLimit, Constants.DriveTrainConstants.kFrontRightInverted);
  private PearadoxSparkMax _backLeft = new PearadoxSparkMax(DriveTrainConstants.BACKLEFT, MotorType.kBrushless, IdleMode.kBrake, Constants.DriveTrainConstants.kCurrentLimit, Constants.DriveTrainConstants.kRearLeftInverted, _frontLeft);
  private PearadoxSparkMax _backRight = new PearadoxSparkMax(DriveTrainConstants.BACKRIGHT, MotorType.kBrushless, IdleMode.kBrake, Constants.DriveTrainConstants.kCurrentLimit, Constants.DriveTrainConstants.kRearRightInverted, _frontRight);

  private DifferentialDrive _drive = new DifferentialDrive(_frontLeft, _frontRight);

  private AHRS _gyro;
  private DifferentialDriveOdometry _odometry;
  private RelativeEncoder _frontLeftEncoder;
  private RelativeEncoder _frontRightEncoder;
  private RelativeEncoder _backLeftEncoder;
  private RelativeEncoder _backRightEncoder;

  /** Creates a new DriveTrain. */
  public DriveTrain(AHRS gyro) {

    _gyro = gyro;

    _frontLeftEncoder = _frontLeft.getEncoder();
    _frontRightEncoder = _frontRight.getEncoder();
    _backLeftEncoder = _backLeft.getEncoder();
    _backRightEncoder = _backRight.getEncoder();

    _frontLeftEncoder.setPositionConversionFactor(DriveTrainConstants.kDistancePerWheelRevolutionMeters / DriveTrainConstants.kGearReduction);
    _frontRightEncoder.setPositionConversionFactor(DriveTrainConstants.kDistancePerWheelRevolutionMeters / DriveTrainConstants.kGearReduction);
    
    resetEncoders();

    _odometry = new DifferentialDriveOdometry(_gyro.getRotation2d(), _frontLeftEncoder.getPosition(), -_frontRightEncoder.getPosition());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _odometry.update(_gyro.getRotation2d(), _frontLeftEncoder.getPosition(), -_frontRightEncoder.getPosition());

    // SmartDashboard.putNumber("Gyro", _gyro.getAngle());
    // SmartDashboard.putNumber("frontLeftEnc", _frontLeftEncoder.getPosition());
    // SmartDashboard.putNumber("frontRightEnc", -_frontRightEncoder.getPosition());
    // SmartDashboard.putNumber("backLeftEnc", _backLeftEncoder.getPosition());
    // SmartDashboard.putNumber("backRightEnc", _backRightEncoder.getPosition());
    // SmartDashboard.putNumber("Pose X", _odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Pose y", _odometry.getPoseMeters().getY());
  }
  
  public void teleopDrive(Joystick joystick) {
    _drive.arcadeDrive(joystick.getRawAxis(JoystickConstants.RIGHT_STICK_X), joystick.getRawAxis(JoystickConstants.LEFT_STICK_Y));
  }

  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_frontLeftEncoder.getVelocity() / 60, -_frontRightEncoder.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(_gyro.getRotation2d(), _frontLeftEncoder.getPosition(), -_frontRightEncoder.getPosition(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _frontLeft.setVoltage(leftVolts);
    _backLeft.setVoltage(leftVolts);
    _frontRight.setVoltage(-rightVolts);
    _backRight.setVoltage(-rightVolts);

    _drive.feed();
  }

  public void resetEncoders() {
    _frontLeftEncoder.setPosition(0);
    _frontRightEncoder.setPosition(0);
  }

  public void zeroHeading() {
    _gyro.reset();
  }
}
