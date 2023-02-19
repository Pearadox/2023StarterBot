// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GripperSubsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private AHRS _gyro = new AHRS(SPI.Port.kMXP);
  private final GripperSubsystem _gripper = new GripperSubsystem();
  private final ArmSubsystem _arm = new ArmSubsystem();
  private DriveTrain _drivetrain = new DriveTrain(_gyro);
  private Joystick _joystick = new Joystick(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController _driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    _drivetrain.setDefaultCommand(new ArcadeDrive(_drivetrain, _joystick));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //set up gripper open/close
    new JoystickButton(_driverController, XboxController.Button.kRightBumper.value)
      .onTrue(new InstantCommand(() -> _gripper.openGripper()))
      .onFalse(new InstantCommand(() -> _gripper.closeGripper()));

    //set up arm preset positions
    new JoystickButton(_driverController, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(() -> _arm.setTargetPosition(Constants.Arm.kHomePosition, _gripper)));
    new JoystickButton(_driverController, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> _arm.setTargetPosition(Constants.Arm.kScoringPosition, _gripper)));
    new JoystickButton(_driverController, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> _arm.setTargetPosition(Constants.Arm.kIntakePosition, _gripper)));
    new JoystickButton(_driverController, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(() -> _arm.setTargetPosition(Constants.Arm.kFeederPosition, _gripper)));

    //set up arm manual and auto functions
    _arm.setDefaultCommand(new RunCommand(
      () ->
        _arm.runAutomatic()
      , _arm)
    );
    new Trigger(() -> 
      Math.abs(_driverController.getRightTriggerAxis() - _driverController.getLeftTriggerAxis()) > Constants.OperatorConstants.kArmManualDeadband
      ).whileTrue(new RunCommand(
        () ->
          _arm.runManual((_driverController.getRightTriggerAxis() - _driverController.getLeftTriggerAxis()) * Constants.OperatorConstants.kArmManualScale)
        , _arm));




    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // _driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    _drivetrain.resetEncoders();
    _drivetrain.zeroHeading();
    return Autos.TestAuto(_drivetrain);
  }
}
