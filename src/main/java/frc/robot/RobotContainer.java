// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.Logitech;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();

  private Logitech driver = new Logitech(0);

  private JoystickButton a = new JoystickButton(driver, Logitech.Ports.A);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    swerveDrivetrain.setDefaultCommand(
      new RunCommand(
        () -> {
          double scale = 1;
          swerveDrivetrain.drive(
            driver.getRawAxis(Logitech.Ports.LEFT_STICK_Y) * scale,
            driver.getRawAxis(Logitech.Ports.LEFT_STICK_X) * -1 * scale,
            driver.getRawAxis(Logitech.Ports.RIGHT_STICK_X) * -1 * scale
          );
        }, 
        swerveDrivetrain
      )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver.setDeadband(Logitech.Ports.LEFT_STICK_X, 0.07);
    driver.setDeadband(Logitech.Ports.LEFT_STICK_Y, 0.07);
    driver.setDeadband(Logitech.Ports.RIGHT_STICK_Y, 0.07);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
