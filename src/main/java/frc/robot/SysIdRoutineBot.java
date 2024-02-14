// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import static frc.robot.Constants.OIConstants;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;
//import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SysIdRoutineBot {
  // The robot's subsystems
  private final Drive m_drive = new Drive();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The driver's controller
  //PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);




  public SysIdRoutineBot(){

    configureBindings();

    autoChooser.setDefaultOption("Do Nothing", null);
    autoChooser.addOption("Quasi Fwd", m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(2));
    autoChooser.addOption("Quasi Rev", m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(2));
    autoChooser.addOption("Dynamic Fwd", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(2));
    autoChooser.addOption("Dynamic Rev", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(2));
    autoChooser.addOption("Full Test", m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(2)
                                            .andThen(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(2))
                                            .andThen(m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(2))
                                            .andThen(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(2)));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }




  /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called during {@link Robot#robotInit()}.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {
    // Control the drive with split-stick arcade controls
    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // once.
    /*
    m_driverController.a().whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.b().whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.x().whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.y().whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */
    new JoystickButton(m_driverController, Button.kA.value)
      .onTrue(new InstantCommand(() -> m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)));
    new JoystickButton(m_driverController, Button.kB.value)
      .onTrue(new InstantCommand(() -> m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
    new JoystickButton(m_driverController, Button.kX.value)
      .onTrue(new InstantCommand(() -> m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward)));
    new JoystickButton(m_driverController, Button.kY.value)
      .onTrue(new InstantCommand(() -> m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)));

  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing
    return autoChooser.getSelected();
    //return m_drive.run(() -> {});
  }
}
