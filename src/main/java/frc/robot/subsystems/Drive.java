// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Drive extends SubsystemBase {

 /*  // The motors on the left side of the drive.
  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(DriveConstants.kLeftFrontMotorPort, MotorType.kBrushed);
  private final CANSparkMax m_leftBackMotor = new CANSparkMax(DriveConstants.kLeftBackMotorPort, MotorType.kBrushed);

  // The motors on the right side of the drive.
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(DriveConstants.kRightFrontMotorPort, MotorType.kBrushed);
  private final CANSparkMax m_rightBackMotor = new CANSparkMax(DriveConstants.kRightBackMotorPort, MotorType.kBrushed);
*/
    // The motors on the left side of the drive.
  private final WPI_TalonSRX m_leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
  private final WPI_VictorSPX m_leftBackMotor = new WPI_VictorSPX(DriveConstants.kLeftBackMotorPort);

  // The motors on the right side of the drive.
  private final WPI_TalonSRX m_rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
  private final WPI_VictorSPX m_rightBackMotor = new WPI_VictorSPX(DriveConstants.kRightBackMotorPort);

  // The robot's drive
  private final MecanumDrive m_drive =
      new MecanumDrive(m_leftFrontMotor::set, m_leftBackMotor::set, m_rightFrontMotor::set, m_rightBackMotor::set);

  // The left-side drive encoder
  private final Encoder m_leftFrontEncoder =
      new Encoder(
          DriveConstants.kLeftFrontEncoderPorts[0],
          DriveConstants.kLeftFrontEncoderPorts[1],
          DriveConstants.kLeftFrontEncoderReversed);

  private final Encoder m_leftBackEncoder =
      new Encoder(
          DriveConstants.kLeftBackEncoderPorts[0],
          DriveConstants.kLeftBackEncoderPorts[1],
          DriveConstants.kLeftFrontEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightFrontEncoder =
      new Encoder(
          DriveConstants.kRightFrontEncoderPorts[0],
          DriveConstants.kRightFrontEncoderPorts[1],
          DriveConstants.kRightFrontEncoderReversed);

    private final Encoder m_rightBackEncoder =
      new Encoder(
          DriveConstants.kRightBackEncoderPorts[0],
          DriveConstants.kRightBackEncoderPorts[1],
          DriveConstants.kRightFrontEncoderReversed);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_leftFrontMotor.setVoltage(volts.in(Volts));
                m_leftBackMotor.setVoltage(volts.in(Volts));
                m_rightFrontMotor.setVoltage(volts.in(Volts));
                m_rightBackMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-leftFront")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftFrontMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftFrontEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftFrontEncoder.getRate(), MetersPerSecond));
                log.motor("drive-leftBack")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftBackMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftBackEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftBackEncoder.getRate(), MetersPerSecond));


                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-rightFront")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightFrontMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightFrontEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightFrontEncoder.getRate(), MetersPerSecond));

                log.motor("drive-rightBack")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightBackMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightBackEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightBackEncoder.getRate(), MetersPerSecond));

              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drive subsystem. */
  public Drive() {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontMotor.setInverted(true);
    m_rightBackMotor.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftFrontEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_leftBackEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightFrontEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightBackEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier strafe, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.driveCartesian(fwd.getAsDouble(), strafe.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");

  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

}
