// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleOpDrive extends Command {
  private final PIDController turretPID = new PIDController(.1, 0, 0);
  /** Creates a new TeleOpDrive. */

  private final CommandSwerveDrivetrain m_drivetrain = RobotContainer.drivetrain;
  public TeleOpDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drivetrain.setControl(
    RobotContainer.pilot.R1().getAsBoolean() ? 
    RobotContainer.driveRobotCentric
        .withVelocityX((-RobotContainer.pilot.getLeftY() * (RobotContainer.pilot.L1().getAsBoolean() ? .2 : 1)) * (RobotContainer.MaxSpeed * .35)) // Drive forward with negative Y (forward)
        .withVelocityY((-RobotContainer.pilot.getLeftX() * (RobotContainer.pilot.L1().getAsBoolean() ? .2 : 1)) * (RobotContainer.MaxSpeed * .35)) // Drive left with negative X (left)
        .withRotationalRate((-RobotContainer.pilot.getRightX() * (RobotContainer.pilot.L1().getAsBoolean() ? .2 : 1)) * (RobotContainer.MaxSpeed * .7)) 
    :
    RobotContainer.drive
        .withVelocityX((-RobotContainer.pilot.getLeftY() + SmartDashboard.getNumber("Joystick Y", 0) * (RobotContainer.pilot.L1().getAsBoolean() ? .2 : 1)) * (RobotContainer.MaxSpeed * .4)) // Drive forward with negative Y (forward)
        // .withVelocityX((-0.2 * (RobotContainer.pilot.leftBumper().getAsBoolean() ? .2 : 1)) * (RobotContainer.MaxSpeed * .5)) // Drive forward with negative Y (forward)
        .withVelocityY((-RobotContainer.pilot.getLeftX() - SmartDashboard.getNumber("Joystick X", 0) * (RobotContainer.pilot.L1().getAsBoolean() ? .2 : 1)) * (RobotContainer.MaxSpeed * .4)) // Drive left with negative X (left)
        .withRotationalRate((-RobotContainer.pilot.getRightX() - SmartDashboard.getNumber("Joystick Rotation", 0) * (RobotContainer.pilot.L1().getAsBoolean() ? .2 : 1)) * (RobotContainer.MaxSpeed * .4)) 
  );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
