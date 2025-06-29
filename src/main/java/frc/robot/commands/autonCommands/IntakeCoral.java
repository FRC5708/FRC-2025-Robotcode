// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  /** Creates a new spitCoral. */
  private CoralSubsystem m_coral;
  private ElevatorSubsystem m_elevator;
  private int coralThrough;
  public IntakeCoral(CoralSubsystem coral,ElevatorSubsystem elevator) {
    m_coral = coral;
    m_elevator = elevator;
    addRequirements(m_coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_elevator.checkPhotoeye()) {
      coralThrough = 1;
    }
    else {
      coralThrough = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_elevator.checkPhotoeye()) {
      coralThrough = 1;
    }
    if(coralThrough == 1 && !m_elevator.checkPhotoeye()) {
      coralThrough = 2;
    }
    m_coral.coralPlace(0.2);
    System.out.println(coralThrough);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.coralPlace(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralThrough == 2;
  }
}
