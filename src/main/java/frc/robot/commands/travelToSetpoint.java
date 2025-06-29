// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class travelToSetpoint extends Command {
  /** Creates a new travelToSetpoint. */
  private ElevatorSubsystem m_elevator;
  public travelToSetpoint(ElevatorSubsystem elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if(m_elevator.getDriveSpeed()<0.5) {
       m_elevator.setDriveSpeed(m_elevator.getDriveSpeed()+0.05);
    }
    if(m_elevator.getCooked()) {
       m_elevator.elevatorHold(0.045);
    }
    else {
        int direction = m_elevator.atPoint();
        if(direction == 1) {
            m_elevator.elevatorUp(m_elevator.getDriveSpeed());
            //m_elevator.elevatorUp(0.5);
        }
        else if(direction == -1) {
            m_elevator.elevatorDown(-m_elevator.getDriveSpeed());
            //m_elevator.elevatorDown(-0.5);
        }
        else {
            m_elevator.elevatorHold(m_elevator.getSpeed());
        }
        // m_elevator.elevatorUp(MathUtil.clamp(m_elevator.calculatePID(),-1,1) + 0.02);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.elevatorHold(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
