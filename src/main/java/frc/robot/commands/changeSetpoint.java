package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class changeSetpoint extends Command {
  int m_level = 0;
  /** Creates a new travelToSetpoint. */
  private ElevatorSubsystem m_elevator;
  public changeSetpoint(ElevatorSubsystem elevator, int level) {
    m_elevator = elevator;
    m_level = level;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      switch (m_level) {
        case 1:
          m_elevator.moveToL1();
          break;
        case 2:
          m_elevator.moveToL2();
          break;
        case 3:
          m_elevator.moveToL3();
          break;
        case 4:
          m_elevator.moveToL4();
          break;
        default:
          m_elevator.moveToL0();
          break;
      }
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
