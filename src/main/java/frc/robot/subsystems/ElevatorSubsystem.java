// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase {
  public SparkMax elevatorMotor1 = new SparkMax(Constants.NonChassis.elevatorMotorID1, MotorType.kBrushless);
  public SparkMax elevatorMotor2 = new SparkMax(Constants.NonChassis.elevatorMotorID2, MotorType.kBrushless);
  private RelativeEncoder encoder1 = elevatorMotor1.getEncoder();
  private RelativeEncoder encoder2 = elevatorMotor2.getEncoder();
  private static final double ramprate = .04;

  private PIDController m_pid = new PIDController(0.2, 0, 0.01);

  private double targetPos = 0.0;
  private boolean manualMode;  
  private int i = 0;
  // private int desiredPoint = 0;
  // private double driveSpeed = 0.5;

  DigitalInput input = new DigitalInput(1);

  public ElevatorSubsystem() {
    i = 0;
    m_pid.setTolerance(0.3);
    resetEncoder();
    elevatorStop();
    manualMode = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setSpeed(double speed){
    //Ramp logic, can't change speed by more .05 or 5% power per tick
    double currentSpeed = elevatorMotor1.get();
    double speedDifference = speed - currentSpeed;
    if (speedDifference > ramprate){
      speed = currentSpeed + ramprate;
    }
    else if (speedDifference < -ramprate){
      speed = currentSpeed + -ramprate;
    }

    //Clamp logic, can't go above .7 or 70% power or go below -.4 or -40% making it unable to break the elevator
    if (speed > .7){
      speed = .7;
    }
    else if (speed < -.4){
      speed = -.4;
    }
    //This is how we set the speed after the Ramp & Clamp logic
    elevatorMotor1.set(speed);
    elevatorMotor2.set(-speed); 
  } 
  
  public void elevatorUp(double speed) {
    if (!coralBlockingElevator()) {
      setSpeed(speed);
    } else {
      System.out.println("Elevator Blocked By Coral");
    }
  }

  private boolean coralBlockingElevator() {
    return input.get();
    //return false;
  }

  public void elevatorUpFast(double speed) {
    if (!coralBlockingElevator()) {
      setSpeed(speed);
    } else {
      System.out.println("Elevator Blocked By Coral");
    }
  }

  public void elevatorDown(double speed) {
    if (!coralBlockingElevator()) {
      setSpeed(speed);
    } else {
      System.out.println("Elevator Blocked By Coral");
    }
  }

  public void elevatorHold() {
    setSpeed(0.02);
  }

  public void elevatorStop()
  {
    setSpeed(0);
  }

  public boolean checkPhotoeye() {
    SmartDashboard.putNumber("encoder position Motor 1", getPosition());
    SmartDashboard.putNumber("encoder position Motor 2", getPositionEncoder2());
    SmartDashboard.putBoolean("Photoeye", coralBlockingElevator());
    return coralBlockingElevator();
  }

  public void moveToL0(){
    targetPos = 0;
  }

  public void moveToL1(){
    targetPos = Constants.NonChassis.ticksToL1;
  }

  public void moveToL2(){
    targetPos = Constants.NonChassis.ticksToL2;
  }
  
  public void moveToL3(){
    targetPos = Constants.NonChassis.ticksToL3;
  }

  public void moveToL4(){
    targetPos = Constants.NonChassis.ticksToL4;
  }

//encoder and PID are same number
  public void resetEncoder() {
  encoder1.setPosition(0.2);
  encoder2.setPosition(0.2);
}

public double getPosition() {
  return encoder1.getPosition();
}

public double getPositionEncoder2() {
  return encoder2.getPosition();
}

public void travelToSetpoint() {
  i+=1;
  boolean coralBlockingElevator = coralBlockingElevator();
  double speed = m_pid.calculate(getPosition(), targetPos);
  SmartDashboard.putBoolean("Manual Mode", manualMode);
  SmartDashboard.putNumber("Counter", i);
  SmartDashboard.putBoolean("Photoeye", coralBlockingElevator);
  if (!manualMode){
    if(!coralBlockingElevator) {
      setSpeed(speed);
    }
  }
  else {
    elevatorHold();
  }
  SmartDashboard.putNumber("Elevator Speed", speed);
  SmartDashboard.putNumber("PID position", getPosition());
  SmartDashboard.putNumber("Target", targetPos);
}

public void manualModeToggle() {
  manualMode = !manualMode;
  targetPos = getPosition();
}
  public double calculatePID() {
    return m_pid.calculate(getPosition());
  }

}
