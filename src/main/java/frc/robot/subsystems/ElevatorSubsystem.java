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

  private PIDController m_pid = new PIDController(0.04, 0,0.01);

  private double targetPos = 0.0;
  private boolean manualMode;
  // private int desiredPoint = 0;
  // private double driveSpeed = 0.5;

  DigitalInput input = new DigitalInput(1);

  public ElevatorSubsystem() {
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
    elevatorMotor1.set(speed);
    elevatorMotor2.set(-speed); //This way you can set them both at once 
  } 
  
  public void elevatorUp(double power) {
    if (input.get()) {
      setSpeed(power);
      System.out.println("Elevator Up Power: " + power);
    } else {
      System.out.println("Elevator Blocked By Coral");
    }
  }

  public void elevatorUpFast(double power) {
    if (input.get()) {
      setSpeed(power);
      System.out.println("Elevator Up Power Motor; " + power);
    } else {
      System.out.println("Elevator Blocked By Coral");

    }
  }

  public void elevatorDown(double power) {
    setSpeed(power);
    System.out.println("Elevator Down Power: " + power);
  }

  public void elevatorHold() {
    setSpeed(0.03);
    System.out.println("Elevator Hold Power: .03");
  }

  public void elevatorStop()
  {
    setSpeed(0);
    targetPos = calculatePID();
  }

  public boolean checkPhotoeye() {
    SmartDashboard.putNumber("encoder position Motor 1", getPosition());
    SmartDashboard.putNumber("encoder position Motor 2", getPositionEncoder2());
    SmartDashboard.putBoolean("Photoeye", input.get());
    return input.get();
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
  if (!manualMode){
    setSpeed(m_pid.calculate(getPosition(), targetPos));
  }
  else {
    elevatorHold();
  }
}

public void manualModeToggle() {
  manualMode = !manualMode;
}

  public double calculatePID() {
    return m_pid.calculate(getPosition());
  }

}
