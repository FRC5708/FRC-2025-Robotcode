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
  private int desiredPoint = 0;
  private boolean cooked;
  private double speed = 0.02;
  private double driveSpeed = 0.5;

  private PIDController m_pid = new PIDController(0.04, 0, 0.01);

  DigitalInput input = new DigitalInput(1);

  public ElevatorSubsystem() {
    m_pid.setTolerance(0.3);
    cooked = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorUp(double power) {
    if (input.get()) {
      elevatorMotor1.set(power);
      elevatorMotor2.set(-power);
      System.out.println("Elevator Up Power: " + power);
    } else {
      System.out.println("Elevator Blocked By Coral");
    }
  }

  public void elevatorTwoMotorUp(double power1, double power2) {
    if (input.get()) {
      elevatorMotor1.set(power1);
      elevatorMotor2.set(-power2);
      System.out.println("Elevator Up Power Motor 1: " + power1);
      System.out.println("Elevator Up Power Motor 2: " + power2);
    } else {
      System.out.println("Elevator Blocked By Coral");
    }
  }

  public void resetEnc() {
    encoder1.setPosition(0.2);
  }

  public void elevatorUpFast(double power) {
    if (input.get()) {
      elevatorMotor1.set(power);
      elevatorMotor2.set(power);
      System.out.println("Elevator Up Power Motor; " + power);
    } else {
      System.out.println("Elevator Blocked By Coral");

    }
  }

  public void elevatorDown(double power) {
    elevatorMotor1.set(power);
    elevatorMotor2.set(-power);
    System.out.println("Elevator Down Power: " + power);
  }

  public void elevatorHold(double power) {
    elevatorMotor1.set(power);
    elevatorMotor2.set(-power);
    System.out.println("Elevator Hold Power: " + power);
  }

  public void elevatorStop(double power) {
    elevatorMotor1.set(power);
    elevatorMotor2.set(-power);
  }

  public boolean checkPhotoeye() {
    SmartDashboard.putNumber("encoder position Motor 1", getPosition());
    SmartDashboard.putNumber("encoder position Motor 2", getPositionEncoder2());
    SmartDashboard.putBoolean("Photoeye", input.get());
    return input.get();
  }

  public void setTarget(int target) {
    if (atPoint() == 0) {
      driveSpeed = 0.01;
    }
    if ((target < 4 && target > -1) | target == 10) {
      desiredPoint = target;
    }
    m_pid.setSetpoint(0.2);
    speed = 0.02;
    if (desiredPoint == 1) {
      m_pid.setSetpoint(Constants.NonChassis.ticksToL2);
      speed = 0.05;
    } else if (desiredPoint == 2) {
      m_pid.setSetpoint(Constants.NonChassis.ticksToL3);
      speed = 0.06;
    } else if (desiredPoint == 3) {
      m_pid.setSetpoint(Constants.NonChassis.ticksToL4);
      speed = 0.07;
    }
  }

  public int getTarget() {
    return desiredPoint;
  }

  public double getPosition() {
    return encoder1.getPosition();
  }

  public double getPositionEncoder2() {
    return encoder2.getPosition();
  }

  public int atPoint() {
    SmartDashboard.putNumber("desired point", desiredPoint);
    SmartDashboard.putNumber("encoder position", getPosition());
    double targetTicks = 0.1;
    if (desiredPoint == 1) {
      targetTicks = Constants.NonChassis.ticksToL2;
    } else if (desiredPoint == 2) {
      targetTicks = Constants.NonChassis.ticksToL3;
    } else if (desiredPoint == 3) {
      targetTicks = Constants.NonChassis.ticksToL4;
    } else if (desiredPoint == 10) {
      targetTicks = Constants.NonChassis.ticksShootL1;
    }
    if (encoder1.getPosition() > targetTicks + 1.5) {
      return -1;
    } else if (encoder1.getPosition() < targetTicks - 1.5) {
      return 1;
    } else {
      return 0;
    }
  }

  public boolean getCooked() {
    return cooked;
  }

  public double calculatePID() {
    return m_pid.calculate(getPosition());
  }

  public void cook() {
    cooked = !cooked;
  }

  public double getSpeed() {
    return speed;
  }

  public void setSpeed(double newSpeed) {
    speed = newSpeed;
  }

  public double getDriveSpeed() {
    return driveSpeed;
  }

  public void setDriveSpeed(double newSpeed) {
    driveSpeed = newSpeed;
  }
}
