// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;

public class Conveyor extends SubsystemBase {
  private CANSparkMax motor1, motor2;
  private RelativeEncoder encoder1, encoder2;
  private double speed;
  private NetworkTable table;
  private DigitalInput sensor1;
  private DigitalInput sensor2;

  public Conveyor() {
    motor1 =  new CANSparkMax(Electrical.conveyor1, MotorType.kBrushless);
    encoder1 = motor1.getEncoder();

    motor2 = new CANSparkMax(Electrical.conveyor2, MotorType.kBrushless);
    encoder2 = motor2.getEncoder();

    sensor1 = new DigitalInput(9);
    sensor2 = new DigitalInput(10);

    motor1.setIdleMode(IdleMode.kBrake);
    motor2.setIdleMode(IdleMode.kBrake);

    table = NetworkTableInstance.getDefault().getTable("conveyor");

    periodic();
  }

  public void runConveyor() {
    motor1.set(speed);
    motor2.set(-speed);
  }

  public void runConveyor(double newSpeed) {
    motor1.set(newSpeed);
    motor2.set(-newSpeed);
  }

  public void stopConveyor() {
    motor1.set(0);
    motor2.set(0);
  }

  public boolean getTopSensor() {
    return !sensor1.get();
  }

  public boolean getBotSensor() {
    return !sensor2.get();
  }

  public void initDefaultCommand() {
    
  }

  @Override
  public void periodic() {
    SmartDashboard.getNumber("ConveyorSetpoint", 0.7);
    SmartDashboard.getNumber("ConveyorSpeed", encoder1.getVelocity());
    SmartDashboard.getNumber("HopperSpeed", encoder2.getVelocity());
    SmartDashboard.getBoolean("BallsDetectedTop", !sensor1.get());
    SmartDashboard.getBoolean("BallsDetectedBot", !sensor1.get());
  }
}
