// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public static IntakeSubsystem instance;

  private Spark frontIntakeMotor;
  private Spark backIntakeMotor;
  private SparkMax upperIntakeMotor;

  private DoubleSolenoid frontLeftSolenoid;
  private DoubleSolenoid frontRightSolenoid;
  private DoubleSolenoid backLeftSolenoid;
  private DoubleSolenoid backRightSolenoid;

  private DigitalInput centerBeamBreak;
  private Servo centerServo;

  public IntakeSubsystem() {
    frontIntakeMotor = new Spark(0);
    backIntakeMotor = new Spark(1);
    upperIntakeMotor = new SparkMax(8, MotorType.kBrushless);

    frontLeftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    frontRightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    backLeftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    backRightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);

    centerBeamBreak = new DigitalInput(0);
    centerServo = new Servo(2);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  public void extendFrontIntake() {
    frontLeftSolenoid.set(Value.kForward);
    frontRightSolenoid.set(Value.kForward);
  }

  public void retractFrontIntake() {
    frontLeftSolenoid.set(Value.kReverse);
    frontRightSolenoid.set(Value.kReverse);
  }

  public void extendBackIntake() {
    backLeftSolenoid.set(Value.kForward);
    backRightSolenoid.set(Value.kForward);
  }

  public void retractBackIntake() {
    backLeftSolenoid.set(Value.kReverse);
    backRightSolenoid.set(Value.kReverse);
  }

  public void runFrontIntake(double speed) {
    frontIntakeMotor.set(speed);
  }

  public void runBackIntake(double speed) {
    backIntakeMotor.set(speed);
  }

  public boolean getCenterBB() {
    return centerBeamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("ball in intake???", getCenterBB());
  }
}
