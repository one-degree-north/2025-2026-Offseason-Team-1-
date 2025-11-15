
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


@SuppressWarnings("unused")
public class OffseasonIntake extends SubsystemBase {


private static final double INTAKE_VOLTAGE = 4; 
private static final int m_intake1_ID = 1;
private static final int m_intake2_ID = 2;
private static final double SHOOT_RPM = 5500.00; //This is the RPM for shooting the object out
private static final double RPM_TOLERANCE = 50.0; 
private static final IntakeStates INTAKE = null;
       
/** Creates a new Intake. */
  private static TalonFX m_intake1;
  private static TalonFX m_intake2;
  private VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);

  //Hardware
  public OffseasonIntake() {
        setName("OffseasonIntake");
        m_intake1 = new TalonFX(m_intake1_ID, "rio");
        m_intake2 = new TalonFX(m_intake2_ID, "rio");
        configureTalonFX();
      }
    
    
  public static void configureTalonFX(){
      TalonFXConfiguration config = new TalonFXConfiguration();
  
      CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(30)
        .withStatorCurrentLimitEnable(true);
  
      config.withCurrentLimits(currentLimits);
  
      //Configure motor outputs
      
      MotorOutputConfigs motorOutput = new MotorOutputConfigs();
      motorOutput.withNeutralMode(NeutralModeValue.Coast);
      motorOutput.withInverted(InvertedValue.Clockwise_Positive);
  
      config.withMotorOutput(motorOutput);
      
      m_intake1.getConfigurator().apply(config);
      m_intake2.getConfigurator().apply(config);
    }
  
  //Enum for the intake at different points
  public enum IntakeStates {
      INTAKE,
      OUTTAKE,
      SHOOT,
      IDLE
    }

  private IntakeStates currentState = IntakeStates.IDLE;

  public void executeCurrentStateBehavior(){
    switch (currentState){

      case INTAKE:
      //Run intake motors
       setVoltage(3);
       break;
     case OUTTAKE:
     //Run the outtake motors
      setVoltage(-3);
      break;
    case SHOOT:
      setTargetRPM(5500);
      break;
    case IDLE:
    default:
       setVoltage(0);
       break;

      }

    }


  public void setState (IntakeStates newStates) {

  this.currentState = newStates;
  }
  

  private void setVoltage(double voltage) {
    System.out.println("Setting voltage to: " + voltage);
  } 
 

  private void setTargetRPM(double targetRPM) {
    if (m_intake1.isAlive()) {
      double velocitySetpointRPS = targetRPM / 60;
      m_intake1.setControl(velocityControl.withVelocity(velocitySetpointRPS));
      }
    }
    
  public boolean isAtTargetRPM() {
    double currentRPM = getCurrentRPM();
    IntakeStates desiredState = (IntakeStates) getDesiredState();
                
    if (desiredState == IntakeStates.SHOOT) {
      return Math.abs(currentRPM - SHOOT_RPM) <= RPM_TOLERANCE;
          }
                
      return false; 
        }

 private void setTargetRPM(double targetOutputRPM) {
    double gearRatio = 5.0 / 4.0;  
    double motorRPM = targetOutputRPM / gearRatio;
    double motorRPS = motorRPM / 60;  // TalonFX uses RPS
    m_intake1.setControl(velocityControl.withVelocity(motorRPS));
        
  private IntakeStates getDesiredState() {
                
      throw new UnsupportedOperationException("Unimplemented method 'getDesiredState'");
              }
        
        
  public double getCurrentRPM() {
      return m_intake1.getVelocity().getValueAsDouble() * 60; 
    }


  public static double getIntakeVoltage() {
    return INTAKE_VOLTAGE;
    }


  public static int getmIntake1Id() {
    return m_intake1_ID;
    }


  public static int getmIntake2Id() {
    return m_intake2_ID;
    }


  public static double getShootRpm() {
    return SHOOT_RPM;
    }


  public static double getRpmTolerance() {
    return RPM_TOLERANCE;
    }


  public static IntakeStates getIntake() {
    return INTAKE;
    }


  public static TalonFX getM_intake1() {
    return m_intake1;
    }


  public static void setM_intake1(TalonFX m_intake1) {
    OffseasonIntake.m_intake1 = m_intake1;
    }


  public static TalonFX getM_intake2() {
    return m_intake2;
    }


  public static void setM_intake2(TalonFX m_intake2) {
    OffseasonIntake.m_intake2 = m_intake2;
    }


  public VelocityVoltage getVelocityControl() {
    return velocityControl;
    }


  public void setVelocityControl(VelocityVoltage velocityControl) {
    this.velocityControl = velocityControl;
    }

    
  }






