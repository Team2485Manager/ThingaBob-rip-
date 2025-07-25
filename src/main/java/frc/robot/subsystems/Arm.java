package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    public enum ArmStates{

        StateZero,
        StateIdle,
        StateMax,
        StateTrans

    }

    public ArmStates currentState = ArmStates.StateZero;
    public ArmStates requestedState = ArmStates.StateZero;
    public double desiredPosition = 0;
    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    public static GenericEntry motorVelocity = Shuffleboard.getTab("Arm").add("velocity", 0.0).getEntry();
  public static GenericEntry motorPosition = Shuffleboard.getTab("Arm").add("Position", 0.0).getEntry();

    

    private final TalonFX m_talon = new TalonFX(4, "rio");
    
    public Arm(){
        
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0;
        slot0Configs.kG = 0.3;

        slot0Configs.kV = 0.16;
        slot0Configs.kA = 0.005;

        slot0Configs.kP = 0.867;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.32;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 16;
        // vel/acc = time to reach constant velocity
        motionMagicConfigs.MotionMagicAcceleration = 267;
        // acc/jerk = time to reach constant acceleration
        motionMagicConfigs.MotionMagicJerk = 6700;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 70;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 120;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        m_talon.getConfigurator().apply(talonFXConfigs);

    }

    @Override
    public void periodic(){
        switch(requestedState){
            case StateZero:
                System.out.println("StateZero");
                //m_talon.set(1)
                desiredPosition = 0;
                break;
            case StateIdle:
                System.out.println("StateIdle");
                desiredPosition = 0.67;
                if(getError() < 0.4)
                    requestedState = ArmStates.StateZero;
                break;
            case StateMax:
                System.out.println("StateMax");
                desiredPosition = 1.0;
                // m_talon.setVoltage(0.2);
                break;
            default:
                desiredPosition = 0.2;
                break;
        }
        //System.out.println("ts is working!!!");
        runControlLoop();
        if (getError() < 1){
        currentState = requestedState;
        }
        else{
        currentState = ArmStates.StateTrans; 
        }
    }

    public void runControlLoop() {
        
        m_talon.setControl(request.withPosition(desiredPosition));
        
        //System.out.println("ts is working!");
        motorVelocity.setDouble(m_talon.getVelocity().getValueAsDouble());
     

        motorPosition.setDouble(m_talon.getPosition().getValueAsDouble());
      }
    

   //m_talon.setControl(request.withPosition(desiredPosition)); 
    
    public double getError() {
        return Math.abs(m_talon.getPosition().getValueAsDouble() - desiredPosition);
    }
 
    public void requestState(ArmStates rState) {
        requestedState = rState;
    }

    
}
