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
    public static GenericEntry motorVoltage = Shuffleboard.getTab("Arm").add("Voltage", 0.0).getEntry();
    public static GenericEntry setPos = Shuffleboard.getTab("Arm").add("Target", 0.0).getEntry();
    

    private final TalonFX m_talon = new TalonFX(4, "rio");
    
    public Arm(){
        
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0;
        slot0Configs.kG = 0;

        slot0Configs.kV = 0.11;
        slot0Configs.kA = 0.005;

        slot0Configs.kP = 2.2;
        slot0Configs.kI = 0.4;
        slot0Configs.kD = 0.32;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 13.6;
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
        m_talon.setPosition(0);
        
    }

    @Override
    public void periodic(){
        switch(requestedState){
            case StateZero:
                //System.out.println("StateZero");
                //m_talon.set(1)
                desiredPosition = 0;
                break;
            case StateIdle:
                if(desiredPosition < 0.02){
                    requestedState = ArmStates.StateZero;
                    break;    
                }       
                //System.out.println("StateIdle");
                desiredPosition -= 0.01;
                break;    
            case StateMax:
                //System.out.println("StateMax");
                desiredPosition = 0.769;
                // m_talon.setVoltage(0.2);
                break;
            /*default:
                desiredPosition = 0.2;
                break;*/
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
     
        setPos.setDouble(desiredPosition);

        motorVoltage.setDouble(m_talon.getMotorVoltage().getValueAsDouble());

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
