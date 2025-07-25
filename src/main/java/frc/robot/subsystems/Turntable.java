package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turntable extends SubsystemBase {
    

    public enum TurntableStates{

        StateJoystickDriven,
        StateZero

    }
    public TurntableStates currentState = TurntableStates.StateJoystickDriven;
    public TurntableStates requestedState = TurntableStates.StateJoystickDriven;


    private final TalonFX m_talon = new TalonFX(3,"rio");
    private PIDController controller = new PIDController(0.05, 0, 0.0001);
    public DoubleSupplier axisSupplier;

    DoublePublisher voltagePub;
    DoublePublisher positionPub;

    public Turntable(DoubleSupplier axisSupplier){
        this.axisSupplier = axisSupplier;
        
        var talonFXConfigs = new TalonFXConfiguration();
            // These will be derived experimentally but in case you are wondering
            // How these terms are defined from the TalonFX docs
            // kS adds n volts to overcome static friction
            // kV outputs n volts when the velocity target is 1 rotation per second
            // kP outputs 12 volts when the positional error is 12/n rotations
            // kI adds n volts per second when the positional error is 1 rotation
            // kD outputs n volts when the velocity error is 1 rotation per second
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.5;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;


        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable=true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit=70;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit=120;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable=true;
  

        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        m_talon.getConfigurator().apply(talonFXConfigs);

        controller.setTolerance(0);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
    
        NetworkTable table = inst.getTable("datatable");

        voltagePub = table.getDoubleTopic("voltage").publish();

        positionPub = table.getDoubleTopic("position").publish();
    }

    double voltage;
    double position;
    // Publish values that are constantly increasing.
    
    
    @Override
    public void periodic(){
        switch(currentState){
            case StateZero:
                double power = controller.calculate(m_talon.getPosition().getValueAsDouble(), 0);
                m_talon.set(power);
                break;
            case StateJoystickDriven:
                m_talon.set(-axisSupplier.getAsDouble()*0.025);
                if((position > 2.5 && -axisSupplier.getAsDouble() > 0) || (position < -2.5 && -axisSupplier.getAsDouble() < 0))
                    m_talon.set(0);
                break;
        }

        currentState = requestedState;
        voltagePub.set(voltage);
        voltage = m_talon.getMotorVoltage().getValueAsDouble();

        positionPub.set(position);
        position = m_talon.getPosition().getValueAsDouble();

    }


    
    
    public void requestState(TurntableStates req){

        requestedState = req;

    }

}
