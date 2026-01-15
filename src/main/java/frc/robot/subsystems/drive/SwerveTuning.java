package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Class for tuning swerve drive PID gains via NetworkTables/Dashboard.
 * This class is separate from the generated TunerConstants to avoid being overwritten.
 */
public class SwerveTuning {
    private final CommandSwerveDrivetrain drivetrain;
    
    // Steer gain entries
    private final NetworkTableEntry steerKP;
    private final NetworkTableEntry steerKI;
    private final NetworkTableEntry steerKD;
    private final NetworkTableEntry steerKS;
    private final NetworkTableEntry steerKV;
    private final NetworkTableEntry steerKA;
    
    // Drive gain entries
    private final NetworkTableEntry driveKP;
    private final NetworkTableEntry driveKI;
    private final NetworkTableEntry driveKD;
    private final NetworkTableEntry driveKS;
    private final NetworkTableEntry driveKV;
    
    // Store previous values to detect changes
    private double prevSteerKP, prevSteerKI, prevSteerKD, prevSteerKS, prevSteerKV, prevSteerKA;
    private double prevDriveKP, prevDriveKI, prevDriveKD, prevDriveKS, prevDriveKV;
    
    public SwerveTuning(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        var table = NetworkTableInstance.getDefault().getTable("SwerveTuning");
        
        // Initialize steer gain entries with current values from TunerConstants
        steerKP = table.getEntry("Steer_kP");
        steerKI = table.getEntry("Steer_kI");
        steerKD = table.getEntry("Steer_kD");
        steerKS = table.getEntry("Steer_kS");
        steerKV = table.getEntry("Steer_kV");
        steerKA = table.getEntry("Steer_kA");
        
        steerKP.setDouble(100);
        steerKI.setDouble(0);
        steerKD.setDouble(0.5);
        steerKS.setDouble(0.1);
        steerKV.setDouble(2.33);
        steerKA.setDouble(0);
        
        // Initialize drive gain entries with current values from TunerConstants
        driveKP = table.getEntry("Drive_kP");
        driveKI = table.getEntry("Drive_kI");
        driveKD = table.getEntry("Drive_kD");
        driveKS = table.getEntry("Drive_kS");
        driveKV = table.getEntry("Drive_kV");
        
        driveKP.setDouble(0.1);
        driveKI.setDouble(0);
        driveKD.setDouble(0);
        driveKS.setDouble(0);
        driveKV.setDouble(0.124);
        
        // Initialize previous values
        updatePreviousValues();
    }
    
    /**
     * Call this method periodically (e.g., in Robot.periodic() or a subsystem's periodic())
     * to check for changes and update motor gains.
     */
    public void periodic() {
        boolean steerChanged = checkSteerGainsChanged();
        boolean driveChanged = checkDriveGainsChanged();
        
        if (steerChanged) {
            applySteerGains();
            updatePreviousValues();
            SmartDashboard.putString("Tuning Status", "Steer gains updated!");
        }
        
        if (driveChanged) {
            applyDriveGains();
            updatePreviousValues();
            SmartDashboard.putString("Tuning Status", "Drive gains updated!");
        }
        
        // Publish current values to dashboard for visibility
        SmartDashboard.putNumber("Steer kP", steerKP.getDouble(100));
        SmartDashboard.putNumber("Drive kP", driveKP.getDouble(0.1));
    }
    
    private boolean checkSteerGainsChanged() {
        return steerKP.getDouble(100) != prevSteerKP ||
               steerKI.getDouble(0) != prevSteerKI ||
               steerKD.getDouble(0.5) != prevSteerKD ||
               steerKS.getDouble(0.1) != prevSteerKS ||
               steerKV.getDouble(2.33) != prevSteerKV ||
               steerKA.getDouble(0) != prevSteerKA;
    }
    
    private boolean checkDriveGainsChanged() {
        return driveKP.getDouble(0.1) != prevDriveKP ||
               driveKI.getDouble(0) != prevDriveKI ||
               driveKD.getDouble(0) != prevDriveKD ||
               driveKS.getDouble(0) != prevDriveKS ||
               driveKV.getDouble(0.124) != prevDriveKV;
    }
    
    private void applySteerGains() {
        Slot0Configs steerGains = new Slot0Configs()
            .withKP(steerKP.getDouble(100))
            .withKI(steerKI.getDouble(0))
            .withKD(steerKD.getDouble(0.5))
            .withKS(steerKS.getDouble(0.1))
            .withKV(steerKV.getDouble(2.33))
            .withKA(steerKA.getDouble(0));
        
        // Apply to all modules
        drivetrain.getModule(0).getSteerMotor().getConfigurator().apply(steerGains);
        drivetrain.getModule(1).getSteerMotor().getConfigurator().apply(steerGains);
        drivetrain.getModule(2).getSteerMotor().getConfigurator().apply(steerGains);
        drivetrain.getModule(3).getSteerMotor().getConfigurator().apply(steerGains);
    }
    
    private void applyDriveGains() {
        Slot0Configs driveGains = new Slot0Configs()
            .withKP(driveKP.getDouble(0.1))
            .withKI(driveKI.getDouble(0))
            .withKD(driveKD.getDouble(0))
            .withKS(driveKS.getDouble(0))
            .withKV(driveKV.getDouble(0.124));
        
        // Apply to all modules
        drivetrain.getModule(0).getDriveMotor().getConfigurator().apply(driveGains);
        drivetrain.getModule(1).getDriveMotor().getConfigurator().apply(driveGains);
        drivetrain.getModule(2).getDriveMotor().getConfigurator().apply(driveGains);
        drivetrain.getModule(3).getDriveMotor().getConfigurator().apply(driveGains);
    }
    
    private void updatePreviousValues() {
        prevSteerKP = steerKP.getDouble(100);
        prevSteerKI = steerKI.getDouble(0);
        prevSteerKD = steerKD.getDouble(0.5);
        prevSteerKS = steerKS.getDouble(0.1);
        prevSteerKV = steerKV.getDouble(2.33);
        prevSteerKA = steerKA.getDouble(0);
        
        prevDriveKP = driveKP.getDouble(0.1);
        prevDriveKI = driveKI.getDouble(0);
        prevDriveKD = driveKD.getDouble(0);
        prevDriveKS = driveKS.getDouble(0);
        prevDriveKV = driveKV.getDouble(0.124);
    }
}