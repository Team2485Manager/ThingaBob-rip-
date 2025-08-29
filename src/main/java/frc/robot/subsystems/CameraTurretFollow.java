package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraTurretFollow extends SubsystemBase{
    PhotonCamera camera = new PhotonCamera("cam1");

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
        boolean targetVisible = false;

        double targetYaw = 0.0;

        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            // Camera processed a new frame since last

            // Get the last one in the list.

            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                var target = result.getTargets().get(0);
                targetYaw = target.getYaw();
                targetVisible = true;
                System.out.println("Degrees:" + targetYaw + ", Radians:" + targetYaw/180 * Math.PI);
            }
        }
    }
}
