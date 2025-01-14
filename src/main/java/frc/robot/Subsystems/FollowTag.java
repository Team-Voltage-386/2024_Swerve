package frc.robot.Subsystems;

import frc.robot.Utils.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import frc.robot.Subsystems.Limelight;
public class FollowTag {
    
    private NetworkTableEntry ta = new Limelight().ta;
    private NetworkTableEntry tx = new Limelight().tx;
    private double tA = ta.getDouble(0.0);
    private double tX = tx.getDouble(0.0);


    public double calculateLimelightInputX(){

        double desiredXSpeed=0;

        if ((tX > 15) && !((desiredXSpeed > 0.5)||(desiredXSpeed < -0.5))){
            if (desiredXSpeed<0.02){
                desiredXSpeed +=0.03;
            } else {
                desiredXSpeed +=0.01;
            }
        } else if (tX >-15){
            if (desiredXSpeed<0){
                desiredXSpeed -=0.03;
            } else {
                desiredXSpeed -=0.01;
            }
        } else if ((tX < 15) && (tX > - 15)){
            if (desiredXSpeed > 0){
                if (desiredXSpeed > 0.15 || desiredXSpeed > -0.15){
                    desiredXSpeed = desiredXSpeed/4;
                } else if (desiredXSpeed < 0.15 && desiredXSpeed > 0){
                    desiredXSpeed -= 0.01;
                } else if (desiredXSpeed > -0.15 && desiredXSpeed < 0){
                    desiredXSpeed += 0.01;
                }
            }
        }
        

        SmartDashboard.putNumber("followTag tx", desiredXSpeed);
        return desiredXSpeed;
    }


    public FollowTag(){



    }
}
