package frc.robot;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public class CameraPoseConvertor {
    private AprilTagFieldLayout m_tags;
    private Transform3d m_robotToCamera = new Transform3d();
    private CoordinateSystem m_cameraCoordinateSystem = CoordinateSystem.NWU();
    private double m_cameraDistanceUnit = 1.0;

    public CameraPoseConvertor(AprilTagFieldLayout tags) {
        m_tags = tags;
    }

    // Use this to locate camera within robot's co-ordinate frame
    public CameraPoseConvertor setRobotToCamera(Transform3d transform) {
        m_robotToCamera = transform;
        return this;
    }

    // Use this if the camera is, say, using a EAST-DOWN-UP co-ordinate frame
    public CameraPoseConvertor setCameraCoordinateSystem(CoordinateSystem system) {
        m_cameraCoordinateSystem = system;
        return this;
    }

    // Use this is the camera co-ordinates aren't in metres
    public CameraPoseConvertor setCameraDistanceUnit(double unit) {
        m_cameraDistanceUnit = unit;
        return this;
    }

    static String transformToString(Transform3d t) {
        Rotation3d r = t.getRotation();
        return String.format("t=%s, r=(%f, %f, %f), d=%f", t.getTranslation().toString(), r.getX() * 180/Math.PI, r.getY() * 180/Math.PI, r.getZ() * 180/Math.PI, t.getTranslation().getNorm());
    }

    static String poseToString(Pose3d t) {
        Rotation3d r = t.getRotation();
        return String.format("t=%s, r=(%f, %f, %f), d=%f", t.getTranslation().toString(), r.getX() * 180/Math.PI, r.getY() * 180/Math.PI, r.getZ() * 180/Math.PI, t.getTranslation().getNorm());
    }

    // Convert camera's tvec and rvec into robot's position on field
    // tvec and rvec are in the camera's co-ordinate system
    // rev uses Rodriques parameters
    public Pose3d convert(int tag, 
        double tx, double ty, double tz,
        double rx, double ry, double rz) {
        // Get tag position in field co-ordinates
        Optional<Pose3d> tagPoseOptional = m_tags.getTagPose(tag);
        if(tagPoseOptional.isEmpty()) { // No such tag
            return null;
        }
        // axis of rotation from rvec
        Vector<N3> axis = VecBuilder.fill(rx, ry, rz);
        // angle of rotation from rvec
        //double theta = axis.norm();
        double theta = Math.sqrt(axis.get(0,0) * axis.get(0,0) + 
            axis.get(1,0) * axis.get(1,0) + axis.get(2,0) * axis.get(2,0));
        System.out.println(String.format("theta=%f", theta));
        Transform3d cameraToTag1 = new Transform3d(
            new Translation3d(tx * m_cameraDistanceUnit, ty * m_cameraDistanceUnit, tz * m_cameraDistanceUnit),
            new Rotation3d(axis, theta)
        );
        System.out.println(String.format("cameraToTag1=%s", transformToString(cameraToTag1)));

        // Transform to NORTH-WEST-UP co-ordinate system, FRC convention
        Transform3d cameraToTag2 = CoordinateSystem.convert(
            cameraToTag1,
            m_cameraCoordinateSystem,
            CoordinateSystem.NWU()
        );
        System.out.println(String.format("cameraToTag2=%s", transformToString(cameraToTag2)));
        // Find transform from robot to tag
        Transform3d robotToTag = m_robotToCamera.plus(cameraToTag2);
        // Invert robot-to-tag transform to get tag-to-robot
        // Add tag position on field to tag-to-robot
        // to get robot position on field
        System.out.println(String.format("robotToTag=%s", transformToString(robotToTag)));
        System.out.println(String.format("robotToTagInverse=%s", transformToString(robotToTag.inverse())));
        System.out.println(String.format("tagPose=%s", poseToString(tagPoseOptional.get())));
        return tagPoseOptional.get().plus(robotToTag.inverse());
    }

    public static void main(String[] args) throws IOException {
        System.out.println("*************************************");
        Transform3d robotToCamera = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
        );

        //String apriltagsPath = CameraPoseConvertor.class.getClassLoader().getResource("edu/wpi/first/apriltag/2022-rapidreact.json").getPath();
        String apriltagsPath = args[0];
        System.out.println(apriltagsPath);
        AprilTagFieldLayout tags = new AprilTagFieldLayout(apriltagsPath);
        System.out.println(String.format("tag0pose=%s", tags.getTagPose(0).get().toString()));
        CameraPoseConvertor cameraPoseConvertor = new CameraPoseConvertor(tags)
            .setRobotToCamera(robotToCamera) // camera is not at centre of robot
            .setCameraCoordinateSystem(CoordinateSystem.EDN()) // Camera uses EAST-DOWN-UP
            .setCameraDistanceUnit(0.0254); // Camera uses inches
        // Take tvec and rvec from camera and convert to robot position on field
        Pose3d robotPosition3d = cameraPoseConvertor.convert(0, -1.2, -2.4, 47, -0.06, -0.05, -0.01);
        System.out.println(String.format("robotPosition3d=%s", poseToString(robotPosition3d)));
        if(robotPosition3d != null) {
            // Convert 3d pose to 2d pose
            Pose2d robotPosition2d = new Pose2d(
                robotPosition3d.getX(),
                robotPosition3d.getY(),
                new Rotation2d(robotPosition3d.getRotation().getZ())
            );
            System.out.println(String.format("robotPosition2d=%s", robotPosition2d.toString()));
            // Update estimator with position from camera
            // m_estimator.addVisionMeasurment(
            //     robotPosition2d,
            //     timestampSeconds // when picture was taken
            // );
        }
    }
}