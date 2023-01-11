package frc.robot;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
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
    private Pose3d m_cameraTagPose = new Pose3d();
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

    // Use this if the camera's idea of the tag centre isn't at origin, upright, facing +X
    public CameraPoseConvertor setCameraTagPose(Pose3d pose) {
        this.m_cameraTagPose = pose;
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

    // We don't convert the usual way because the camera solved the tag location using points in the
    // camera co-ordinate system.  This means that we need to interpret the rotation specially,
    static Transform3d convertCoordinateSystem(Transform3d transform, CoordinateSystem from, CoordinateSystem to) {
        Translation3d translation = CoordinateSystem.convert(transform.getTranslation(), from, to);
        Rotation3d rotation = transform.getRotation();
        return new Transform3d(translation, rotation);
    }

    // Rodrigues paranmeters represent a rotation as a 3-vector where the
    // direction is the axis of rotation and the magnitude is tan(theta/2).
    static Rotation3d rodriguesToRotation3d(double x, double y, double z) {
        Vector<N3> axis = VecBuilder.fill(x, y, z);
        // angle of rotation from rvec
        double theta = Math.sqrt(x*x + y*y + z*z);
        return new Rotation3d(axis, theta);
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

        Transform3d cameraToTag = new Transform3d(
            new Translation3d(tx, ty, tz).times(m_cameraDistanceUnit),
            rodriguesToRotation3d(rx, ry, rz)
        );
        System.out.println(String.format("cameraToTag=%s", transformToString(cameraToTag)));

        Transform3d cameraToTag2 = convertCoordinateSystem(
            cameraToTag, 
            m_cameraCoordinateSystem,
            CoordinateSystem.NWU()
        );
        System.out.println(String.format("cameraToTag2=%s", transformToString(cameraToTag2)));

        Transform3d cameraToTag3 = cameraToTag2.plus(new Transform3d(new Pose3d(), m_cameraTagPose));
        System.out.println(String.format("cameraToTag3=%s", transformToString(cameraToTag3)));

        // Find transform from robot to tag
        Transform3d robotToTag = m_robotToCamera.inverse().plus(cameraToTag3);
        // Invert robot-to-tag transform to get tag-to-robot
        // Add tag position on field to tag-to-robot
        // to get robot position on field
        System.out.println(String.format("robotToTag=%s", transformToString(robotToTag)));
        System.out.println(String.format("robotToTagInverse=%s", transformToString(robotToTag.inverse())));
        System.out.println(String.format("tagPose=%s", poseToString(tagPoseOptional.get())));
        return tagPoseOptional.get().plus(robotToTag.inverse());
    }

    private static final double[][] testData = {
        // dx, dy, dz in camera frame (EDN)
        // rvec, tvec in camera frame (EDN) 
        // All distances in inches
        { 8.75,0,102.5,-0.09,0.1,0.02,8.6,0.8,102.6 },
        { 11,0,102.5,-0.03,0.03,-0.02,10.5,0.8,102.4 },
        { 14.5,0,102.5,0,0.01,-0.03,13.8,0.8,102.2 },
        { 17.25,0,102.5,-0.04,0.01,-0.03,17.6,0.9,102.5 },
        { 22.74,0,102.5,0.01,0.39,0.01,21.8,1,101.5 },
        { 26.5,0,102.5,0.02,0.47,0.02,25.7,1.1,102.2 },
        { 30.25,0,102.5,-0.06,0,0,29.3,1.4,101.3 },
        { 34.5,0,102.5,-0.08,0,0,33.4,1.4,101.7 },
        { 0,-11,102.5,-0.04,0.03,0.03,0.3,-10.2,103.4 },
        { 0,-13.75,102.5,-0.07,0.05,0.01,0.2,-13,103.8 },
        { 0,-18,102.5,-0.05,0.01,0.05,0.6,-17.1,104.4 },
        { 0,-22,102.5,-0.05,0.02,0.05,0.7,-21.1,104.2 },    
        { 0,-25.5,102.5,-0.07,0.01,0.08,0.9,-24.6,105.1 },
        { 0,0,102.5,0.02,0.06,0.02,0.2,0.6,102.4 },
        { 0,0,102.5,-0.06,0,0.02,0.2,0.6,102.9 },
        { 0,0,102.5,-0.09,0.06,0.03,0.1,0.6,102.2 },
        { 0,0,102.5,-0.08,0.03,0.03,0.1,0.6,102.2 },  
        { 0,0,102.5,-0.07,0.04,0.02,0.2,0.6,103 },
        { 0,0,102.5,-0.09,0.02,0.03,0.1,0.6,102.7 },
        { 0,0,102.5,-0.11,0.01,0.02,0.1,0.6,103.2 },
        { 0,0,102.5,-0.08,0.03,0.03,0.1,0.6,103 },
        { 0,0,102.5,-0.11,0.05,0.02,0.1,0.6,10 },
        { 0,0,102.5,-0.07,-0.03,0.03,0.1,0.6,102.5 },
        { 0,0,102.5,-0.08,-0.05,0.03,0.1,0.6,102.3 },
        { 0,0,102.5,-0.08,-0.02,0.03,0.1,0.6,102.5 },
        { 0,0,102.5,-0.09,0.01,0.03,0.1,0.6,102.8 }
    };

    public static void main(String[] args) throws IOException {
        System.out.println("*************************************");
        Transform3d robotToCamera = new Transform3d(
            new Translation3d(0.2, 0, 0.5),
            new Rotation3d(0, 0, 0)
        );

        Pose3d tagPose = new Pose3d(new Translation3d(1,2,3), new Rotation3d());
        AprilTagFieldLayout tags = new AprilTagFieldLayout(Arrays.asList(new AprilTag[]{
            new AprilTag(0, tagPose)}), 16.54175, 8.0137);

        CameraPoseConvertor cameraPoseConvertor = new CameraPoseConvertor(tags)
            .setRobotToCamera(robotToCamera) // camera is not at centre of robot
            .setCameraCoordinateSystem(CoordinateSystem.EDN()) // Camera uses EAST-DOWN-UP
            .setCameraDistanceUnit(0.0254) // Camera uses inches
            .setCameraTagPose(new Pose3d( // Tag points backwards
                new Translation3d(), 
                new Rotation3d(0, 0, Math.PI)
            ));

        for(double[] data : testData) {
            System.out.println("***");
            System.out.println(String.format("data=(dx=%f, dy=%f, dz=%f, rx=%f, ry=%f, rz=%f, tx=%f, ty=%f, tz=%f}", 
                data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]));
            Pose3d robotPosition3d = cameraPoseConvertor.convert(0, 
                data[6], data[7], data[8], // tvec 
                data[3], data[4], data[5] // rvec
                );
            Pose3d expectedPosition = new Pose3d(
                new Translation3d(1 + data[2] * 0.0254 - 0.2, 2 + data[0] * 0.0254, 3 - data[1] * 0.0254 - 0.5),
                new Rotation3d(0, 0, Math.PI)
            );
            System.out.println(String.format("robotPosition3d=%s, expectedPosition=%s", poseToString(robotPosition3d), poseToString(expectedPosition)));

        }
    }
}