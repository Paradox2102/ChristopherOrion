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

    // Rodrigues paranmeters represent a rotation as a 3-vector where the
    // direction is the axis of rotation and the magnitude is tan(theta/2).
    static Rotation3d rodriguesToRotation3d(double x, double y, double z) {
        Vector<N3> axis = VecBuilder.fill(x, y, z);
        // angle of rotation from rvec
        double theta = 2 * Math.atan(Math.sqrt(x*x + y*y + z*z));
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

        Transform3d cameraToTag2 = CoordinateSystem.convert(
            cameraToTag, 
            m_cameraCoordinateSystem,
            CoordinateSystem.NWU()
        );
        System.out.println(String.format("cameraToTag2=%s", transformToString(cameraToTag2)));

        Transform3d cameraToTag3 = cameraToTag2.plus(new Transform3d(new Pose3d(), m_cameraTagPose));
        System.out.println(String.format("cameraToTag3=%s", transformToString(cameraToTag3)));

        // Find transform from robot to tag
        Transform3d robotToTag = m_robotToCamera.plus(cameraToTag3);
        // Invert robot-to-tag transform to get tag-to-robot
        // Add tag position on field to tag-to-robot
        // to get robot position on field
        System.out.println(String.format("robotToTag=%s", transformToString(robotToTag)));
        System.out.println(String.format("robotToTagInverse=%s", transformToString(robotToTag.inverse())));
        System.out.println(String.format("tagPose=%s", poseToString(tagPoseOptional.get())));
        return tagPoseOptional.get().plus(robotToTag.inverse());
    }

    private static final double[][] testData = {
       // rx,ry,rz,tx,ty,tz
        { -0.06,-0.05,-0.01,-1.2,-2.4,47 },
        { -0.07,-0.06,-0.01,-2.9,-2.3,54.8 },
        { -0.06,-0.11,-0.01,-4.8,-2.2,63.4 },
        { -0.07,-0.11,-0.02,-3.1,-2,76 },
        { -0.07,-0.03,-0.02,-4.7,-1.1,87.2 },
        { -0.04,0,-0.02,0.3,-0.6,95.6 },
        { 0,-0.06,-0.02,-0.8,-0.7,99.7 },
        { 0.08,-0.06,-0.01,0.4,-0.8,108.2 },
        { 0.04,-0.02,-0.02,1.4,0.3,117.7 },
        { -0.1,-0.08,-0.02,-0.5,0.6,136.8 },
        { -0.1,-0.77,-0.04,-14.7,-2.3,69.7 }
    };

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
            .setCameraDistanceUnit(0.0254) // Camera uses inches
            .setCameraTagPose(new Pose3d( // Tag points backwards
                new Translation3d(), 
                new Rotation3d(-Math.PI/2, Math.PI, 0)
            ));
        // Take tvec and rvec from camera and convert to robot position on field
        //Pose3d robotPosition3d = cameraPoseConvertor.convert(0, -1.2, -2.4, 47, -0.06, -0.05, -0.01);
        Pose3d robotPosition3d = cameraPoseConvertor.convert(0, -14.7,-2.3,69.7, -0.10,-0.77,-0.04);
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