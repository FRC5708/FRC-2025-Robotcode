package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Uses the CameraServer class to automatically capture video from a USB webcam
 * and send it to the
 * FRC dashboard without doing any vision processing. This is the easiest way to
 * get camera images
 * to the dashboard. Just add this to the robot class constructor.
 */
public class SensorSubsystem extends SubsystemBase {
    Thread m_visionThread;

    AnalogInput distenceSensor = new AnalogInput(0);

    public SensorSubsystem() {
        m_visionThread = new Thread(
                () -> {
                    // Get the UsbCamera from CameraServer
                    UsbCamera camera = CameraServer.startAutomaticCapture();
                    // Set the resolution
                    camera.setResolution(640, 480);

                    // Get a CvSink. This will capture Mats from the camera
                    CvSink cvSink = CameraServer.getVideo();
                    // Setup a CvSource. This will send images back to the Dashboard
                    CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

                    // Mats are very memory expensive. Lets reuse this Mat.
                    Mat mat = new Mat();

                    // This cannot be 'true'. The program will never exit if it is. This
                    // lets the robot stop this thread when restarting robot code or
                    // deploying.
                    while (!Thread.interrupted()) {
                        // Tell the CvSink to grab a frame from the camera and put it
                        // in the source mat. If there is an error notify the output.
                        if (cvSink.grabFrame(mat) == 0) {
                            // Send the output the error.
                            outputStream.notifyError(cvSink.getError());
                            // skip the rest of the current iteration
                            continue;
                        }
                        // Put a rectangle on the image
                        Imgproc.rectangle(
                                mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                        // Give the output stream a new image to display
                        outputStream.putFrame(mat);
                    }
                });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }

    public double checkSensors() {
        SmartDashboard.putNumber("Distence Sensor", (1 / ((distenceSensor.getVoltage() * 1000) - 1125) / 137500));
        return distenceSensor.getVoltage();
    }
}
