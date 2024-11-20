using UnityEngine;
using System.Collections;
using System.IO;

public class CameraCaptureMoveCircular : MonoBehaviour
{
    public string screenshotFolder = "Screenshots";  // Folder name to store screenshots
    public string screenshotName = "screenshot";  // Base name for the screenshots
    public Transform targetPoint;  // The point the camera circles around
    public float radius = 5f;  // Radius of the circle around the point
    public int framesPerRotation = 360;  // Number of frames to complete one full rotation
    public float heightOffset = 2f;  // Height offset to look downward
    public float downwardLookAngle = 10f;  // Degree by which the camera looks down at the target point
    private int screenshotCount = 0;  // Keeps track of screenshot numbers

    void Start()
    {
        // Set the folder path to the Assets folder
        string folderPath = Path.Combine(Application.dataPath, screenshotFolder);
        
        // Create the screenshot folder if it doesn't exist
        if (!Directory.Exists(folderPath))
        {
            Directory.CreateDirectory(folderPath);
        }

        // Start the coroutine that takes pictures and moves the camera continuously
        StartCoroutine(CaptureAndMove(folderPath));
    }

    IEnumerator CaptureAndMove(string folderPath)
    {
        while (true)  // Infinite loop to keep taking screenshots and moving the camera
        {
            TakeScreenshot(folderPath);
            MoveCameraInCircle();
            
            // Wait until the end of the frame before taking the next screenshot
            yield return new WaitForEndOfFrame();
        }
    }

    // Function to take a screenshot and save it to the specified folder
    void TakeScreenshot(string folderPath)
    {
        string fileName = Path.Combine(folderPath, screenshotName + "_" + screenshotCount + ".png");
        ScreenCapture.CaptureScreenshot(fileName);
        Debug.Log("Screenshot taken: " + fileName);
        screenshotCount++;  // Increment the screenshot count for the next filename
    }

    // Function to move the camera in a circular path around the target point based on frame count
    void MoveCameraInCircle()
    {
        // Calculate the current angle based on the frame number
        float currentAngle = (Time.frameCount % framesPerRotation) * (360f / framesPerRotation);
        
        // Convert the angle to radians for trigonometric calculations
        float angleInRadians = currentAngle * Mathf.Deg2Rad;

        // Calculate the new position based on the angle
        float x = Mathf.Cos(angleInRadians) * radius;
        float z = Mathf.Sin(angleInRadians) * radius;

        // Update the camera's position with a height offset to create a downward look effect
        transform.position = new Vector3(x, targetPoint.position.y + heightOffset, z) + targetPoint.position;

        // Calculate the downward look direction by applying the angle
        Vector3 lookDirection = targetPoint.position - transform.position;
        lookDirection.y -= Mathf.Tan(downwardLookAngle * Mathf.Deg2Rad) * radius;

        // Ensure the camera is always looking slightly downward at the target point
        transform.rotation = Quaternion.LookRotation(lookDirection);

        Debug.Log("Camera moved to position: " + transform.position);
    }
}
