using UnityEngine;
using System.Collections;
using System.IO;  // Required for working with file directories

public class CameraCaptureMove : MonoBehaviour
{
    public string screenshotFolder = "Screenshots";  // Folder name to store screenshots
    public string screenshotName = "groundPic";  // Base name for the screenshots
    public float moveDistance = 0.1f;  // Distance to move the camera in the X direction
    private int screenshotCount = 0;  // Keeps track of screenshot numbers

    void Start()
    {
        // Create the screenshot folder if it doesn't exist
        string folderPath = Path.Combine(Application.dataPath, screenshotFolder);
        if (!Directory.Exists(folderPath))
        {
            Directory.CreateDirectory(folderPath);
        }

        // Start the coroutine that takes pictures and moves the camera continuously
        StartCoroutine(CaptureAndMove(folderPath));
    }

    // Coroutine to take screenshots and move the camera
    IEnumerator CaptureAndMove(string folderPath)
    {
        while (true)  // Infinite loop to keep taking screenshots and moving the camera
        {
            TakeScreenshot(folderPath);
            MoveCamera();
            
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

    // Function to move the camera in the X direction
    void MoveCamera()
    {
        transform.position += new Vector3(moveDistance, 0, 0);  // Move 0.1 meters in the X direction
        Debug.Log("Camera moved to position: " + transform.position);
    }
}
