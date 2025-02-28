using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosClock = RosMessageTypes.Rosgraph.ClockMsg;


public class StereoImagePub : MonoBehaviour
{
    ROSConnection ros;
    public string ImagetopicName = "/d455/infra2/image_rect_raw";
    public string cameraInfoTopicName = "/d455/infra2/camera_info";
    // The game object
    private Camera ImageCamera;
    public string FrameId = "d455_frame";
    public int resolutionWidth = 848;
    public int resolutionHeight = 480;
    [Range(0, 100)]
    public int qualityLevel = 100;
    private Texture2D texture2D;
    private Rect rect;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 15.0f;
    
    private float publishMessageTime;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    private uint seq_num = 0;
    private ImageMsg message;
    private float currentRosTime = 0.0f;
    private float previousRosTime = 0.0f;

    void Start()
    {   
        publishMessageTime = 1 / publishMessageFrequency;
        // Gets the camera this file is attatched to
        ImageCamera = GetComponent<Camera>(); 
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(ImagetopicName);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopicName);
        ros.Subscribe<RosClock>("/clock",UpdateClock);

        // Initialize a texture to use
        texture2D = new Texture2D(resolutionWidth, resolutionHeight,TextureFormat.RGB24, false);
        // This is used for pixel reading later
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        // Sets the camera's target to a new rendering texture
        ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        // Update the image sent to ROS everytime the camera finishes rendering
        Camera.onPostRender += UpdateImage;
    }
    // Subscribing to clock time
    void UpdateClock(RosClock clockMessage)
    {   
        float seconds = clockMessage.clock.sec;
        float nanoseconds = clockMessage.clock.nanosec / 1e9f;
        currentRosTime = seconds + nanoseconds;

        // Debug.Log($"Seconds: {seconds}, Nanoseconds: {nanoseconds}, Current ROS Time (seconds): {currentRosTime}");
    }
    private void UpdateImage(Camera _camera)
    {
        if (texture2D != null && _camera == this.ImageCamera)
            UpdateMessage();
    }

    private void UpdateMessage()
{
    timeElapsed = currentRosTime - previousRosTime;

    // if (timeElapsed > publishMessageTime)
    // {
        timeElapsed = 0;
        previousRosTime = currentRosTime;

        // Copy image to texture
        texture2D.ReadPixels(rect, 0, 0);
        texture2D.Apply();

        // Flip the texture vertically
        Color32[] pixels = texture2D.GetPixels32();
        Color32[] flippedPixels = new Color32[pixels.Length];
        int width = texture2D.width;
        int height = texture2D.height;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                flippedPixels[x + y * width] = pixels[x + (height - y - 1) * width];
            }
        }

        // Convert RGB image to grayscale
        byte[] grayscaleData = new byte[flippedPixels.Length];
        for (int i = 0; i < flippedPixels.Length; i++)
        {
            // Use luminance formula to convert to grayscale
            grayscaleData[i] = (byte)(0.299f * flippedPixels[i].r + 0.587f * flippedPixels[i].g + 0.114f * flippedPixels[i].b);
        }

        TimeMsg timeStamp = ConvertFloatTimeToRosTimeMsg(currentRosTime);

        // Message
        ImageMsg message = new ImageMsg
        {
            header = new HeaderMsg
            {
                seq = seq_num,
                frame_id = FrameId,
                stamp = timeStamp
            },
            height = (uint)resolutionHeight,
            width = (uint)resolutionWidth,
            encoding = "mono8",
            is_bigendian = 0,
            step = (uint)resolutionWidth,
            data = grayscaleData
        };
        seq_num += 1;

        // Publish the message
        ros.Publish(ImagetopicName, message);

        // Camera Info message
        CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(ImageCamera, message.header, 0.0f, 0.01f);
        ros.Publish(cameraInfoTopicName, cameraInfoMessage);
    // }
}
    private TimeMsg ConvertFloatTimeToRosTimeMsg(float timeInSeconds)
{
    // Convert directly to ROS message without extra variables
    return new TimeMsg
    {
        sec = (uint)Mathf.FloorToInt(timeInSeconds),
        nanosec = (uint)Mathf.FloorToInt((timeInSeconds - Mathf.Floor(timeInSeconds)) * 1e9f)
    };
}
}