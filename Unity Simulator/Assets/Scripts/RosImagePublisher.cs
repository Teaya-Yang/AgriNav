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


public class RosImagePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string ImagetopicName = "rgb_camera/image_raw/compressed";
    public string cameraInfoTopicName = "rgb_camera/camera_info";
    // The game object
    private Camera ImageCamera;
    public string FrameId = "rgb_frame";
    public int resolutionWidth = 1280;
    public int resolutionHeight = 720;
    [Range(0, 100)]
    public int qualityLevel = 75;
    private Texture2D texture2D;
    private Rect rect;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.01f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    private uint seq_num = 0;
    private CompressedImageMsg message;
    private float currentRosTime = 0.0f;
    private float previousRosTime = 0.0f;

    void Start()
    {   
        // Gets the camera this file is attatched to
        ImageCamera = GetComponent<Camera>(); 
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CompressedImageMsg>(ImagetopicName);
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
        float seconds = clockMessage.clock.sec- 1715380747;
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
        
        // Debug.Log($"Current Time Diff(seconds): {timeElapsed}");

        if (timeElapsed > publishMessageFrequency)
        {
            timeElapsed = 0;
            previousRosTime = currentRosTime;
            // Debug.Log("Time to publish");
            TimeMsg timeStamp = ConvertFloatTimeToRosTimeMsg(currentRosTime);

            // Copy image to texture
            texture2D.ReadPixels(rect, 0, 0);
            texture2D.Apply();

            //Message
            CompressedImageMsg message = new CompressedImageMsg
            {
                header = new HeaderMsg
                {
                    seq = seq_num,
                    frame_id = FrameId,
                    stamp = timeStamp
                },
                format = "jpg",
                data = texture2D.EncodeToJPG(qualityLevel)
            };
            seq_num += 1;

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(ImagetopicName, message);

            //Camera Info message
            CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(ImageCamera, message.header, 0.0f, 0.01f);
            ros.Publish(cameraInfoTopicName, cameraInfoMessage);

        }
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