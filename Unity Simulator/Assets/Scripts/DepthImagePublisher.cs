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

public class DepthImagePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string ImagetopicName = "depth_camera/image_raw";
    public string cameraInfoTopicName = "depth_camera/camera_info";
    private Camera ImageCamera;
    public string FrameId = "depth_frame";
    public int resolutionWidth = 1280;
    public int resolutionHeight = 720;
    private Texture2D texture2D;
    private Rect rect;
    public float publishMessageFrequency = 0.01f;
    private float timeElapsed;
    private uint seq_num = 0;
    private float currentRosTime = 0.0f;
    private float previousRosTime = 0.0f;

    // Shader files
    public Shader blurShader;
    public Shader effectsShader;
    private Material blurMaterial;

    void Start()
    {
        ImageCamera = GetComponent<Camera>(); 
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(ImagetopicName);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopicName);
        ros.Subscribe<RosClock>("/clock", UpdateClock);

        blurMaterial = new Material(blurShader);

        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.R16, false); // R16 for 16-bit grayscale
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24, RenderTextureFormat.R16, RenderTextureReadWrite.Linear);

        ImageCamera.SetReplacementShader(effectsShader, "");
        ImageCamera.clearFlags = CameraClearFlags.SolidColor;
        ImageCamera.backgroundColor = Color.white;

        Camera.onPostRender += UpdateImage;
    }

    void UpdateClock(RosClock clockMessage)
    {
        float seconds = clockMessage.clock.sec - 1724706699;
        float nanoseconds = clockMessage.clock.nanosec / 1e9f;
        currentRosTime = seconds + nanoseconds;
    }

    private void UpdateImage(Camera _camera)
    {
        if (texture2D != null && _camera == this.ImageCamera)
            UpdateMessage();
    }

private void UpdateMessage()
{   
    timeElapsed = currentRosTime - previousRosTime;
    
    if (timeElapsed > publishMessageFrequency)
    {
        previousRosTime = currentRosTime;
        TimeMsg timeStamp = ConvertFloatTimeToRosTimeMsg(currentRosTime);

        // Copy image to texture and flip it vertically
        texture2D.ReadPixels(rect, 0, 0, false);
        texture2D.Apply();
        FlipTextureVertically(texture2D);

        // Get raw image data from the texture
        byte[] rawImageData = texture2D.GetRawTextureData();

        // Convert the byte array to a ushort array
        ushort[] imageDataUshort = new ushort[rawImageData.Length / 2];
        Buffer.BlockCopy(rawImageData, 0, imageDataUshort, 0, rawImageData.Length);

        // Convert ushort array back to byte array for publishing
        byte[] imageData = new byte[imageDataUshort.Length * 2];
        Buffer.BlockCopy(imageDataUshort, 0, imageData, 0, imageData.Length);

        // Create the Image message
        ImageMsg imageMessage = new ImageMsg
        {
            header = new HeaderMsg
            {
                seq = seq_num,
                frame_id = FrameId,
                stamp = timeStamp
            },
            height = (uint)resolutionHeight,
            width = (uint)resolutionWidth,
            encoding = "16UC1", // 16-bit unsigned single channel
            is_bigendian = 0,
            step = (uint)(resolutionWidth * 2), // 2 bytes per pixel for 16UC1
            data = imageData
        };
        seq_num += 1;

        // Publish the image message
        ros.Publish(ImagetopicName, imageMessage);

        // Publish Camera Info message
        CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(ImageCamera, imageMessage.header, 0.0f, 0.01f);
        ros.Publish(cameraInfoTopicName, cameraInfoMessage);
    }
}


    private void FlipTextureVertically(Texture2D original)
    {
        Color[] originalPixels = original.GetPixels();
        Color[] flippedPixels = new Color[originalPixels.Length];

        int width = original.width;
        int height = original.height;

        for (int y = 0; y < height; y++)
        {
            Array.Copy(originalPixels, y * width, flippedPixels, (height - y - 1) * width, width);
        }

        original.SetPixels(flippedPixels);
        original.Apply();
    }

    private TimeMsg ConvertFloatTimeToRosTimeMsg(float timeInSeconds)
    {
        return new TimeMsg
        {
            sec = (uint)Mathf.FloorToInt(timeInSeconds),
            nanosec = (uint)Mathf.FloorToInt((timeInSeconds - Mathf.Floor(timeInSeconds)) * 1e9f)
        };
    }
}
