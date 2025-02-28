using System;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosClock = RosMessageTypes.Rosgraph.ClockMsg;

public class RGBImagePub : MonoBehaviour
{
    ROSConnection ros;
    public string ImagetopicName = "/d455/infra2/image_rect_raw";
    public string cameraInfoTopicName = "/d455/infra2/camera_info";
    private Camera ImageCamera;
    public string FrameId = "d455_frame";
    public int resolutionWidth = 848;
    public int resolutionHeight = 480;
    [Range(0, 100)] public int qualityLevel = 100;
    private Texture2D texture2D;
    private Rect rect;
    private float currentRosTime = 0.0f;
    private float previousRosTime = 0.0f;
    private uint seq_num = 0;
    
    void Start()
    {   
        ImageCamera = GetComponent<Camera>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(ImagetopicName);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopicName);
        ros.Subscribe<RosClock>("/clock", UpdateClock);
        
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        Camera.onPostRender += UpdateImage;
    }

    void UpdateClock(RosClock clockMessage)
    {   
        float seconds = clockMessage.clock.sec;
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
        previousRosTime = currentRosTime;
        
        texture2D.ReadPixels(rect, 0, 0);
        texture2D.Apply();
        
        Color32[] pixels = texture2D.GetPixels32();
        byte[] rgbData = new byte[pixels.Length * 3];
        
        for (int i = 0; i < pixels.Length; i++)
        {
            rgbData[i * 3] = pixels[i].r;
            rgbData[i * 3 + 1] = pixels[i].g;
            rgbData[i * 3 + 2] = pixels[i].b;
        }
        
        TimeMsg timeStamp = ConvertFloatTimeToRosTimeMsg(currentRosTime);
        
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
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(resolutionWidth * 3),
            data = rgbData
        };
        seq_num += 1;
        ros.Publish(ImagetopicName, message);
        
        CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(ImageCamera, message.header, 0.0f, 0.01f);
        ros.Publish(cameraInfoTopicName, cameraInfoMessage);
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
