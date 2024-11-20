using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosPose = RosMessageTypes.HiperlabRostools.Simulator_truthMsg;
public class SimPoseSubscriber : MonoBehaviour
{
    // Whichever object this script is attatched to is the object that will move in space
    public int vehicleID = 1; // Should manually set vehicle ID, consistent with simulation node
    // A list of things from the message we care about
    private double posx;
    private double posy;
    private double posz;
    private double q0;
    private double q1;
    private double q2;
    private double q3;

    // Start is called before the first frame update
    void Start()
    {   
        string topicName = $"simulator_truth{vehicleID}";
        ROSConnection.GetOrCreateInstance().Subscribe<RosPose>(topicName, PoseChange);
    }

    void PoseChange(RosPose poseMessage)
    {
        // First we extract what position the drone is located at
        posx = poseMessage.posx;
        posy = poseMessage.posy;
        posz = poseMessage.posz;
        // Then we update where the object is actually located
        Vector3<FLU> rosPos3 = new Vector3<FLU>((float)posx, (float)posy, (float)posz);
        Vector3 unityPos = rosPos3.toUnity;
        transform.position = unityPos;

        // Get attitude from ros message
        q0 = poseMessage.attq0;
        q1 = poseMessage.attq1;
        q2 = poseMessage.attq2;
        q3 = poseMessage.attq3;
        Quaternion<FLU> rosRot = new Quaternion<FLU>((float)q1, (float)q2, (float)q3, (float)q0);
        Quaternion unityRot = rosRot.toUnity;
        transform.rotation = unityRot;
    }
}
