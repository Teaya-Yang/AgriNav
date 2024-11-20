using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosPose = RosMessageTypes.TestPublisher.VehiclePoseMsg;
public class VehiclePoseSubscriber : MonoBehaviour
{
    // This is the object that will move in space
    // public GameObject vehicle;
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
        ROSConnection.GetOrCreateInstance().Subscribe<RosPose>("pose", PoseChange);
    }

    void PoseChange(RosPose poseMessage)
    {
        // First we extract what position the drone is located at
        posx = poseMessage.posx;
        posy = poseMessage.posy;
        posz = poseMessage.posz;
        // Then we update where the object is actually located
        transform.position = new Vector3((float)posx, (float)posy, (float)posz);

        // Get attitude from ros message
        q0 = poseMessage.attq0;
        q1 = poseMessage.attq1;
        q2 = poseMessage.attq2;
        q3 = poseMessage.attq3;
        transform.rotation = new Quaternion((float)q1, (float)q2, (float)q3, (float)q0);

    }
}
