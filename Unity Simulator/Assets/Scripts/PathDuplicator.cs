using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class PathDuplicator : MonoBehaviour
{
    public string csvFile = "simulator_truth110_trimmed.csv"; // File in StreamingAssets
    public float scaleFactor = 1.0f; // Adjust if needed
    public int maxCopies = 10; // Number of evenly spaced spheres
    public bool destroyOriginal = false; // Remove the first sphere after spawning

    private void Start()
    {
        // Prevent clones from running this script
        if (gameObject.name.Contains("(Clone)")) return;

        List<Vector3> points = ReadCsv(csvFile);
        if (points.Count == 0) return;

        int totalPoints = points.Count;
        if (totalPoints < maxCopies) maxCopies = totalPoints; // Ensure we don’t exceed available points

        // Calculate spacing interval
        int stepSize = totalPoints / maxCopies;

        for (int i = 0; i < maxCopies; i++)
        {
            int index = i * stepSize; // Evenly spaced index
            GameObject clone = Instantiate(gameObject, points[index], Quaternion.identity);
            clone.transform.parent = transform.parent; // Keep hierarchy clean
            Destroy(clone.GetComponent<PathDuplicator>()); // Remove script from clones
        }

        if (destroyOriginal)
        {
            Destroy(gameObject);
        }
    }

    List<Vector3> ReadCsv(string filename)
    {
        List<Vector3> positions = new List<Vector3>();
        string path = Path.Combine(Application.streamingAssetsPath, filename);

        if (!File.Exists(path))
        {
            Debug.LogError($"File not found: {path}");
            return positions;
        }

        using (StreamReader reader = new StreamReader(path))
        {
            bool firstLine = true;
            while (!reader.EndOfStream)
            {
                string line = reader.ReadLine();
                if (firstLine) { firstLine = false; continue; } // Skip header

                string[] values = line.Split(',');
                if (values.Length < 3) continue; // Ensure correct format

                float csvX = float.Parse(values[1]) * scaleFactor;
                float csvY = float.Parse(values[2]) * scaleFactor;
                float csvZ = float.Parse(values[3]) * scaleFactor;

                // Convert to Unity's left-handed coordinate system
                Vector3 unityPosition = new Vector3(-csvY, csvZ, csvX);
                positions.Add(unityPosition);
            }
        }
        return positions;
    }
}
