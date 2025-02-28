using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
[ExecuteInEditMode]
public class TubeRendererCSV : MonoBehaviour
{
    public string csvFile = "simulator_truth110_trimmed.csv"; // File in StreamingAssets
    [Min(1)] public int subdivisions = 3;
    [Min(0)] public int segments = 8;
    [Min(0)] public float startWidth = 0.1f;
    [Min(0)] public float endWidth = 0.1f;
    public Vector2 uvScale = Vector2.one;
    public bool inside = false;
    
    public Material tubeMaterial; // Assign this in Inspector
    public Color tubeColor = Color.green; // Set tube color in Inspector

    private Vector3[] positions;
    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;
    private Mesh mesh = null;
    private float theta = 0f;
    private int lastUpdate = 0;

    private void Start()
    {
        LoadPositionsFromCSV();
        GenerateTube();
    }

    private void LoadPositionsFromCSV()
    {
        List<Vector3> loadedPositions = new List<Vector3>();
        string path = Path.Combine(Application.streamingAssetsPath, csvFile);

        if (!File.Exists(path))
        {
            Debug.LogError($"CSV file not found: {path}");
            return;
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

                float csvX = float.Parse(values[1]);
                float csvY = float.Parse(values[2]);
                float csvZ = float.Parse(values[3]);

                // Convert to Unity’s left-handed coordinate system
                Vector3 unityPosition = new Vector3(csvY, csvZ, csvX);
                loadedPositions.Add(unityPosition);
            }
        }
        positions = loadedPositions.ToArray();
    }

    private void GenerateTube()
    {
        if (positions == null || positions.Length == 0) return;

        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        if (mesh == null) mesh = new Mesh();
        meshFilter.mesh = CreateMesh();

        // Assign material and color
        if (tubeMaterial != null)
        {
            meshRenderer.material = tubeMaterial;
            meshRenderer.material.color = tubeColor;
        }

        lastUpdate = PropHashCode();
    }

    private Mesh CreateMesh()
    {
        if (positions == null || positions.Length < 2) return null;

        Vector3[] interpolatedPositions = Enumerable.Range(0, (positions.Length - 1) * subdivisions)
            .Select(i => ((float)i) / ((float)subdivisions))
            .Select(f => GetPosition(f))
            .Append(positions.Last())
            .ToArray();

        theta = (Mathf.PI * 2) / segments;

        Vector3[] verts = new Vector3[interpolatedPositions.Length * segments];
        Vector2[] uvs = new Vector2[verts.Length];
        Vector3[] normals = new Vector3[verts.Length];
        int[] tris = new int[2 * 3 * verts.Length];

        for (int i = 0; i < interpolatedPositions.Length; i++)
        {
            float dia = Mathf.Lerp(startWidth, endWidth, (float)i / interpolatedPositions.Length);

            Vector3 localForward = GetVertexFwd(interpolatedPositions, i);
            Vector3 localUp = Vector3.Cross(localForward, Vector3.up);
            Vector3 localRight = Vector3.Cross(localForward, localUp);

            for (int j = 0; j < segments; ++j)
            {
                float t = theta * j;
                Vector3 vert = interpolatedPositions[i] + (Mathf.Sin(t) * localUp * dia) + (Mathf.Cos(t) * localRight * dia);
                int x = i * segments + j;
                verts[x] = vert;
                uvs[x] = uvScale * new Vector2(t / (Mathf.PI * 2), ((float)i * positions.Length) / (float)subdivisions);
                normals[x] = (vert - interpolatedPositions[i]).normalized;

                if (i >= interpolatedPositions.Length - 1) continue;

                if (inside) normals[x] = -normals[x];
                if (inside)
                {
                    tris[x * 6] = x;
                    tris[x * 6 + 1] = x + segments;
                    tris[x * 6 + 2] = x + 1;

                    tris[x * 6 + 3] = x;
                    tris[x * 6 + 4] = x + segments - 1;
                    tris[x * 6 + 5] = x + segments;
                }
                else
                {
                    tris[x * 6] = x + 1;
                    tris[x * 6 + 1] = x + segments;
                    tris[x * 6 + 2] = x;

                    tris[x * 6 + 3] = x + segments;
                    tris[x * 6 + 4] = x + segments - 1;
                    tris[x * 6 + 5] = x;
                }
            }
        }

        mesh.Clear();
        mesh.vertices = verts;
        mesh.uv = uvs;
        mesh.normals = normals;
        mesh.SetTriangles(tris, 0);
        mesh.RecalculateBounds();
        return mesh;
    }

    private Vector3 GetPosition(float f)
    {
        int a = Math.Max(0, Math.Min(positions.Length - 1, Mathf.FloorToInt(f)));
        int b = Math.Max(0, Math.Min(positions.Length - 1, Mathf.CeilToInt(f)));
        float t = f - a;
        return Vector3.Lerp(positions[a], positions[b], t);
    }

    private Vector3 GetVertexFwd(Vector3[] positions, int i)
    {
        Vector3 lastPosition;
        Vector3 thisPosition;
        if (i <= 0)
        {
            lastPosition = positions[i];
        }
        else
        {
            lastPosition = positions[i - 1];
        }
        if (i < positions.Length - 1)
        {
            thisPosition = positions[i + 1];
        }
        else
        {
            thisPosition = positions[i];
        }
        return (lastPosition - thisPosition).normalized;
    }

    private int PropHashCode()
    {
        return positions.Aggregate(0, (total, it) => total ^ it.GetHashCode()) ^ positions.GetHashCode() ^ segments.GetHashCode() ^ subdivisions.GetHashCode() ^ startWidth.GetHashCode() ^ endWidth.GetHashCode();
    }

    private void LateUpdate()
    {
        if (lastUpdate != PropHashCode())
        {
            GenerateTube();
        }
    }
}
