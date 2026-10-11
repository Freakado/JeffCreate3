using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ArtifactReceiver : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(
            "/JeffCreate3/discovered_artifact",
            OnArtifactDiscovered
        );
    }

    void OnArtifactDiscovered(StringMsg msg)
    {
        // Parse JSON: {"tag_id": 1, "payload": "https://...", "map_x": 1.2, "map_y": 0.8}
        ArtifactData data = JsonUtility.FromJson<ArtifactData>(msg.data);
        Debug.Log($"Artifact {data.tag_id} Found! Link: {data.payload}");

        // Spawn UI card in VR over coordinates
        SpawnMarkerCard(data.map_x, data.map_y, data.payload);
    }
}

[System.Serializable]
public class ArtifactData
{
    public int tag_id;
    public string payload;
    public float map_x;
    public float map_y;
}