using UnityEngine;
using UnityEditor;

public class PlaneMeshGenerator : MonoBehaviour {
    public int rows = 10;
    public int columns = 10;

    [ContextMenu("Generate Mesh")]
    public void GenerateMesh() {
        Mesh mesh = new Mesh();
        Vector3[] vertices = new Vector3[(rows + 1) * (columns + 1)];
        int[] triangles = new int[rows * columns * 6];

        // Create vertices
        for (int i = 0; i <= rows; i++) {
            for (int j = 0; j <= columns; j++) {
                vertices[i * (columns + 1) + j] = new Vector3(j, 0, i);
            }
        }

        // Create triangles
        int triangleIndex = 0;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                int vertexIndex = i * (columns + 1) + j;

                triangles[triangleIndex++] = vertexIndex;
                triangles[triangleIndex++] = vertexIndex + columns + 1;
                triangles[triangleIndex++] = vertexIndex + 1;

                triangles[triangleIndex++] = vertexIndex + 1;
                triangles[triangleIndex++] = vertexIndex + columns + 1;
                triangles[triangleIndex++] = vertexIndex + columns + 2;
            }
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        SaveMeshAsAsset(mesh);
    }

    private void SaveMeshAsAsset(Mesh mesh) {
        string path = $"Assets/GeneratedMesh{rows}x{columns}.asset";
        AssetDatabase.CreateAsset(mesh, path);
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log("Mesh saved at: " + path);
    }
}