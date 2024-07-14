using NaughtyAttributes;
using UnityEditor;
using UnityEngine;

namespace HalfEdge {
    public class Playground : MonoBehaviour {
        [SerializeField]
        private MeshFilter meshFilter;

        [SerializeField]
        private HalfEdgeMesh halfEdgeMesh;

        [SerializeField]
        private bool labelHalfEdges = false;

        [SerializeField]
        private bool labelFaces = false;

        [SerializeField]
        private Transform plane;

        [SerializeField]
        private int splitEdge = -1;

        [SerializeField]
        private Transform newEdgePos;

        [Button("Start Bisection Test")]
        public void BisectionTest() {
            halfEdgeMesh = new HalfEdgeMesh(meshFilter.sharedMesh);
            halfEdgeMesh.CombineCoplanarFaces();
        }

        [Button("Bisection Right")]
        public void BisectionRight() {
            halfEdgeMesh.Bisect(Vector3.right, new Vector3(1f, 0f, 1f));
        }

        [Button("Bisection Forward")]
        public void BisectionForward() {
            halfEdgeMesh.Bisect(Vector3.forward, new Vector3(1f, 0f, 1f));
        }

        [Button("Print Plane Info")]
        public void PrintPlaneInfo() {
            if (plane == null) {
                Debug.Log($"No plane to print.");
                return;
            }

            Debug.Log($"Plane position: {plane.position}, forward: {plane.forward}, distance: {-Vector3.Dot(plane.forward, plane.position)}");
        }

        [Button("Test")]
        public void Test() {
            halfEdgeMesh = new HalfEdgeMesh(meshFilter.sharedMesh);
        }

        [Button("Combine")]
        public void Combine() {
            if (halfEdgeMesh == null) {
                Debug.Log($"No mesh structure yet.");
                return;
            }

            halfEdgeMesh.CombineCoplanarFaces();
        }

        [Button("Bisect")]
        public void Bisect() {
            if (halfEdgeMesh == null) {
                Debug.Log($"No mesh structure yet.");
                return;
            }

            if (plane == null) {
                Debug.Log($"No plane to bisect with.");
                return;
            }

            UnityEngine.Debug.Log($"Creating a plane from position {FormatVector3(plane.position)} and forward {FormatVector3(plane.forward)}.");
            halfEdgeMesh.Bisect(new Plane(plane.forward, plane.position));
        }

        public static string FormatVector3(Vector3 vector) {
            return $"({vector.x:F7}, {vector.y:F7}, {vector.z:F7})";
        }

        private void OnDrawGizmos() {
            if (halfEdgeMesh == null) {
                return;
            }

            halfEdgeMesh.DrawGizmos(labelHalfEdges, labelFaces);

            if (plane != null && plane.gameObject.activeInHierarchy) {
                var col = Color.Lerp(Color.blue, Color.clear, 0.2f);

                Gizmos.color = col;
                Gizmos.DrawRay(plane.position, plane.forward);

                var mat = Matrix4x4.TRS(plane.position, Quaternion.LookRotation(plane.forward), Vector3.one);
                const float scale = 3f / 2f;
                var vertices = new[] {
                        mat.MultiplyPoint3x4(new Vector3(scale, scale, 0f)),
                        mat.MultiplyPoint3x4(new Vector3(-scale, scale, 0f)),
                        mat.MultiplyPoint3x4(new Vector3(-scale, -scale, 0f)),
                        mat.MultiplyPoint3x4(new Vector3(scale, -scale, 0f))
                };
                Handles.zTest = UnityEngine.Rendering.CompareFunction.LessEqual;
                Handles.color = col;
                Handles.DrawAAConvexPolygon(vertices);
            }
        }
    }
}