using System.Collections;
using System.Collections.Generic;
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
        
        [Button("Split Edge")]
        public void SplitEdge() {
            if (halfEdgeMesh == null) {
                Debug.Log($"No mesh structure yet.");
                return;
            }

            if (splitEdge < 0) {
                Debug.Log($"No edge to split.");
                return;
            }
            
            if (newEdgePos == null) {
                Debug.Log($"No new edge position.");
                return;
            }

            halfEdgeMesh.SplitEdge(splitEdge, newEdgePos.position, newEdgePos.up);
        }
        
        [Button("Test")]
        public void Test() {
            halfEdgeMesh = new HalfEdgeMesh(meshFilter.sharedMesh);
        }

        private readonly List<(Vector3? a, Vector3? b)> bisectionResult = new();

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

            bisectionResult.Clear();
            halfEdgeMesh.Bisect(new Plane(plane.forward, plane.position), bisectionResult);
        }

        [Button("Combine All")]
        public void CombineNow() {
            if (halfEdgeMesh == null) {
                Debug.Log($"No mesh structure yet.");
                return;
            }

            halfEdgeMesh.CombineCoplanarFaces();
        }

        private void OnDrawGizmos() {
            if (halfEdgeMesh == null) {
                return;
            }

            // if (testFace >= 0) {
            //     var faces = halfEdgeMesh.Faces;
            //     if (testFace < faces.Count) {
            //         var face = faces[testFace];
            //         halfEdgeMesh.DrawFace(face, Color.red, true);
            //         
            //         foreach (var adjacentFace in halfEdgeMesh.GetAdjacentFaces(face)) {
            //             halfEdgeMesh.DrawFace(adjacentFace, Color.black, true);
            //         }
            //     }  
            // } else {
            halfEdgeMesh.DrawGizmos(labelHalfEdges, labelFaces);
            //}

            if (plane != null) {
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

            if (!bisectionResult.IsNullOrEmpty()) {
                foreach (var (a, b) in bisectionResult) {
                    if (a.HasValue && b.HasValue) {
                        Gizmos.color = Color.green;
                        Gizmos.DrawLine(a.Value, b.Value);
                        Gizmos.DrawSphere(a.Value, 0.02f);
                        Gizmos.DrawSphere(b.Value, 0.02f);
                    } else {
                        Gizmos.color = Color.red;
                        if (a.HasValue) {
                            Gizmos.DrawSphere(a.Value, 0.02f);
                        }

                        if (b.HasValue) {
                            Gizmos.DrawSphere(b.Value, 0.02f);
                        }
                    }
                }
            }
        }
    }
}