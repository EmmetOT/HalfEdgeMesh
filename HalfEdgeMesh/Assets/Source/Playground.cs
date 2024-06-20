using System.Collections;
using NaughtyAttributes;
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
        
        [Button("Test")]
        public void Test() {
            combineCoplanarFacesEnumerator = null;
            count = 0;
            halfEdgeMesh = new HalfEdgeMesh(meshFilter.sharedMesh);
        }

        private IEnumerator combineCoplanarFacesEnumerator;
        private int count = 0;
        
        [Button("Combine All")]
        public void CombineNow() {
            if (halfEdgeMesh == null) {
                Debug.Log($"No mesh structure yet.");
                return;
            }
            
            combineCoplanarFacesEnumerator = null;
            count = 0;
            
            halfEdgeMesh.CombineCoplanarFaces();
        }
        
        [Button("Combine Step")]
        public void TestCombine() {
            if (halfEdgeMesh == null) {
                Debug.Log($"No mesh structure yet.");
                return;
            }
            
            if (combineCoplanarFacesEnumerator == null) {
                combineCoplanarFacesEnumerator = halfEdgeMesh.CombineCoplanarFacesEnumerable().GetEnumerator();
                combineCoplanarFacesEnumerator.MoveNext();
                ++count;
            } else {
                try {
                    if (combineCoplanarFacesEnumerator.MoveNext()) {
                        Debug.Log($"Iteration {count}");
                        ++count;
                    } else {
                        Debug.Log($"Finished in {count} iterations.");
                        combineCoplanarFacesEnumerator = null;
                        count = 0;
                    }
                } catch (System.Exception e) {
                    Debug.LogError(e);
                    combineCoplanarFacesEnumerator = null;
                    count = 0;
                }
            }
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
        }
    }
}