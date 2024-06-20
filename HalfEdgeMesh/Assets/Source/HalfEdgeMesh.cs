using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace HalfEdge {
    [Serializable]
    public class HalfEdgeMesh : ISerializationCallbackReceiver {
        [SerializeField]
        private Vector3[] vertices;

        [SerializeField]
        private Vector3[] normals;

        private Dictionary<int, HalfEdge> halfEdges;
        private HashSet<int> faces;
        
        [SerializeField]
        private List<SerializableHalfEdge> halfEdgesSerialized = new();
        
        [SerializeField]
        private List<SerializableFace> facesSerialized = new();

        public HalfEdgeMesh(Mesh mesh) {
            vertices = mesh.vertices;
            normals = mesh.normals;

            var triangles = mesh.triangles;
            halfEdges = new Dictionary<int, HalfEdge>(triangles.Length);
            faces = new HashSet<int>(mesh.triangles.Length / 3);

            InitializeHalfEdges(mesh);
        }

        // Create a half-edge for each edge in the mesh.
        // Each half-edge has a vertex index, a next half-edge index, and a twin half-edge index.
        private void InitializeHalfEdges(Mesh mesh) {
            // We won't be able to determine the twins until all half-edges are created, so store each vertex pair to look up later.
            var edgeToHalfEdge = new Dictionary<(int, int), int>();

            var triangles = mesh.triangles;

            for (var i = 0; i < triangles.Length; i += 3) {
                // Indices of the vertices of the triangle
                var v0 = triangles[i];
                var v1 = triangles[i + 1];
                var v2 = triangles[i + 2];

                // Create three half-edges
                var he0 = i;
                var he1 = i + 1;
                var he2 = i + 2;

                halfEdges[he0] = new HalfEdge {
                        Vertex = v0,
                        Next = he1,
                        Previous = he2,
                        Twin = -1,
                        Face = he0
                };
                edgeToHalfEdge[(v0, v1)] = he0;

                halfEdges[he1] = new HalfEdge {
                        Vertex = v1,
                        Next = he2,
                        Previous = he0,
                        Twin = -1,
                        Face = he0
                };
                edgeToHalfEdge[(v1, v2)] = he1;

                halfEdges[he2] = new HalfEdge {
                        Vertex = v2,
                        Next = he0,
                        Previous = he1,
                        Twin = -1,
                        Face = he0
                };

                edgeToHalfEdge[(v2, v0)] = he2;

                // Store the face index
                faces.Add(he0);
                UpdateSerialization();
            }

            // We don't know all the twins until after all half-edges are created.
            // Once they're created, we can go back over each half edge, look at the start and end vertices, flip them, and find the twin if it exists.
            for (var i = 0; i < halfEdges.Count; i++) {
                if (halfEdges[i].Twin != -1) {
                    continue;
                }

                var startVertex = halfEdges[i].Vertex;
                var endVertex = halfEdges[halfEdges[i].Next].Vertex;

                if (edgeToHalfEdge.TryGetValue((endVertex, startVertex), out var twinIndex)) {
                    SetTwins(i, twinIndex);
                }
            }

            void SetTwins(int indexA, int indexB) {
                var heA = halfEdges[indexA];
                var heB = halfEdges[indexB];

                halfEdges[indexA] = new HalfEdge {
                        Vertex = heA.Vertex,
                        Next = heA.Next,
                        Previous = heA.Previous,
                        Twin = indexB,
                        Face = heA.Face
                };
                halfEdges[indexB] = new HalfEdge {
                        Vertex = heB.Vertex,
                        Next = heB.Next,
                        Previous = heB.Previous,
                        Twin = indexA,
                        Face = heB.Face
                };
            }
        }

        public IEnumerable CombineCoplanarFacesEnumerable() {
            //Debug.Log($"Starting. There are now {faces.Count} faces.");
            
            // Iterate over all half-edges. If we find and remove a pair of coplanar faces, we start again, and keep going until we find no more.
            bool startOver;
            var count = 0;
            do {
                count++;
                startOver = false;
                foreach (var (id, he) in halfEdges) {
                    if (he.Twin != -1) {
                        //Debug.Log($"Checking the half edge {id}->{he.Next}, twin {he.Twin}->{halfEdges[he.Twin].Next}");
                        if (IsCoplanar(he.Face, halfEdges[he.Twin].Face)) {
                            //Debug.Log($"Collapsing edge {id}");
                            CollapseEdge(id);
                            startOver = true;
                            UpdateSerialization();
                            yield return null;
                            break;
                        }
                    }
                }
            } while (startOver && count < 1000);

            if (count >= 1000) {
                Debug.LogError("Too many iterations. Aborting.");
            }
        }
        
        public void CombineCoplanarFaces() {
            // Iterate over all half-edges. If we find and remove a pair of coplanar faces, we start again, and keep going until we find no more.
            bool startOver;
            var count = 0;
            do {
                count++;
                startOver = false;
                foreach (var (id, he) in halfEdges) {
                    if (he.Twin != -1) {
                        if (IsCoplanar(he.Face, halfEdges[he.Twin].Face)) {
                            CollapseEdge(id);
                            startOver = true;
                            break;
                        }
                    }
                }
            } while (startOver && count < 1000);

            if (count >= 1000) {
                Debug.LogError("Too many iterations. Aborting.");
            }
        }

        private bool IsCoplanar(int faceIndexA, int faceIndexB) {
            if (faceIndexA == faceIndexB) {
                return true;
            }
            
            var normal1 = GetFaceNormal(faceIndexA);
            var normal2 = GetFaceNormal(faceIndexB);
            
            return Vector3.Dot(normal1, normal2) > 0.999f;
        }
        
        private bool IsCoplanar(int faceIndex, Vector3 normal) {
            var faceNormal = GetFaceNormal(faceIndex);
            return Vector3.Dot(faceNormal, normal) > 0.999f;
        }

        private void CollapseEdge(int id) {
            var heA = halfEdges[id];
            var heB = halfEdges[heA.Twin];
            
            var oldFaceA = heA.Face;
            var oldFaceB = heB.Face;

            // Get the normal of one of the two faces. It's assumed that the faces are coplanar.
            var oldNormal = GetFaceNormal(oldFaceA);

            SetNext(heA.Previous, heB.Next);
            SetNext(heB.Previous, heA.Next);

            // Find starting index which isn't the same as one of the removed half-edges.
            // The new face index should also produce the same normal as the old face.
            var currentIndex = heA.Next;
            while (currentIndex == id || currentIndex == heA.Twin || !IsCoplanar(currentIndex, oldNormal)) {
                currentIndex = halfEdges[currentIndex].Next;
            }
            
            halfEdges.Remove(id);
            halfEdges.Remove(heA.Twin);
            
            // Update the face indices of all half-edges in the face.
            var faceIndex = currentIndex;
            do {
                var old = halfEdges[currentIndex];
                
                halfEdges[currentIndex] = new HalfEdge {
                        Vertex = old.Vertex,
                        Next = old.Next,
                        Previous = old.Previous,
                        Twin = old.Twin,
                        Face = faceIndex
                };
                
                currentIndex = old.Next;
            } while (currentIndex != faceIndex);

            // Remove the old face indices
            faces.Remove(oldFaceA);
            faces.Remove(oldFaceB);

            // Add the new face index
            faces.Add(faceIndex);
            
            return;

            void SetNext(int index, int next) {
                var he = halfEdges[index];
                halfEdges[index] = new HalfEdge {
                        Vertex = he.Vertex,
                        Next = next,
                        Previous = he.Previous,
                        Twin = he.Twin,
                        Face = he.Face
                };

                he = halfEdges[next];
                halfEdges[next] = new HalfEdge {
                        Vertex = he.Vertex,
                        Next = he.Next,
                        Previous = index,
                        Twin = he.Twin,
                        Face = he.Face
                };
            }
        }

        /// Get the normal of the face with the given index.
        public Vector3 GetFaceNormal(int faceIndex) {
            var he0 = halfEdges[faceIndex];
            var he1 = halfEdges[he0.Next];
            var he2 = halfEdges[he1.Next];
            
            var v0 = vertices[he0.Vertex];
            var v1 = vertices[he1.Vertex];
            var v2 = vertices[he2.Vertex];
            
            // If he2 is colinear with he0 and he1, we need to find a new he2
            while (Vector3.Cross(v1 - v0, v2 - v0).sqrMagnitude < 0.0001f) {
                he2 = halfEdges[he2.Next];
                v2 = vertices[he2.Vertex];
            }
            
            return Vector3.Cross(v1 - v0, v2 - v0).normalized;
        }

        /// Enumerate the face indices of all faces adjacent to the face with the given index.
        public IEnumerable<int> GetAdjacentFaces(int faceIndex) {
            var currentIndex = faceIndex;
            do {
                var he = halfEdges[currentIndex];

                if (he.Twin != -1) {
                    yield return halfEdges[he.Twin].Face;
                }

                currentIndex = he.Next;
            } while (currentIndex != faceIndex);
        }

        public void DrawGizmos(bool labelHalfEdges = false, bool labelFaces = false) {
#if UNITY_EDITOR
            if (halfEdges == null) {
                return;
            }

            if (faces == null) {
                return;
            }

            foreach (var f in faces) {
                DrawFace(f, Color.black, labelHalfEdges, labelFaces);
            }
#endif
        }

#if UNITY_EDITOR
        [SerializeField]
        [HideInInspector]
        private List<int> halfEdgesInFaceToDraw = new();

        public void DrawFace(int faceIndex, Color color, bool labelHalfEdges = false, bool labelFaces = false) {
            Handles.zTest = UnityEngine.Rendering.CompareFunction.LessEqual;
            Handles.color = color;

            halfEdgesInFaceToDraw.Clear();

            var centroid = Vector3.zero;

            var count = 0;
            try {
                var currentIndex = faceIndex;
                do {
                    var he = halfEdges[currentIndex];
                    halfEdgesInFaceToDraw.Add(currentIndex);
                    centroid += vertices[he.Vertex];
                    currentIndex = he.Next;
                } while (currentIndex != faceIndex && count++ < 100);
            } catch (KeyNotFoundException e) {
                Debug.LogError($"Face {faceIndex} does not exist. Aborting drawing.");
                
                foreach (var (key, value) in halfEdges) {
                    Debug.Log($"Half edge {key}: {value.Vertex} {value.Previous} {value.Next} {value.Twin} {value.Face}");
                }
                
                return;
            }

            
            if (count >= 100) {
                Debug.LogError($"Face {faceIndex} has more than 100 vertices, or does not loop. Aborting drawing.");
                return;
            }

            Debug.Assert(halfEdgesInFaceToDraw.Count >= 3, "Face must have at least 3 vertices");

            centroid /= halfEdgesInFaceToDraw.Count;

            foreach (var i in halfEdgesInFaceToDraw) {
                var he = halfEdges[i];

                var from = vertices[he.Vertex];
                var to = vertices[halfEdges[he.Next].Vertex];

                int? fromLabel = labelHalfEdges ? i : null;
                int? toLabel = labelHalfEdges ? he.Next : null;

                DrawHalfArrow(from, to, centroid, normals[he.Vertex], fromLabel, toLabel);
            }
            
            if (labelFaces) {
                Utils.Label(centroid, faceIndex, col:Color.black);
                
                Handles.color = color;
                Handles.DrawAAPolyLine(6f, centroid, centroid + GetFaceNormal(faceIndex));
            }
        }
#endif

#if UNITY_EDITOR
        private void DrawHalfArrow(Vector3 from, Vector3 to, Vector3 centroid, Vector3 normal, int? fromLabel = null, int? toLabel = null) {
            const float arrowScale = 0.06f;
            const float offset = 0.05f;

            // Move both from and to a tiny bit towards the centroid
            from = Vector3.MoveTowards(from, centroid, offset);
            to = Vector3.MoveTowards(to, centroid, offset);

            var perpendicular = GetPerpendicularVector(to, from, centroid);
            var basePoint = to - arrowScale * (to - from).normalized;
            
            Handles.DrawAAPolyLine(6f, from, to);
            Handles.DrawAAConvexPolygon(to, basePoint, basePoint + arrowScale * perpendicular * 0.5f);

            if (fromLabel.HasValue) {
                Utils.Label(from, fromLabel.Value, col:Color.black);
            }

            if (toLabel.HasValue) {
                Utils.Label(to, toLabel.Value, col:Color.black);
            }
        }
#endif

        private static Vector3 GetPerpendicularVector(Vector3 to, Vector3 from, Vector3 centroid) {
            // Calculate the displacement vector
            var displacement = to - from;

            // Find the vector from 'from' to the centroid
            var vectorToCentroid = centroid - from;

            // Calculate the normal vector of the plane defined by the triangle
            var normal = Vector3.Cross(displacement, vectorToCentroid).normalized;

            // Calculate two perpendicular vectors in the plane
            var perp1 = Vector3.Cross(displacement, normal);
            var perp2 = Vector3.Cross(displacement, -normal);

            // Determine which perpendicular vector points towards the centroid
            var dot1 = Vector3.Dot(vectorToCentroid, perp1);

            // Select the vector with the positive dot product
            return dot1 > 0 ? perp1 : perp2;
        }

        private void UpdateSerialization() {
            halfEdgesSerialized.Clear();
            facesSerialized.Clear();
            foreach (var face in faces) {
                facesSerialized.Add(new SerializableFace(face));
                
                // Serialize the half edges by face
                var currentIndex = face;
                do {
                    var he = halfEdges[currentIndex];
                    halfEdgesSerialized.Add(new SerializableHalfEdge(currentIndex, he));
                    currentIndex = he.Next;
                } while (currentIndex != face);
            }
        }
        
        public void OnBeforeSerialize() {
            UpdateSerialization();
        }

        public void OnAfterDeserialize() {
            halfEdges = new Dictionary<int, HalfEdge>(halfEdgesSerialized.Count);
            foreach (var pair in halfEdgesSerialized) {
                halfEdges[pair.Key] = pair.Value;
            }
            
            faces = new HashSet<int>(facesSerialized.Count);
            foreach (var face in facesSerialized) {
                faces.Add(face.Index);
            }
        }
    }
    
    [Serializable]
    public struct SerializableHalfEdge {
        public string Name;
        public int Key;
        public HalfEdge Value;
        
        public SerializableHalfEdge(int key, HalfEdge value) {
            Name = $"{key}->{value.Next}";
            Key = key;
            Value = value;
        }
    }

    [Serializable]
    public struct SerializableFace {
        public string Name;
        public int Index;
        
        public SerializableFace(int index) {
            Name = index.ToString();
            Index = index;
        }
    }

    [Serializable]
    public struct HalfEdge {
        public int Vertex; // The index of the vertex/normal/etc
        public int Previous; // The index of the previous half edge in the halfEdges array
        public int Next; // The index of the next half edge in the halfEdges array
        public int Twin; // The index of the twin half edge in the halfEdges array
        public int Face; // The index of the face this half edge belongs to (i.e., the index of the first half edge in the face)
    }
}