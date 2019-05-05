using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text;
using System.Linq;

[RequireComponent(typeof(MeshFilter))]
public class OctreeRayTracer : MonoBehaviour
{
	public Transform Camera;
	public int MinTrisPerOctant = 1;
	public bool Deepen;
	public float fov;
	public int width;
	public int height;
	private Octree o;
	private Mesh mesh;
	private static int[] tris;
	private static Vector3[] verts;
	private static Vector3[] normals;
	private static Transform thisTransform;
	private bool canDeepen = true;

	private UnityEngine.Random.State randState;

	void Start()
    {
		thisTransform = transform;
		mesh = GetComponent<MeshFilter>().mesh;
		tris = mesh.triangles;
		verts = mesh.vertices;
		normals = mesh.normals;
		Vector3 corner1 = verts[0];
		Vector3 corner2 = verts[0];
		foreach (Vector3 vert in verts) {
			corner1 = Vector3.Min(corner1, vert);
			corner2 = Vector3.Max(corner2, vert);
		}
		o = new Octree(new Box(corner1, corner2));
		for (int i=0; i<tris.Length/3; i++) { 
			o.AddTriangle(i);
		}
		randState = UnityEngine.Random.state;
    }

	class Ray {
		public Ray(Vector3 s, Vector3 d) {
			Start = s;
			Dir = d.normalized;
			Inv_dir = new Vector3(1 / d.x, 1 / d.y, 1 / d.z);
			sign = new int[3]{
				Inv_dir.x < 0 ? 1 : 0,
				Inv_dir.y < 0 ? 1 : 0,
				Inv_dir.z < 0 ? 1 : 0
			};
		}

		public int[] sign { get; }
		public Vector3 Start { get; }
		public Vector3 Dir { get; }
		public Vector3 Inv_dir { get; }

		public bool IntersectTriangle(int triIndex, out Hit hit) {
			/* https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm */

			const float EPSILON = 0.0000001f;
			Vector3 A = verts[tris[3 * triIndex + 0]];
			Vector3 B = verts[tris[3 * triIndex + 1]];
			Vector3 C = verts[tris[3 * triIndex + 2]];

			Vector3 edge1 = B - A;
			Vector3 edge2 = C - A;
			Vector3 h = Vector3.Cross(Dir, edge2);
			float a = Vector3.Dot(edge1, h);
			if (-EPSILON < a && a < EPSILON) {
				hit = null;
				return false;
			}
			float f = 1f / a;
			Vector3 s = Start - A;
			Vector2 uv;
			uv.x = f * Vector3.Dot(s, h);
			if (uv.x < 0 || uv.x > 1) {
				hit = null;
				return false;
			}
			Vector3 q = Vector3.Cross(s, edge1);
			uv.y = f * Vector3.Dot(Dir, q);
			if (uv.y < 0 || uv.x + uv.y > 1) {
				hit = null;
				return false;
			}
			float d = f * Vector3.Dot(edge2, q);
			if (d > EPSILON) {
				hit = new Hit(this, d, this.Start + this.Dir * d, triIndex, uv);
				return true;
			} else {
				hit = null;
				return false;
			}
		}
	}

	class Hit {
		public Hit (Ray ray, float dist, Vector3 pt, int triIndex, Vector2 uv) {
			this.ray = ray;
			this.dist = dist;
			this.pt = pt;
			this.triIndex = triIndex;
			this.uv = uv;
		}
		public Ray ray { get; }
		public float dist { get; }
		public Vector3 pt { get; }
		public int triIndex { get; }
		public Vector2 uv { get; }
	}

	class Box {
		public Box(Vector3 corner1, Vector3 corner2) {
			Bounds = new Vector3[2] {
				Vector3.Min(corner1, corner2),
				Vector3.Max(corner1, corner2)
			};
			Extents = Bounds[1] - Bounds[0];
			Center = Bounds[0] + Extents / 2;
		}

		public Vector3[] Bounds { get; }
		public Vector3 Extents { get; }
		public Vector3 Center { get; }

		public bool Intersect(Ray r, ref Vector2 d, ref int closeSide, ref int farSide) {
			/* http://www.cs.utah.edu/~awilliam/box/box.pdf */
			d.x = (Bounds[r.sign[0]].x - r.Start.x) *r.Inv_dir.x;
			d.y = (Bounds[1 - r.sign[0]].x - r.Start.x) *r.Inv_dir.x;
			closeSide = 2 + r.sign[0];
			farSide = 3 - r.sign[0];
			float tymin = (Bounds[r.sign[1]].y - r.Start.y) * r.Inv_dir.y;
			float tymax = (Bounds[1 - r.sign[1]].y - r.Start.y) * r.Inv_dir.y;
			if ((d[0] > tymax) || (tymin > d[1])) {
				return false;
			}
			if (tymin > d[0]) {
				d[0] = tymin;
				closeSide = 4 + r.sign[1];
			}
			if (tymax < d[1]) {
				d[1] = tymax;
				farSide = 5 - r.sign[1];
			}
			float tzmin = (Bounds[r.sign[2]].z - r.Start.z) * r.Inv_dir.z;
			float tzmax = (Bounds[1 - r.sign[2]].z - r.Start.z) * r.Inv_dir.z;
			if ((d[0] > tzmax) || (tzmin > d[1])) {
				return false;
			}
			if (tzmin > d[0]) {
				d[0] = tzmin;
				closeSide = r.sign[2];
			}
			if (tzmax < d[1]) {
				d[1] = tzmax;
				farSide = 1 - r.sign[2];
			}

			return d[1] > 0;
		}

		public void Draw(Color color) {
			Vector3 c000 = thisTransform.TransformPoint(Bounds[0]);
			Vector3 c111 = thisTransform.TransformPoint(Bounds[1]);

			Vector3 c100 = thisTransform.TransformPoint(Bounds[0] + Extents.x * Vector3.right);
			Vector3 c010 = thisTransform.TransformPoint(Bounds[0] + Extents.y * Vector3.up);
			Vector3 c001 = thisTransform.TransformPoint(Bounds[0] + Extents.z * Vector3.forward);

			Vector3 c110 = thisTransform.TransformPoint(Bounds[0] + Extents.x * Vector3.right + Extents.y * Vector3.up);
			Vector3 c101 = thisTransform.TransformPoint(Bounds[0] + Extents.x * Vector3.right + Extents.z * Vector3.forward);
			Vector3 c011 = thisTransform.TransformPoint(Bounds[0] + Extents.y * Vector3.up + Extents.z * Vector3.forward);

			Debug.DrawLine(c000, c001, color);
			Debug.DrawLine(c000, c010, color);
			Debug.DrawLine(c000, c100, color);
			Debug.DrawLine(c001, c101, color);
			Debug.DrawLine(c001, c011, color);
			Debug.DrawLine(c010, c011, color);
			Debug.DrawLine(c010, c110, color);
			Debug.DrawLine(c100, c110, color);
			Debug.DrawLine(c100, c101, color);
			Debug.DrawLine(c110, c111, color);
			Debug.DrawLine(c101, c111, color);
			Debug.DrawLine(c011, c111, color);
		}
	}
	class Octree {
		/*
		Octants indexed by numbered corners
		Neighbors indexed by numbered arrows
		
		1         5           
		  🡖       ↓           
		   3─────────7        
		   │╲        │╲       
		   │ ╲       │ ╲      
		   │  ╲      │  ╲     
		   1───╲─────5   ╲ ←3 
		 2→ ╲   2─────────6   
		     ╲  │      ╲  │   
		      ╲ │       ╲ │   
		       ╲│        ╲│   
		        0─────────4   
		          ↑        🡔  
		          4          0
		*/
		public Octree(Box box) {
			this.box = box;
			children = null;
			depth = 0;
			triIndices = new List<int>();
			id = 0;
			neighbors = new Octree[6] { null, null, null, null, null, null };
		}
		private Octree(Box box, int depth, int id) {
			this.box = box;
			children = null;
			this.depth = depth;
			triIndices = new List<int>();
			this.id = id;
			neighbors = new Octree[6];
		}
		public Box box { get; }
		Octree[] children;
		private Octree[] neighbors { get; set; }
		private List<int> triIndices;
		public int depth { get; }
		readonly int id;

		private int GetChildIndex(int side, ref Vector3 uv) {
			switch (side) {
				case 0:
					if (uv.x < 0.5f) {
						uv.x *= 2;
						if (uv.y < 0.5f) {
							uv.y *= 2;
							return 0;
						} else {
							uv.y = 2 * uv.y - 1;
							return 2;
						}
					} else {
						uv.x = 2 * uv.x - 1;
						if (uv.y < 0.5f) {
							uv.y *= 2;
							return 4;
						} else {
							uv.y = 2 * uv.y - 1;
							return 6;
						}
					}
				case 1:
					if (uv.x < 0.5f) {
						uv.x *= 2;
						if (uv.y < 0.5f) {
							uv.y *= 2;
							return 1;
						} else {
							uv.y = 2 * uv.y - 1;
							return 3;
						}
					} else {
						uv.x = 2 * uv.x - 1;
						if (uv.y < 0.5f) {
							uv.y *= 2;
							return 5;
						} else {
							uv.y = 2 * uv.y - 1;
							return 7;
						}
					}
				case 2:
					if (uv.y < 0.5f) {
						uv.y *= 2;
						if (uv.z < 0.5f) {
							uv.z *= 2;
							return 0;
						} else {
							uv.z = 2 * uv.z - 1;
							return 1;
						}
					} else {
						uv.y = 2 * uv.y - 1;
						if (uv.z < 0.5f) {
							uv.z *= 2;
							return 2;
						} else {
							uv.z = 2 * uv.z - 1;
							return 3;
						}
					}
				case 3:
					if (uv.y < 0.5f) {
						uv.y *= 2;
						if (uv.z < 0.5f) {
							uv.z *= 2;
							return 4;
						} else {
							uv.z = 2 * uv.z - 1;
							return 5;
						}
					} else {
						uv.y = 2 * uv.y - 1;
						if (uv.z < 0.5f) {
							uv.z *= 2;
							return 6;
						} else {
							uv.z = 2 * uv.z - 1;
							return 7;
						}
					}
				case 4:
					if (uv.x < 0.5f) {
						uv.x *= 2;
						if (uv.z < 0.5f) {
							uv.z *= 2;
							return 0;
						} else {
							uv.z = 2 * uv.z - 1;
							return 1;
						}
					} else {
						uv.x = 2 * uv.x - 1;
						if (uv.z < 0.5f) {
							uv.z *= 2;
							return 4;
						} else {
							uv.z = 2 * uv.z - 1;
							return 5;
						}
					}
				case 5:
					if (uv.x < 0.5f) {
						uv.x *= 2;
						if (uv.z < 0.5f) {
							uv.z *= 2;
							return 2;
						} else {
							uv.z = 2 * uv.z - 1;
							return 3;
						}
					} else {
						uv.x = 2 * uv.x - 1;
						if (uv.z < 0.5f) {
							uv.z *= 2;
							return 6;
						} else {
							uv.z = 2 * uv.z - 1;
							return 7;
						}
					}
				default:
					throw new IndexOutOfRangeException("Invalid edge index");
			}
		}

		private bool FindRayIntersections(Vector3 closeUV, Vector3 farUV, int closeSide, Ray r, ref HashSet<int> checkedTris, out Hit hit) {
			hit = null;
			if (children != null) {
				float[] divs = {
					(1 - 2 * closeUV.x) / (2 * farUV.x - 2 * closeUV.x),
					(1 - 2 * closeUV.y) / (2 * farUV.y - 2 * closeUV.y),
					(1 - 2 * closeUV.z) / (2 * farUV.z - 2 * closeUV.z)
				};
				bool xValid = 0 <= divs[0] && divs[0] <= 1,
					 yValid = 0 <= divs[1] && divs[1] <= 1,
					 zValid = 0 <= divs[2] && divs[2] <= 1;
				int[] divAxes = { -1, -1, -1 };
				int divCount = 0;
				if (xValid) {
					divCount++;
					if (yValid) {
						divCount++;
						bool xy = divs[0] < divs[1];
						if (zValid) { // x && y && z
							divCount++;
							bool xz = divs[0] < divs[2];
							bool yz = divs[1] < divs[2];
							if (xy) { // x < y
								if (xz) { // x < y && x < z
									divAxes[0] = 0;
									if (yz) { // x < y < z
										divAxes[1] = 1;
										divAxes[2] = 2;
									} else { // x < z < y
										divAxes[1] = 2;
										divAxes[2] = 1;
									}
								} else { // z < x < y
									divAxes[0] = 2;
									divAxes[1] = 0;
									divAxes[2] = 1;
								}
							} else { // y < x
								if (xz) { // y < x < z
									divAxes[0] = 1;
									divAxes[1] = 0;
									divAxes[2] = 2;
								} else { // y < x && z < x
									divAxes[2] = 0;
									if (yz) { // y < z < x
										divAxes[0] = 1;
										divAxes[1] = 2;
									} else { // z < y < x
										divAxes[0] = 2;
										divAxes[1] = 1;
									}
								}
							}
						} else { // x && y && !z
							if (xy) { // x < y
								divAxes[0] = 0;
								divAxes[1] = 1;
							} else { // y < x
								divAxes[0] = 1;
								divAxes[1] = 0;
							}
						}
					} else {
						if (zValid) { // x && !y && z
							divCount++;
							bool xz = divs[0] < divs[2];
							if (xz) { // x < z
								divAxes[0] = 0;
								divAxes[1] = 2;
							} else { // z < x
								divAxes[0] = 2;
								divAxes[1] = 0;
							}
						} else { // x && !y && !z
							divAxes[0] = 0;
						}
					}
				} else { 
					if (yValid) {
						divCount++;
						if (zValid) { // !x && y && z
							divCount++;
							bool yz = divs[1] < divs[2];
							if (yz) { // y < z
								divAxes[0] = 1;
								divAxes[1] = 2;
							} else { // z < y
								divAxes[0] = 2;
								divAxes[1] = 1;
							}
						} else { // !x && y && !z
							divAxes[0] = 1;
						}
					} else {
						if (zValid) { // !x && !y && z
							divCount++;
							divAxes[0] = 2;
						}
					}
				}
				Vector3 currCloseUV = closeUV;
				int childIndex = GetChildIndex(closeSide, ref currCloseUV);
				Octree curr = children[childIndex];
				int currSide = closeSide;
				for(int i=0; i<divCount; i++) {
					float div = divs[divAxes[i]];

					Vector3 currFarUV = closeUV + div * (farUV - closeUV);
					currFarUV.x = currFarUV.x < 0.5f ? 2 * currFarUV.x : 2 * currFarUV.x - 1;
					currFarUV.y = currFarUV.y < 0.5f ? 2 * currFarUV.y : 2 * currFarUV.y - 1;
					currFarUV.z = currFarUV.z < 0.5f ? 2 * currFarUV.z : 2 * currFarUV.z - 1;
					int nextSide;
					Octree next;
					Vector3 nextCloseUV;
					switch (divAxes[i]) { 
						case 0: // x-axis
							switch (childIndex) {
								case 0:
								case 1:
								case 2:
								case 3:
									currFarUV.x = 1;
									nextCloseUV = currFarUV;
									nextCloseUV.x = 0;
									nextSide = 2;
									next = curr.neighbors[3];
									break;
								default:
									currFarUV.x = 0;
									nextCloseUV = currFarUV;
									nextCloseUV.x = 1;
									nextSide = 3;
									next = curr.neighbors[2];
									break;
							}
							break;
						case 1: // y-axis
							switch (childIndex) {
								case 0:
								case 1:
								case 4:
								case 5:
									currFarUV.y = 1;
									nextCloseUV = currFarUV;
									nextCloseUV.y = 0;
									nextSide = 4;
									next = curr.neighbors[5];
									break;
								default:
									currFarUV.y = 0;
									nextCloseUV = currFarUV;
									nextCloseUV.y = 1;
									nextSide = 5;
									next = curr.neighbors[4];
									break;
							}
							break;
						default: // z-axis
							switch (childIndex) {
								case 0:
								case 2:
								case 4:
								case 6:
									currFarUV.z = 1;
									nextCloseUV = currFarUV;
									nextCloseUV.z = 0;
									nextSide = 0;
									next = curr.neighbors[1];
									break;
								default:
									currFarUV.z = 0;
									nextCloseUV = currFarUV;
									nextCloseUV.z = 1;
									nextSide = 1;
									next = curr.neighbors[0];
									break;
							}
							break;
					}
					curr.FindRayIntersections(currCloseUV, currFarUV, currSide, r, ref checkedTris, out Hit newHit);
					if (newHit != null) {
						if (hit == null || newHit.dist < hit.dist) {
							hit = newHit;
						}
						Vector3 nextClosePt = curr.box.Bounds[0] + Vector3.Scale(currCloseUV, curr.box.Extents);
						if ((nextClosePt - r.Start).magnitude > hit.dist) return true; //Next octant is further than nearest found triangle
					}
					currSide = nextSide;
					curr = next;
					currCloseUV = nextCloseUV;

				}
				farUV.x = farUV.x < 0.5f ? 2 * farUV.x : 2 * farUV.x - 1;
				farUV.y = farUV.y < 0.5f ? 2 * farUV.y : 2 * farUV.y - 1;
				farUV.z = farUV.z < 0.5f ? 2 * farUV.z : 2 * farUV.z - 1;
				if (curr.FindRayIntersections(currCloseUV, farUV, currSide, r, ref checkedTris, out Hit newHit2)) {
					if (hit == null || newHit2.dist < hit.dist) {
						hit = newHit2;
					}
				}
			} else if (triIndices.Count > 0){
				Vector3 start = box.Bounds[0] + Vector3.Scale(closeUV, box.Extents);
				Vector3 end = box.Bounds[0] + Vector3.Scale(farUV, box.Extents);
				foreach (int tri in triIndices) {
					if (!checkedTris.Contains(tri)) {
						if (r.IntersectTriangle(tri, out Hit newHit)) {
							if (hit == null || newHit.dist < hit.dist) {
								hit = newHit;
							}
						}
						checkedTris.Add(tri);
					}
				}
			}
			return hit != null;
		}

		public bool RayTrace(Ray r, out Hit hit) {
            Vector2 d = new Vector2();
            int closeSide = 0, farSide = 0;
            hit = null;
            if (!box.Intersect(r, ref d, ref closeSide, ref farSide))
            {
                
                return false;
            }
            Debug.DrawRay(thisTransform.TransformPoint(r.Start), thisTransform.TransformDirection(r.Dir) * 100);
            Vector3 uv = r.Start + d[0] * r.Dir - box.Bounds[0];
            uv.x /= box.Extents.x;
            uv.y /= box.Extents.y;
            uv.z /= box.Extents.z;
            Debug.Log(uv);
            Octree curr = this;
            
            while (curr != null)
            {
                while (curr.children != null)
                {
                    int childIndex = GetChildIndex(closeSide, ref uv);
                    curr = curr.children[childIndex];
                    Debug.Log(uv);
                }
                curr.Draw(Color.white);
                Vector3 scaledDir = new Vector3(
                    r.Dir.x / curr.box.Extents.x,
                    r.Dir.y / curr.box.Extents.y,
                    r.Dir.z / curr.box.Extents.z
                );
                Vector3 inv_dir = new Vector3(
                    1f / scaledDir.x,
                    1f / scaledDir.y,
                    1f / scaledDir.z
                );
                int[] sign = new int[3]{
                    inv_dir.x < 0 ? 1 : 0,
                    inv_dir.y < 0 ? 1 : 0,
                    inv_dir.z < 0 ? 1 : 0
                };
                float dx = (1 - sign[0] - uv.x) * inv_dir.x;
                float dy = (1 - sign[1] - uv.y) * inv_dir.y;
                float dz = (1 - sign[2] - uv.z) * inv_dir.z;
                if (dx < dy)
                {
                    if (dx < dz)
                    { /* dx < dz && dx < dy */
                        uv += scaledDir * dx;
                        farSide = 3 - sign[0];
                    }
                    else
                    { /* dz <= dx < dy */
                        uv += scaledDir * dz;
                        farSide = 1 - sign[2];
                    }
                }
                else
                { /* dy <= dx */
                    if (dy < dz)
                    {
                        uv += scaledDir * dy;
                        farSide = 5 - sign[1];
                    }
                    else
                    { /* dz <= dy <= dx */
                        uv += scaledDir * dz;
                        farSide = 1 - sign[2];
                    }
                }
                closeSide = farSide - 2 * (farSide % 2) + 1;
                uv = curr.box.Bounds[0] + Vector3.Scale(uv, curr.box.Extents);
                curr = curr.neighbors[farSide];
                if (curr != null)
                {
                    uv.x = (uv.x - curr.box.Bounds[0].x) / curr.box.Extents.x;
                    uv.y = (uv.y - curr.box.Bounds[0].y) / curr.box.Extents.y;
                    uv.y = (uv.z - curr.box.Bounds[0].z) / curr.box.Extents.z;
                    Debug.Log("\t" + uv);
                }
            }
            return false;
            /*
            Vector2 d = new Vector2();
            int closeSide = 0, farSide = 0;
            if (box.Intersect(r, ref d, ref closeSide, ref farSide)) {
                Vector3 u1 = r.Start + d[0] * r.Dir - box.Bounds[0];
                u1.x /= box.Extents.x;
                u1.y /= box.Extents.y;
                u1.z /= box.Extents.z;
                Vector3 u2 = r.Start + d[1] * r.Dir - box.Bounds[0];
                u2.x /= box.Extents.x;
                u2.y /= box.Extents.y;
                u2.z /= box.Extents.z;
                var checkedTris = new HashSet<int>();
                return FindRayIntersections(u1, u2, closeSide, r, ref checkedTris, out hit);
            } else {
                hit = null;
                return false;
            }
            */
         }

		public bool Deepen(int minTrisPerOctant) {
			if (children == null) {
				if (triIndices.Count > minTrisPerOctant) {
					Subdivide();
					foreach (int tri in triIndices) {
						for (int i = 0; i < 8; i++) {
							children[i].AddTriangle(tri);
						}
					}
					return true;
				}
				return false;
			} else {
				bool deepened = false;
				for (int i = 0; i < 8; i++) {
					deepened = children[i].Deepen(minTrisPerOctant) || deepened;
				}
				return deepened;
			}
		}

		public void AddTriangle(int index) {
			if (triIndices.Contains(index)) return;
			Vector3 A = verts[tris[3 * index + 0]];
			Vector3 B = verts[tris[3 * index + 1]];
			Vector3 C = verts[tris[3 * index + 2]];
			Vector3 offsetA = A - box.Center;
			Vector3 offsetB = B - box.Center;
			Vector3 offsetC = C - box.Center;

			Vector3 extents = box.Extents / 2;

			Vector3 ba = offsetB - offsetA;
			Vector3 cb = offsetC - offsetB;

			float x_ba_abs = Mathf.Abs(ba.x);
			float y_ba_abs = Mathf.Abs(ba.y);
			float z_ba_abs = Mathf.Abs(ba.z);

			{
				float min = ba.z * offsetA.y - ba.y * offsetA.z;
				float max = ba.z * offsetC.y - ba.y * offsetC.z;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = z_ba_abs * extents.y + y_ba_abs * extents.z;
				if (min > rad || max < -rad) return;
			}
			{
				float min = -ba.z * offsetA.x + ba.x * offsetA.z;
				float max = -ba.z * offsetC.x + ba.x * offsetC.z;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = z_ba_abs * extents.x + x_ba_abs * extents.z;
				if (min > rad || max < -rad) return;
			}
			{
				float min = ba.y * offsetB.x - ba.x * offsetB.y;
				float max = ba.y * offsetC.x - ba.x * offsetC.y;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = y_ba_abs * extents.x + x_ba_abs * extents.y;
				if (min > rad || max < -rad) return;
			}
			float x_cb_abs = Mathf.Abs(cb.x);
			float y_cb_abs = Mathf.Abs(cb.y);
			float z_cb_abs = Mathf.Abs(cb.z);
			{
				float min = cb.z * offsetA.y - cb.y * offsetA.z,
					  max = cb.z * offsetC.y - cb.y * offsetC.z;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = z_cb_abs * extents.y + y_cb_abs * extents.z;
				if (min > rad || max < -rad) return;
			}
			{
				float min = -cb.z * offsetA.x + cb.x * offsetA.z,
					  max = -cb.z * offsetC.x + cb.x * offsetC.z;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = z_cb_abs * extents.x + x_cb_abs * extents.z;
				if (min > rad || max < -rad) return;
			}
			{
				float min = cb.y * offsetA.x - cb.x * offsetA.y,
					  max = cb.y * offsetB.x - cb.x * offsetB.y;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = y_cb_abs * extents.x + x_cb_abs * extents.y;
				if (min > rad || max < -rad) return;
			}
			Vector3 ac = offsetA - offsetC;
			float x_ac_abs = Mathf.Abs(ac.x);
			float y_ac_abs = Mathf.Abs(ac.y);
			float z_ac_abs = Mathf.Abs(ac.z);
			{
				float min = ac.z * offsetA.y - ac.y * offsetA.z,
					  max = ac.z * offsetB.y - ac.y * offsetB.z;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = z_ac_abs * extents.y + y_ac_abs * extents.z;
				if (min > rad || max < -rad) return;
			}
			{
				float min = -ac.z * offsetA.x + ac.x * offsetA.z,
					  max = -ac.z * offsetB.x + ac.x * offsetB.z;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = z_ac_abs * extents.x + x_ac_abs * extents.z;
				if (min > rad || max < -rad) return;
			}
			{
				float min = ac.y * offsetB.x - ac.x * offsetB.y,
					  max = ac.y * offsetC.x - ac.x * offsetC.y;
				if (min > max) {
					float temp = min;
					min = max;
					max = temp;
				}
				float rad = y_ac_abs * extents.x + x_ac_abs * extents.y;
				if (min > rad || max < -rad) return;
			}
			{
				Vector3 normal = Vector3.Cross(ba, cb);
				Vector3 min, max;
				if (normal.x > 0) {
					min.x = -extents.x - offsetA.x;
					max.x =  extents.x - offsetA.x;
				} else {
					min.x =  extents.x - offsetA.x;
					max.x = -extents.x - offsetA.x;
				}
				if (normal.y > 0) {
					min.y = -extents.y - offsetA.y;
					max.y =  extents.y - offsetA.y;
				} else {
					min.y =  extents.y - offsetA.y;
					max.y = -extents.y - offsetA.y;
				}
				if (normal.z > 0) {
					min.z = -extents.z - offsetA.z;
					max.z =  extents.z - offsetA.z;
				} else {
					min.z =  extents.z - offsetA.z;
					max.z = -extents.z - offsetA.z;
				}
				if (Vector3.Dot(normal, min) > 0) return;
				if (Vector3.Dot(normal, max) < 0) return;
			}
			{
				Vector3 min = Vector3.Min(Vector3.Min(offsetA, offsetB), offsetC);
				Vector3 max = Vector3.Max(Vector3.Max(offsetA, offsetB), offsetC);
				if (min.x > extents.x || max.x < -extents.x) return;
				if (min.y > extents.y || max.y < -extents.y) return;
				if (min.z > extents.z || max.z < -extents.z) return;
			}
			triIndices.Add(index);
		}

		public void Subdivide() {
			if (children == null) {
				Vector3 e = box.Extents / 2;
				Vector3 ex = Vector3.right * e.x;
				Vector3 ey = Vector3.up * e.y;
				Vector3 ez = Vector3.forward * e.z;

				children = new Octree[8] {
					new Octree(new Box(box.Bounds[0],           box.Bounds[0] + e     ), depth + 1, id*10),
					new Octree(new Box(box.Bounds[0] + ez,      box.Bounds[0] + ez + e), depth + 1, id*10+1),
					new Octree(new Box(box.Bounds[0] + ey,      box.Bounds[0] + ey + e), depth + 1, id*10+2),
					new Octree(new Box(box.Bounds[0] + ey + ez, box.Bounds[1] - ex    ), depth + 1, id*10+3),
					new Octree(new Box(box.Bounds[0] + ex,      box.Bounds[0] + ex + e), depth + 1, id*10+4),
					new Octree(new Box(box.Bounds[0] + ex + ez, box.Bounds[1] - ey    ), depth + 1, id*10+5),
					new Octree(new Box(box.Bounds[0] + ex + ey, box.Bounds[1] - ez    ), depth + 1, id*10+6),
					new Octree(new Box(box.Bounds[0] + e,       box.Bounds[1]         ), depth + 1, id*10+7)
				};
				children[0].neighbors = new Octree[6] { neighbors[0], children[1],  neighbors[1], children[4],  neighbors[5], children[2]  };
				children[1].neighbors = new Octree[6] { children[0],  neighbors[1], neighbors[2], children[5],  neighbors[5], children[3]  };
				children[2].neighbors = new Octree[6] { neighbors[0], children[3],  neighbors[2], children[6],  children[0],  neighbors[4] };
				children[3].neighbors = new Octree[6] { children[2],  neighbors[1], neighbors[2], children[7],  children[1],  neighbors[4] };
				children[4].neighbors = new Octree[6] { neighbors[0], children[5],  children[0],  neighbors[3], neighbors[5], children[6]  };
				children[5].neighbors = new Octree[6] { children[4],  neighbors[1], children[1],  neighbors[3], neighbors[5], children[7]  };
				children[6].neighbors = new Octree[6] { neighbors[0], children[7],  children[2],  neighbors[3], children[4],  neighbors[4] };
				children[7].neighbors = new Octree[6] { children[6],  neighbors[1], children[3],  neighbors[3], children[5],  neighbors[4] };
			}
		}

		public void Draw(Color color) {
			box.Draw(color);
			if (children != null) {
				DrawLeaves(color);
			}
		}

		private void DrawLeaves(Color color) {
			if (children == null) {
				if (triIndices.Count > 0) {
					box.Draw(color);
				}
			} else {
				foreach (Octree child in children) {
					child.DrawLeaves(color);
				}
			}
		}
	}
	
    void Update()
    {
		UnityEngine.Random.state = randState;
		
		if (Deepen && canDeepen) {
			Deepen = false;
			canDeepen = o.Deepen(MinTrisPerOctant);
		}
		Ray r = new Ray(transform.InverseTransformPoint(Camera.position), transform.InverseTransformDirection(Camera.forward));
		o.Draw(new Color(0,0,0,0.5f));
        /*
		if (o.RayTrace(r, out Hit hit)) {
			Debug.DrawLine(transform.TransformPoint(r.Start), transform.TransformPoint(hit.pt));
			Vector3 A = transform.TransformPoint(verts[tris[3 * hit.triIndex + 0]]);
			Vector3 B = transform.TransformPoint(verts[tris[3 * hit.triIndex + 1]]);
			Vector3 C = transform.TransformPoint(verts[tris[3 * hit.triIndex + 2]]);

			Vector3 Na = transform.TransformDirection(normals[tris[3 * hit.triIndex + 0]]);
			Vector3 Nb = transform.TransformDirection(normals[tris[3 * hit.triIndex + 1]]);
			Vector3 Nc = transform.TransformDirection(normals[tris[3 * hit.triIndex + 2]]);

			Vector3 N = ((1f - hit.uv[0] - hit.uv[1]) * Na + hit.uv[0] * Nb + hit.uv[1] * Nc).normalized;
			Debug.DrawRay(transform.TransformPoint(hit.pt), N, Color.blue);

			Debug.DrawRay(A, Na, Color.cyan);
			Debug.DrawRay(B, Nb, Color.cyan);
			Debug.DrawRay(C, Nc, Color.cyan);
		}
        */
		/*
		float viewH = Mathf.Tan(Mathf.Deg2Rad*fov / 2) * 2;
		for (int y=0; y<height; y++) {
			for (int x=0; x<width; x++) {
				Vector3 dir = (Camera.transform.forward + Camera.transform.right * viewH * (-0.5f + (float)x / (width-1)) * width/height + Camera.transform.up * viewH * (-0.5f + (float)y / (height-1))).normalized;
				//Debug.DrawRay(Camera.transform.position, dir);
				Ray r = new Ray(transform.InverseTransformPoint(Camera.position), transform.InverseTransformDirection(dir));
				if (o.RayTrace(r, out Hit hit)) {
					Debug.DrawLine(transform.TransformPoint(r.Start), transform.TransformPoint(hit.pt));
				}
			}
		}
		*/
	}
}
