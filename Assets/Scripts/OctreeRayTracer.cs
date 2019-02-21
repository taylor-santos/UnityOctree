using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text;

[RequireComponent(typeof(MeshFilter))]
public class OctreeRayTracer : MonoBehaviour
{
	public Transform Camera;
	public bool Deepen;
	private Octree o;
	private Mesh mesh;
	private static int[] tris;
	private static Vector3[] verts;

	private UnityEngine.Random.State randState;

	void Start()
    {
		mesh = GetComponent<MeshFilter>().mesh;
		tris = mesh.triangles;
		verts = mesh.vertices;
		Vector3 corner1 = verts[0];
		Vector3 corner2 = verts[0];
		foreach (Vector3 vert in verts) {
			corner1 = Vector3.Min(corner1, vert);
			corner2 = Vector3.Max(corner2, vert);
		}
		o = new Octree(new Box(corner1, corner2));
		for (int i=0; i<tris.Length/3; i++) { 
			/*
			Vector3 A = verts[tris[3 * i + 0]];
			Vector3 B = verts[tris[3 * i + 1]];
			Vector3 C = verts[tris[3 * i + 2]];
			Vector3 a = transform.TransformPoint(A);
			Vector3 b = transform.TransformPoint(B);
			Vector3 c = transform.TransformPoint(C);
			Debug.DrawLine(a, b, Color.red);
			Debug.DrawLine(b, c, Color.red);
			Debug.DrawLine(c, a, Color.red);
			*/
			o.AddTriangle(i);
		}
		randState = UnityEngine.Random.state;
    }

	class Ray {
		public Ray(Vector3 s, Vector3 d) {
			Start = s;
			Dir = d;
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

		public void Draw(Color color, Transform transform) {
			Vector3 c000 = transform.TransformPoint(Bounds[0]);
			Vector3 c111 = transform.TransformPoint(Bounds[1]);

			Vector3 c100 = transform.TransformPoint(Bounds[0] + Extents.x * Vector3.right);
			Vector3 c010 = transform.TransformPoint(Bounds[0] + Extents.y * Vector3.up);
			Vector3 c001 = transform.TransformPoint(Bounds[0] + Extents.z * Vector3.forward);

			Vector3 c110 = transform.TransformPoint(Bounds[0] + Extents.x * Vector3.right + Extents.y * Vector3.up);
			Vector3 c101 = transform.TransformPoint(Bounds[0] + Extents.x * Vector3.right + Extents.z * Vector3.forward);
			Vector3 c011 = transform.TransformPoint(Bounds[0] + Extents.y * Vector3.up + Extents.z * Vector3.forward);

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
		public Octree(Box box) {
			this.box = box;
			children = null;
			depth = 0;
			triIndices = new List<int>();
			id = 0;
		}
		private Octree(Box box, int depth, int id) {
			this.box = box;
			children = null;
			this.depth = depth;
			triIndices = new List<int>();
			this.id = id;
		}
		public Box box { get; }
		Octree[] children;
		private List<int> triIndices;
		public int depth { get; }
		int id;

		public void Deepen() {
			if (children == null) {
				if (triIndices.Count > 1) {
					Subdivide();
					foreach (int tri in triIndices) {
						for (int i = 0; i < 8; i++) {
							children[i].AddTriangle(tri);
						}
					}
				}
			} else {
				for (int i = 0; i < 8; i++) {
					children[i].Deepen();
				}
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
				if (min > max) { float tmp = min; min = max; max = tmp; }
				float rad = z_ac_abs * extents.y + y_ac_abs * extents.z;
				if (min > rad || max < -rad) return;
			}
			{
				float min = -ac.z * offsetA.x + ac.x * offsetA.z,
					  max = -ac.z * offsetB.x + ac.x * offsetB.z;
				if (min > max) { float tmp = min; min = max; max = tmp; }
				float rad = z_ac_abs * extents.x + x_ac_abs * extents.z;
				if (min > rad || max < -rad) return;
			}
			{
				float min = ac.y * offsetB.x - ac.x * offsetB.y,
					  max = ac.y * offsetC.x - ac.x * offsetC.y;
				if (min > max) { float tmp = min; min = max; max = tmp; }
				float rad = y_ac_abs * extents.x + x_ac_abs * extents.y;
				if (min > rad || max < -rad) return;
			}
			{
				Vector3 normal = Vector3.Cross(ba, cb);
				Vector3 min, max;
				if (normal.x > 0) {
					min.x = -extents.x - offsetA.x;
					max.x = extents.x - offsetA.x;
				} else {
					min.x = extents.x - offsetA.x;
					max.x = -extents.x - offsetA.x;
				}
				if (normal.y > 0) {
					min.y = -extents.y - offsetA.y;
					max.y = extents.y - offsetA.y;
				} else {
					min.y = extents.y - offsetA.y;
					max.y = -extents.y - offsetA.y;
				}
				if (normal.z > 0) {
					min.z = -extents.z - offsetA.z;
					max.z = extents.z - offsetA.z;
				} else {
					min.z = extents.z - offsetA.z;
					max.z = -extents.z - offsetA.z;
				}
				if (Vector3.Dot(normal, min) > 0) return;
				if (Vector3.Dot(normal, max) < 0) return;
			}
			{
				Vector3 min = Vector3.Min(offsetA, offsetB);
				min = Vector3.Min(min, offsetC);
				Vector3 max = Vector3.Max(offsetA, offsetB);
				max = Vector3.Max(max, offsetC);
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
					new Octree(new Box(box.Bounds[0], box.Bounds[0] + e), depth + 1, id*10),
					new Octree(new Box(box.Bounds[0] + ez, box.Bounds[0] + ez + e), depth + 1, id*10+1),
					new Octree(new Box(box.Bounds[0] + ey, box.Bounds[0] + ey + e), depth + 1, id*10+2),
					new Octree(new Box(box.Bounds[0] + ey + ez, box.Bounds[1] - ex), depth + 1, id*10+3),
					new Octree(new Box(box.Bounds[0] + ex, box.Bounds[0] + ex + e), depth + 1, id*10+4),
					new Octree(new Box(box.Bounds[0] + ex + ez, box.Bounds[1] - ey), depth + 1, id*10+5),
					new Octree(new Box(box.Bounds[0] + ex + ey, box.Bounds[1] - ez), depth + 1, id*10+6),
					new Octree(new Box(box.Bounds[0] + e, box.Bounds[1]), depth + 1, id*10+7)
				};
			}
		}

		public void Draw(Color color, Transform transform) {
			box.Draw(color, transform);
			if (children != null) {
				Color nextColor = UnityEngine.Random.ColorHSV(0, 1, 1, 1, 0.5f, 1);
				nextColor.a = 1f / (depth + 2);
				DrawLeaves(Color.black, transform);
			}
		}

		private void DrawLeaves(Color color, Transform transform) {
			if (children == null) {
				if (triIndices.Count > 0) {
					color = Color.HSVToRGB(triIndices.Count / 6f, 0.25f, 1);
					//color.a = 1f / (depth + 1);
					box.Draw(color, transform);
				}
			} else {
				foreach (Octree child in children) {
					child.DrawLeaves(color, transform);
				}
			}
		}

		private void DrawInternal(Color color, Transform transform) {
			Vector3 e = box.Extents;
			Vector3 ex = transform.TransformVector(Vector3.right * e.x);
			Vector3 ey = transform.TransformVector(Vector3.up * e.y);
			Vector3 ez = transform.TransformVector(Vector3.forward * e.z);
			Vector3 b0 = transform.TransformPoint(box.Bounds[0]);
			Vector3 b1 = transform.TransformPoint(box.Bounds[1]);
			Debug.DrawRay(b0 + ex / 2, ez, color);
			Debug.DrawRay(b0 + ex / 2, ey, color);
			Debug.DrawRay(b0 + ey / 2, ex, color);
			Debug.DrawRay(b0 + ey / 2, ez, color);
			Debug.DrawRay(b0 + ez / 2, ex, color);
			Debug.DrawRay(b0 + ez / 2, ey, color);

			Debug.DrawRay(b1 - ex / 2, -ez, color);
			Debug.DrawRay(b1 - ex / 2, -ey, color);
			Debug.DrawRay(b1 - ey / 2, -ex, color);
			Debug.DrawRay(b1 - ey / 2, -ez, color);
			Debug.DrawRay(b1 - ez / 2, -ex, color);
			Debug.DrawRay(b1 - ez / 2, -ey, color);

			Debug.DrawRay(b0 + ey / 2 + ez / 2, ex, color);
			Debug.DrawRay(b0 + ex / 2 + ez / 2, ey, color);
			Debug.DrawRay(b0 + ex / 2 + ey / 2, ez, color);

			Color nextColor = UnityEngine.Random.ColorHSV(0, 1, 1, 1, 0.5f, 1);
			nextColor.a = 1f / (depth + 3);
			foreach (Octree child in children) {
				if (child.children != null) {
					child.DrawInternal(nextColor, transform);
				}
			}
		}

		public Octree this[float x, float y, float z] {
			get {
				if (children == null) {
					return this;
				}
				if (x < 0 || 1 < x || y < 0 || 1 < y || z < 0 || 1 < z)
					throw new IndexOutOfRangeException();
				int index = 0;
				if (x >= 0.5f) {
					x -= 0.5f;
					index += 4;
				}
				if (y >= 0.5f) {
					y -= 0.5f;
					index += 2;
				}
				if (z >= 0.5f) {
					z -= 0.5f;
					index += 1;
				}
				x *= 2;
				y *= 2;
				z *= 2;
				return children[index][x, y, z];
			}
		}
	}

    // Update is called once per frame
    void Update()
    {
		UnityEngine.Random.state = randState;
		Ray r = new Ray(transform.InverseTransformPoint(Camera.position), transform.InverseTransformDirection(Camera.forward));
		if (Deepen) {
			Deepen = false;
			o.Deepen();
		}
		o.Draw(Color.white, transform);
		/*
		Vector2 d = new Vector2();
		if (intersect(r, b, ref d)) {
			d[0] = Mathf.Max(d[0], 0);
			Vector3 intersect = r.Start + r.Dir * d[0];
			Debug.DrawLine(Camera.position, transform.TransformPoint(intersect));
			Vector3 uv = intersect - b.Bounds[0];
			uv = new Vector3(
				uv.x / b.Extents.x,
				uv.y / b.Extents.y,
				uv.z / b.Extents.z
			);
			Box b2 = b;
			for (int i = 0; i < 5; i++) {
				Vector3 corner = b2.Bounds[0];
				if (uv.x >= 0.5f) {
					corner.x += b2.Extents.x / 2;
					uv.x -= 0.5f;
				}
				if (uv.y >= 0.5f) {
					corner.y += b2.Extents.y / 2;
					uv.y -= 0.5f;
				}
				if (uv.z >= 0.5f) {
					corner.z += b2.Extents.z / 2;
					uv.z -= 0.5f;
				}
				uv *= 2;
				b2 = new Box(corner, corner + b2.Extents / 2);
				DrawCuboid(b2, Color.white);
			}
		}
		*/
    }

	bool intersect(Ray r, Box b, ref Vector2 d) {
		/* http://www.cs.utah.edu/~awilliam/box/box.pdf */
		d = new Vector2(
			(b.Bounds[r.sign[0]].x - r.Start.x) * r.Inv_dir.x,
			(b.Bounds[1 - r.sign[0]].x - r.Start.x) * r.Inv_dir.x
		);
		Vector3 hit = r.Start + d[0] * r.Dir - b.Bounds[0];
		float tymin = (b.Bounds[r.sign[1]].y - r.Start.y) * r.Inv_dir.y;
		float tymax = (b.Bounds[1 - r.sign[1]].y - r.Start.y) * r.Inv_dir.y;
		if ((d[0] > tymax) || (tymin > d[1])) {
			return false;
		}
		if (tymin > d[0]) {
			d[0] = tymin;
			hit = r.Start + d[0] * r.Dir - b.Bounds[0];
		}
		if (tymax < d[1]) {
			d[1] = tymax;
		}
		float tzmin = (b.Bounds[r.sign[2]].z - r.Start.z) * r.Inv_dir.z;
		float tzmax = (b.Bounds[1 - r.sign[2]].z - r.Start.z) * r.Inv_dir.z;
		if ((d[0] > tzmax) || (tzmin > d[1])) {
			return false;
		}
		if (tzmin > d[0]) {
			d[0] = tzmin;
			hit = r.Start + d[0] * r.Dir - b.Bounds[0];
		}
		if (tzmax < d[1]) {
			d[1] = tzmax;
		}
		
		return d[1] > 0;
	}
}
