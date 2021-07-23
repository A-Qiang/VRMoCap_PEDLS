using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        enum Feature
        {
            Edge,
            Face
        }
        public class DrawTool
        {
            #region draw

            public static void DrawCapsule(Capsule capsule,Quaternion rotation,Axis axis,Color color)
            {
                Vector3 normal, normal1, normal2; 
                switch (axis)
                {
                    case Axis.X:
                        normal = rotation*Vector3.right;
                        normal1 = rotation * Vector3.up;
                        normal2 = rotation * Vector3.forward;
                        break;
                    case Axis.Y:
                        normal = rotation * Vector3.up;
                        normal1 = rotation * Vector3.forward;
                        normal2 = rotation * Vector3.right;
                        break;
                    default:
                        normal = rotation * Vector3.forward;
                        normal1 = rotation * Vector3.right;
                        normal2 = rotation * Vector3.up;
                        break;
                }

                DrawArc(new Circle(capsule.a, normal, capsule.r), new Interval(-180, 180), capsule.a+normal1 * capsule.r, color);
                DrawArc(new Circle(capsule.b, normal, capsule.r), new Interval(-180, 180),capsule.b+ normal1 * capsule.r, color);

                DrawArc(new Circle(capsule.a, normal1, capsule.r), new Interval(-180, 0), capsule.a+normal2 * capsule.r, color);
                DrawArc(new Circle(capsule.a, normal2, capsule.r), new Interval(0, 180), capsule.a+normal1 * capsule.r, color);
                DrawArc(new Circle(capsule.b, normal1, capsule.r), new Interval(0, 180), capsule.b+normal2 * capsule.r, color);
                DrawArc(new Circle(capsule.b, normal2, capsule.r), new Interval(-180, 0), capsule.b+normal1 * capsule.r, color);

                Debug.DrawLine(capsule.a + normal1 * capsule.r, capsule.b + normal1 * capsule.r, color);
                Debug.DrawLine(capsule.a + normal2 * capsule.r, capsule.b + normal2 * capsule.r, color);
                Debug.DrawLine(capsule.a - normal1 * capsule.r, capsule.b - normal1 * capsule.r, color);
                Debug.DrawLine(capsule.a - normal2 * capsule.r, capsule.b - normal2 * capsule.r, color);
            }

            public static void DrawSphere(Sphere s,Mesh mesh,Material mat,float a)
            {
                mat.color = new Color(1.0f, 1.0f, 1.0f, a);
                Graphics.DrawMesh(mesh, Matrix4x4.TRS(s.c, Quaternion.identity, new Vector3(2 * s.r, 2 * s.r, 2 * s.r)), mat, 0);
            }

            /// <summary>
            /// Display sphere polygon in face
            /// </summary>
            /// <param name="polygon"></param>
            /// <param name="color"></param>
            public static void DrawPolygon(Polygon polygon,Color color)
            {
                if (polygon.boundPoints.Length < 3) return;

                for (int i = 0; i < polygon.boundPoints.Length - 1; ++i)
                {
                    DrawDisk(polygon.s.c, polygon.boundNormal[i], polygon.boundPoints[i], polygon.boundPoints[i + 1], color);
                }
                DrawDisk(polygon.s.c, polygon.boundNormal[polygon.boundNormal.Length - 1], polygon.boundPoints[polygon.boundPoints.Length - 1], polygon.boundPoints[0], color);
            }

            public static void DrawPolygon(Vector3 c,Vector3[] verts,Color color)
            {
                for(int i = 0; i < verts.Length-1; ++i)
                {
                    DrawArc(c, verts[i], verts[i + 1], color);
                }
                DrawArc(c, verts[verts.Length - 1], verts[0], color);
            }

            public static void DrawArc(Vector3 c,Vector3 from,Vector3 to,Color color)
            {
                Vector3 n = Vector3.Cross(from - c, to - c);
                float angle = Vector3.SignedAngle(from - c, to - c, n);

                Vector3 p1=from, p2;
                for(int i = 1; i < angle;++i)
                {
                    p2 = Quaternion.AngleAxis(i, n) * (from - c) + c;
                    Debug.DrawLine(p1, p2, color);
                    p1 = p2;
                }
                Debug.DrawLine(p1, to, color);
            }

            public static void DrawArc(Circle s, Interval interval, Vector3 zero, Color color)
            {
                Vector3 p1, p2;
                p1 = Quaternion.AngleAxis(interval.start, s.n) * (zero - s.c) + s.c;
                for (int i = 1; i < interval.end - interval.start; ++i)
                {
                    p2 = Quaternion.AngleAxis(interval.start + i, s.n) * (zero - s.c) + s.c;
                    Debug.DrawLine(p1, p2, color);
                    p1 = p2;
                }
                p2 = Quaternion.AngleAxis(interval.end, s.n) * (zero - s.c) + s.c;
                Debug.DrawLine(p1, p2, color);
            }

            /// <summary>
            /// Draw disk
            /// </summary>
            /// <param name="v"></param>
            /// <param name="n"></param>
            /// <param name="start"></param>
            /// <param name="end"></param>
            /// <param name="color"></param>
            public static void DrawDisk(Vector3 v, Vector3 n, Vector3 start, Vector3 end, Color color)
            {
                Vector3 p1 = start - v;
                Vector3 p2 = end - v;
                float angle = Vector3.SignedAngle(p1, p2, n);
                if (angle < 0) angle = 180 - angle;
                Quaternion q = Quaternion.AngleAxis(1, n);
                for (int i = 1; i < angle; ++i)
                {
                    p2 = q * p1;
                    Debug.DrawRay(v, p1, color);
                    Debug.DrawRay(v, p2, color);
                    Debug.DrawLine(p1 + v, p2 + v, color);
                    p1 = p2;
                }
                p2 = end - v;
                Debug.DrawRay(v, p2, color);
                Debug.DrawLine(p1 + v, p2 + v, color);
            }

            public static void DrawDisk(Vector3 v, Vector3 n, Vector3 start, float angle, Color color)
            {
                Vector3 p1 = start - v, p2;
                Quaternion q = Quaternion.AngleAxis(1, n);
                for (int i = 1; i < angle; ++i)
                {
                    p2 = q * p1;
                    Debug.DrawRay(v, p1, color);
                    Debug.DrawRay(v, p2, color);
                    Debug.DrawLine(p1 + v, p2 + v, color);
                    p1 = p2;
                }
                q = Quaternion.AngleAxis(angle, n);
                p2 = q * (start - v);
                Debug.DrawRay(v, p2, color);
                Debug.DrawLine(p1 + v, p2 + v, color);
            }

            #endregion
        }

    }
}

