using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {

        public class GeometryTool
        {

            /// <summary>
            /// Intersect line and sphere
            /// </summary>
            /// <param name="p"></param>
            /// <param name="d"></param>
            /// <param name="s"></param>
            /// <param name="p1"></param>
            /// <param name="p2"></param>
            /// <returns></returns>
            public static int IntersectLineSphere(Vector3 p, Vector3 d, Sphere s, ref Vector3 p1, ref Vector3 p2)
            {
                Vector3 m = p - s.c;
                float b = Vector3.Dot(m, d);
                float c = Vector3.Dot(m, m) - s.r * s.r;
                float disr = b * b - c;
                if (disr < 0.0f) return 0;
                else if (disr < Mathf.Epsilon)
                {
                    p1 = p + d * (-b + Mathf.Sqrt(disr));
                    p2 = p1;
                    return 1;
                }
                float sqrtD = Mathf.Sqrt(disr);
                p1 = p + d * (-b + sqrtD);
                p2 = p + d * (-b - sqrtD);
                float distance = (p1 - p2).magnitude;

                return 2;
            }

            /// <summary>
            /// Compute intersection between planes
            /// </summary>
            /// <param name="p1"></param>
            /// <param name="p2"></param>
            /// <param name="d"></param>
            /// <param name="p"></param>
            /// <returns></returns>
            public static int IntersectPlanes(Plane p1, Plane p2, ref Vector3 d, ref Vector3 p)
            {
                d = Vector3.Cross(p1.n, p2.n);

                float dnom = d.magnitude;

                if (dnom < Mathf.Epsilon) return 0;

                //p = Vector3.Cross(p1.d * p2.n - p2.d * p1.n, d) / dnom;

                float d11 = Vector3.Dot(p1.n, p1.n);
                float d12 = Vector3.Dot(p1.n, p2.n);
                float d22 = Vector3.Dot(p2.n, p2.n);

                dnom = d11 * d22 - d12 * d12;
                float k1 = (p1.d * d22 - p2.d * d12) / dnom;
                float k2 = (p2.d * d11 - p1.d * d12) / dnom;

                p = k1 * p1.n + k2 * p2.n;

                return 1;
            }

            /// <summary>
            /// Compute plane and line
            /// </summary>
            /// <param name="a"></param>
            /// <param name="b"></param>
            /// <param name="p"></param>
            /// <returns></returns>
            public static Vector3 IntersectLinePlane(Vector3 a, Vector3 b, Plane p)
            {
                Vector3 ab = b - a;
                float x = Vector3.Dot(p.n, ab);
                float t;
                if (x != 0) t = (p.d - Vector3.Dot(p.n, a)) / x;
                else t = (p.d - Vector3.Dot(p.n, a)) / ab.magnitude;
                return a + t * ab;

            }

            public static Vector3 FlipPointByPlane(Plane plane,Vector3 v)
            {
                return v - 2 * (Vector3.Dot(v, plane.n) - plane.d) * plane.n;
            }
            
            /// <summary>
            /// Decompose polygon sphere into two convex polygons
            /// </summary>
            /// <param name="polygon"></param>
            /// <returns></returns>
            public static Polygon[] DecomposePolygonLimit(PolygonLimit polygon)
            {
                //convex decompose
                List<Vector3> bound1 = new List<Vector3>(), bound2 = new List<Vector3>();
                bool flag = true;

                Plane plane = new Plane(polygon.axis, Vector3.Dot(polygon.axis, polygon.s.c));

                if (Vector3.Dot(polygon.boundPoints[0] - polygon.s.c, polygon.axis) > 0)
                {
                    bound1.Add(polygon.boundPoints[0]);
                }
                else
                {
                    bound2.Add(polygon.boundPoints[0]);
                    flag = false;
                }

                for (int i = 1; i < polygon.boundPoints.Length; ++i)
                {
                    if (flag)
                    {
                        if (Vector3.Dot(polygon.boundPoints[i] - polygon.s.c, polygon.axis) > 0)
                        {
                            bound1.Add(polygon.boundPoints[i]);
                        }
                        else
                        {
                            Vector3 interPoint = polygon.s.r * (IntersectLinePlane(polygon.boundPoints[i - 1], polygon.boundPoints[i], plane) - polygon.s.c).normalized + polygon.s.c;

                            bound1.Add(interPoint);
                            bound2.Add(interPoint);
                            bound2.Add(polygon.boundPoints[i]);
                            flag = !flag;
                        }
                    }
                    else
                    {
                        if (Vector3.Dot(polygon.boundPoints[i] - polygon.s.c, polygon.axis) < 0)
                        {
                            bound2.Add(polygon.boundPoints[i]);
                        }
                        else
                        {
                            Vector3 interPoint = polygon.s.r * (IntersectLinePlane(polygon.boundPoints[i - 1], polygon.boundPoints[i], plane) - polygon.s.c).normalized + polygon.s.c;

                            bound2.Add(interPoint);
                            bound1.Add(interPoint);
                            bound1.Add(polygon.boundPoints[i]);
                            flag = !flag;
                        }
                    }
                }
                Polygon p1 = new Polygon(polygon.s, bound1.ToArray());
                return new Polygon[2] { new Polygon(polygon.s, bound1.ToArray()), new Polygon(polygon.s, bound2.ToArray()) };
            }
            
            /// <summary>
            /// cone intersect circle
            /// </summary>
            /// <param name="s"></param>
            /// <param name="cone"></param>
            /// <param name="swivel"></param>
            /// <param name="zeroSwivel"></param>
            /// <param name="intervals"></param>
            /// <returns></returns>
            public static int IntersectionPolygonCircle(Sphere s, Plane[] planes, Plane circle, Circle swivel, Vector3 zeroSwivel, ref Interval[] intervals)
            {
                List<Interval[]> intervalsList = new List<Interval[]>();

                for (int i = 0; i < 3; ++i)
                {
                    Interval[] tmp = new Interval[0];
                    if (IntersectionPlaneCircle(s, planes[i], circle, swivel, zeroSwivel, ref tmp) == 0) return 0;
                    if (tmp.Length > 0) intervalsList.Add(tmp);
                }
                if (intervalsList.Count > 0)
                {
                    intervals = Interval.GetIntersection(intervalsList);
                }
                return 1;
            }

            /// <summary>
            /// plane intersect circle  
            /// </summary>
            /// <param name="s"></param>
            /// <param name="plane"></param>
            /// <param name="circle"></param>
            /// <param name="swivel"></param>
            /// <param name="zeroSwivel"></param>
            /// <param name="intervals"></param>
            /// <returns></returns>
            public static int IntersectionPlaneCircle(Sphere s, Plane plane, Plane circle, Circle swivel, Vector3 zeroSwivel, ref Interval[] intervals)
            {
                Vector3 p = Vector3.zero, d = Vector3.zero;

                if (IntersectPlanes(circle, plane, ref d, ref p) == 1)
                {
                    //Debug.DrawLine(p-10*d, p + 10 * d,Color.red);
                    //Debug.Log(Vector3.Dot(d, plane.n) + "-" + Vector3.Dot(d, circle.n));
                    Vector3 p1 = Vector3.zero, p2 = Vector3.zero;
                    int intersection = IntersectLineSphere(p, d.normalized, s, ref p1, ref p2);//line intersect sphere
                    if (intersection == 2)
                    {
                        //if (isDraw)
                        //{
                        //    Debug.DrawLine(p1, p2, Color.red);
                        //}
                        float min = Vector3.SignedAngle(zeroSwivel - swivel.c, p1 - swivel.c, swivel.n);
                        float max = Vector3.SignedAngle(zeroSwivel - swivel.c, p2 - swivel.c, swivel.n);

                        if (min > max) MathTool.Swap(ref min, ref max);
                        float tmp = Vector3.Dot(zeroSwivel - s.c, plane.n);

                        if ((tmp > 0 && (min > 0 || max < 0)) || (tmp < 0 && max > 0 && min < 0))
                        {
                            intervals = new Interval[2] { new Interval(-180, min), new Interval(max, 180) };
                        }
                        else if ((tmp > 0 && min < 0 && max > 0) || (tmp < 0 && (min > 0 || max < 0)))
                        {
                            intervals = new Interval[1] { new Interval(min, max) };
                        }
                        else if (tmp == 0 ||min==0||max==0)
                        {
                            Vector3 swivelMid = Quaternion.AngleAxis((min + max) / 2, swivel.n) * (zeroSwivel - swivel.c) + swivel.c;
                            if (Vector3.Dot(swivelMid - s.c, plane.n) > 0) intervals = new Interval[1] { new Interval(min, max) };
                            else intervals = new Interval[2] { new Interval(-180, min), new Interval(max, 180) };
                        }
                    }
                    else if ((intersection == 1 && Vector3.Dot(plane.n, swivel.n) > 0) || (intersection == 0 && Vector3.Dot(plane.n, swivel.n) > 0))
                    {
                        intervals = new Interval[1] { new Interval(-180, 180) };
                    }
                    else return 0;
                }
                return 1;
            }

        }

    }
}



