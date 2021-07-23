using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        public class JointLimitTool
        {
            public static Interval[] ConvertReverseInterval(Interval[] reverseIntervals, float offset)
            {
                List<Interval> intervalList = new List<Interval>();
                for (int i = 0; i < reverseIntervals.Length; ++i)
                {
                    float start = offset - reverseIntervals[i].end;
                    float end = offset - reverseIntervals[i].start;
                    if (start >= -180 || end <= 180) intervalList.Add(new Interval(start, end));
                    else if (end <= -180) intervalList.Add(new Interval(start + 180, end + 180));
                    else if (start >= 180) intervalList.Add(new Interval(start - 180, end - 180));
                    else if (start < 180 && end > 180)
                    {
                        intervalList.Add(new Interval(-180, end - 360));
                        intervalList.Add(new Interval(start, 180));
                    }
                    else if (start < -180 && end > -180)
                    {
                        intervalList.Add(new Interval(-180, end));
                        intervalList.Add(new Interval(start + 360, 180));
                    }
                }
                return intervalList.ToArray();

            }

            public static void ReversePolygon(PolygonLimit polygon, PolygonLimit newPolygon, Vector3 newPosition, Quaternion newRotation, Vector3 mirNormal)
            {
                if (polygon.boundPoints == null) return;

                newPolygon.s.c = newPosition;
                newPolygon.s.r = polygon.s.r;
                newPolygon.smoothness = polygon.smoothness;
                newPolygon.axis =  Vector3.Reflect(newRotation *polygon.axis, mirNormal);
                newPolygon.twistAngle = polygon.twistAngle;

                int len = polygon.boundPoints.Length;
                newPolygon.boundPoints = new Vector3[len];

                Plane p = new Plane(mirNormal, Vector3.Dot(mirNormal, newPosition));
                for (int i = 0; i < len; ++i)
                {
                    newPolygon.boundPoints[i] = GeometryTool.FlipPointByPlane(p, newRotation * (polygon.boundPoints[len - 1 - i] - polygon.s.c) + newPosition);
                }
                newPolygon.ReculculateCones();
            }

            /// <summary>
            /// Get the interval of twist limit
            /// </summary>
            /// <param name="q"></param>
            /// <param name="n"></param>
            /// <param name="twistLimit"></param>
            /// <param name="swivel"></param>
            public static void GetTwistSwivel(Quaternion q, Vector3 n, Interval twistLimit, ref Interval swivel, Axis axis = Axis.Z)
            {
                float A, B, C, D, minSwivel, maxSwivel, minAngle, maxAngle;
                minAngle = twistLimit.start * Mathf.Deg2Rad / 2;
                maxAngle = twistLimit.end * Mathf.Deg2Rad / 2;
                C = q.w;
                D = -Vector3.Dot(new Vector3(q.x, q.y, q.z), n);

                switch (axis)
                {
                    case Axis.X:
                        A = q.x;
                        B = n.y * q.z - n.z * q.y + n.x * q.w;
                        break;
                    case Axis.Y:
                        A = q.y;
                        B = n.z * q.x - n.x * q.z + n.y * q.w;
                        break;
                    default:
                        A = q.z;
                        B = n.x * q.y - n.y * q.x + n.z * q.w;
                        break;
                }
                minSwivel = -2 * Mathf.Atan((A - C * Mathf.Tan(minAngle)) / (B - D * Mathf.Tan(minAngle))) * Mathf.Rad2Deg;
                maxSwivel = -2 * Mathf.Atan((A - C * Mathf.Tan(maxAngle)) / (B - D * Mathf.Tan(maxAngle))) * Mathf.Rad2Deg;
                if (A * D - B * C < 0) swivel = new Interval(minSwivel, maxSwivel);
                else swivel = new Interval(maxSwivel, minSwivel);
            }

            /// <summary>
            /// Get the twist angle given the swivel angle
            /// </summary>
            /// <param name="q"></param>
            /// <param name="n"></param>
            /// <param name="swivelAngle"></param>
            /// <param name="axis"></param>
            /// <returns></returns>
            public static float GetTwist(Quaternion q, Vector3 n, float swivelAngle, Axis axis = Axis.Z)
            {
                float A, B, C, D;
                swivelAngle = Mathf.Deg2Rad * swivelAngle / 2;
                C = q.w;
                D = -Vector3.Dot(new Vector3(q.x, q.y, q.z), n);
                switch (axis)
                {
                    case Axis.X:
                        A = q.x;
                        B = n.y * q.z - n.z * q.y + n.x * q.w;
                        break;
                    case Axis.Y:
                        A = q.y;
                        B = n.z * q.x - n.x * q.z + n.y * q.w;
                        break;
                    default:
                        A = q.z;
                        B = n.x * q.y - n.y * q.x + n.z * q.w;
                        break;
                }
                float cosVal = Mathf.Cos(swivelAngle);
                float sinVal = Mathf.Sin(swivelAngle);
                return 2 * Mathf.Atan((A * cosVal + B * sinVal) / (C * cosVal + D * sinVal)) * Mathf.Rad2Deg;
            }

            /// <summary>
            /// Copy polyLimit and scale by r
            /// </summary>
            /// <param name="clone"></param>
            /// <param name="polygonLimit"></param>
            public static bool CloneLimit(JointLimitPolygon clone, PolygonLimit polygonLimit, float r)
            {
                if (clone == null) return false;
                if (clone.limitSolver.originPoints.Length < 4) return false;

                polygonLimit.axis = clone.limitSolver.axis.normalized * r;
                polygonLimit.twistLimit = clone.limitSolver.twistLimit;
                polygonLimit.smoothness = clone.limitSolver.smoothness;
                polygonLimit.twistAngle = clone.limitSolver.twistAngle;

                polygonLimit.originPoints = new Vector3[clone.limitSolver.originPoints.Length];
                for (int i = 0; i < polygonLimit.originPoints.Length; ++i)
                {
                    polygonLimit.originPoints[i] = clone.limitSolver.originPoints[i] * r;
                }
                polygonLimit.BuildReachCones();

                polygonLimit.s.r = r;
                for (int i = 0; i < polygonLimit.boundPoints.Length; ++i)
                {
                    polygonLimit.boundPoints[i] = clone.limitSolver.boundPoints[i] * r;
                }
                return true;
            }

            public static void TranslateJointLimit(PolygonLimit polygon, PolygonLimit newPolygon, Vector3 newPosition, Quaternion newRotation)
            {
                if (polygon.boundPoints == null) return;

                newPolygon.s.c = newPosition;
                newPolygon.s.r = polygon.s.r;
                newPolygon.axis = newRotation * (polygon.axis);
                newPolygon.smoothness = polygon.smoothness;
                newPolygon.twistAngle = polygon.twistAngle;

                newPolygon.boundPoints = new Vector3[polygon.boundPoints.Length];
                for (int i = 0; i < polygon.boundPoints.Length; ++i)
                {
                    //newPolygon.boundPoints[i] = newPosition + newRotation * (polygon.boundPoints[i] - polygon.s.c);
                    newPolygon.boundPoints[i] = newPosition + newRotation * polygon.boundPoints[i];
                }
                newPolygon.ReculculateCones();
            }

            //static bool isDraw=true;
            //static int count = 8;
            public static int GetSwingSwivel(PolygonLimit polygon, Circle swivel, Vector3 zeroSwivel, ref Interval[] intervals)
            {
                if (polygon.boundPoints == null) return 0;

                Plane circle = new Plane(swivel.n, Vector3.Dot(swivel.n, swivel.c));//swivel circle
                List<Interval[]> intervalsList = new List<Interval[]>();
                Vector3 n1, n2;
                //if (count == 8)
                //{
                //    isDraw = true;
                //    count=1;
                //}
                //else
                //{
                //    isDraw = false;
                //    count++;
                //}
                for (int i = 0; i < polygon.cones.Length; ++i)
                {
                    n1 = polygon.cones[i].sliceNormal;
                    if (i == polygon.cones.Length - 1) n2 = -polygon.cones[0].sliceNormal;
                    else n2 = -polygon.cones[i + 1].sliceNormal;
                    Plane[] planes = new Plane[3]
                    {
                        new Plane(n1, Vector3.Dot(n1, polygon.s.c)),
                        new Plane(polygon.cones[i].boundNormal,Vector3.Dot(polygon.cones[i].boundNormal,polygon.s.c)),
                        new Plane(n2, Vector3.Dot(n2, polygon.s.c))
                    };
                    Interval[] tmp = new Interval[0];
                    //if (i != 2) isDraw = false;
                    //else isDraw = true;
                    GeometryTool.IntersectionPolygonCircle(polygon.s, planes, circle, swivel, zeroSwivel, ref tmp);

                    if (tmp.Length > 0)
                    {
                        //if (isDraw)
                        //{
                        //    DrawTool.DrawPolygon(new Polygon(polygon.s, new Vector3[] { polygon.axis + polygon.s.c, polygon.cones[i].verts[2], polygon.cones[i].verts[3] }), Color.yellow);
                        //}
                        intervalsList.Add(tmp);
                    }
                }
                if (intervalsList.Count > 1)
                {
                    List<Interval> tmp = new List<Interval>();
                    for (int i = 0; i < intervalsList.Count; ++i)
                    {
                        for (int j = 0; j < intervalsList[i].Length; ++j)
                        {
                            tmp.Add(intervalsList[i][j]);
                        }
                    }
                    intervals = Interval.Merge(tmp.ToArray());
                }
                return 1;
            }

        }
    }
}