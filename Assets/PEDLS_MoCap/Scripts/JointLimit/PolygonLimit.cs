using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class PolygonLimit : JointLimit
        {
            [Range(0, 180)] public float twistLimit;
            public Sphere s=new Sphere(Vector3.zero,1);
            public Interval twistAngle; 

            public Vector3[] boundPoints;
            public Vector3[] originPoints;
            public ReachCone[] cones;
            [Range(0,3)]public int smoothness;
            
            [System.Serializable]
            public class ReachCone
            {
                public Vector3[] verts;

                public float vol;
                public Vector3 sliceNormal;
                public Vector3 boundNormal;

                public ReachCone(Vector3 o,Vector3 a,Vector3 b,Vector3 c)
                {
                    verts = new Vector3[4]
                    {
                        o,a,b,c
                    };
                    vol = Vector3.Dot(Vector3.Cross(a, b-o), c-o) / 6;

                    sliceNormal = Vector3.Cross(a, b-o).normalized;
                    boundNormal = Vector3.Cross(b-o, c-o).normalized;
                }

                public void Reculculate()
                {
                    Vector3 o, a, b, c;
                    o = verts[0];
                    a = verts[1];
                    b = verts[2];
                    c = verts[3];
                    vol = Vector3.Dot(Vector3.Cross(a,b-o), c-o) / 6.0f;
                    sliceNormal = Vector3.Cross(a, b-o).normalized;
                    boundNormal = Vector3.Cross(b-o, c-o).normalized;
                }
            }

            protected override Quaternion LimitRotation(Quaternion rotation)
            {
                if (cones.Length == 0) OnInitiated();
                Quaternion swingRot = LimitSwing(rotation);
                return LimitTwist(swingRot, axis, secondAxis, twistLimit);
            }

            public void SetDefaultCones()
            {
                originPoints = new Vector3[4];
                Quaternion swingY = Quaternion.AngleAxis(45,Vector3.up );
                Quaternion swingX = Quaternion.AngleAxis(45, Vector3.right);
                originPoints[0] = swingX * swingY *axis;
                originPoints[1] = Quaternion.Inverse(swingX)  *swingY *  axis;
                originPoints[2] = Quaternion.Inverse(swingX) * Quaternion.Inverse(swingY)  * axis;
                originPoints[3] = swingX  *Quaternion.Inverse(swingY) *  axis;
                BuildReachCones();
            }

            private void OnInitiated()
            {
                if (originPoints.Length < 3) SetDefaultCones();
                else BuildReachCones();
            }

            /// <summary>
            /// Rebuild reach cones
            /// </summary>
            public void BuildReachCones()
            {
                smoothness = Mathf.Clamp(smoothness, 0, 3);

                boundPoints = new Vector3[originPoints.Length];
                for (int i = 0; i < originPoints.Length; ++i) boundPoints[i] = originPoints[i].normalized;

                //caculate points
                for(int i = 0; i < smoothness; ++i)
                {
                    boundPoints=SmoothPoints();
                }

                //calculate cones
                cones = new ReachCone[boundPoints.Length];
                for(int i = 0; i < cones.Length-1; ++i)
                {
                    cones[i] = new ReachCone(s.c, axis, boundPoints[i], boundPoints[i + 1]);
                    cones[i].Reculculate();
                }
                cones[cones.Length - 1] = new ReachCone(s.c, axis, boundPoints[boundPoints.Length - 1], boundPoints[0]);
                cones[cones.Length-1].Reculculate();
            }

            public void ReculculateCones()
            {
                cones = new ReachCone[boundPoints.Length];
                for (int i = 0; i < cones.Length - 1; ++i)
                {
                    cones[i] = new ReachCone(s.c, axis, boundPoints[i], boundPoints[i + 1]);
                }
                cones[cones.Length - 1] = new ReachCone(s.c, axis, boundPoints[boundPoints.Length - 1], boundPoints[0]);
            } 

            #region Smooth boudary
            private Vector3[] SmoothPoints()
            {
                Vector3[] smoothBoundPoints = new Vector3[boundPoints.Length * 2];
                float scalar = GetScalar(boundPoints.Length);


                for (int i = 0; i < smoothBoundPoints.Length; i += 2)
                {
                    smoothBoundPoints[i] = MapPointToTangentPlane(boundPoints[i / 2], 1);
                }
                //interplate
                for(int i = 1; i < smoothBoundPoints.Length; i += 2)
                {
                    Vector3 p1, p2, q2;//pi+1,pi+2,pi-2
                    if (i == 1)
                    {
                        p1 = smoothBoundPoints[i + 1];
                        q2 = smoothBoundPoints[smoothBoundPoints.Length - 2];
                        p2 = smoothBoundPoints[i + 1];
                    }else if (i == smoothBoundPoints.Length - 1)
                    {
                        p1 = smoothBoundPoints[0];
                        q2 = smoothBoundPoints[i - 2];
                        p2 = smoothBoundPoints[1];
                    }
                    else
                    {
                        p1 = smoothBoundPoints[i + 1];
                        q2 = smoothBoundPoints[i - 2];
                        p2 = smoothBoundPoints[i + 1];
                    }

                    //new point
                    smoothBoundPoints[i] = 0.5f * (smoothBoundPoints[i - 1] + p1) + scalar * (smoothBoundPoints[i - 1] - p2) + scalar * (p1 - q2);
                }
                for(int i = 0; i < smoothBoundPoints.Length; ++i)
                {
                    smoothBoundPoints[i] = MapTangentPlaneToSphere(smoothBoundPoints[i], 1);
                }
                return smoothBoundPoints;
            }

            private float GetScalar(int k)
            {
                // Values k (number of points) == 3, 4 and 6 are calculated by analytical geometry, values 5 and 7 were estimated by interpolation
                if (k <= 3) return .1667f;
                if (k == 4) return .1036f;
                if (k == 5) return .0850f;
                if (k == 6) return .0773f;
                if (k == 7) return .0700f;
                return .0625f; // Cubic spline fit
            }

            /// <summary>
            /// Map to tangent plane
            /// </summary>
            /// <param name="p"></param>
            /// <param name="r"></param>
            /// <returns></returns>
            private Vector3 MapPointToTangentPlane(Vector3 p,float r)
            {
                float d = Vector3.Dot(p, axis);
                float u = (2 * r * r) / (d + r * r);
                return u * p + (1 - u) * (-axis);
            }

            /// <summary>
            /// Map to sphere
            /// </summary>
            /// <param name="q"></param>
            /// <param name="r"></param>
            /// <returns></returns>
            private Vector3 MapTangentPlaneToSphere(Vector3 q,float r)
            {
                float d = (q - axis).sqrMagnitude;
                float u = (4 * r * r) / (4 * r * r + d);
                return u * q + (1 - u) * (-axis);
            }

            #endregion

            /// <summary>
            /// Get limited swing rotation
            /// </summary>
            /// <param name="rotation"></param>
            /// <returns></returns>
            private Quaternion LimitSwing(Quaternion rotation)
            {
                if (rotation == Quaternion.identity)
                {
                    return rotation;
                }
                Vector3 swingAxis = rotation * axis;

                //Detect cones 
                int index = GetCone(swingAxis);

                if (index == -1)
                {
                    Debug.Log("error");
                    return rotation;
                }

                //Detect boundary
                float v = Vector3.Dot(cones[index].boundNormal, swingAxis);
                if (v >= 0)
                {
                    return rotation;
                }
                //Culculate boundary
                Vector3 rotateNormal = Vector3.Cross(axis, swingAxis);
                
                Vector3 limitedAxis = Vector3.Cross(cones[index].boundNormal,-rotateNormal);
                Quaternion toLimit = Quaternion.FromToRotation(swingAxis, limitedAxis);
                return toLimit*rotation;
            }

            /// <summary>
            /// Get the index of cone which contain swing axis
            /// </summary>
            /// <param name="swingAxis"></param>
            /// <returns></returns>
            private int GetCone(Vector3 swingAxis)
            {
                float p, p1;
                p1 = Vector3.Dot(cones[0].sliceNormal, swingAxis);
                
                for(int i = 0; i < cones.Length; ++i)
                {
                    p = p1;
                    if (i < cones.Length - 1)
                    {
                        p1 = Vector3.Dot(cones[i + 1].sliceNormal, swingAxis);
                    }
                    else
                    {
                        p1 = Vector3.Dot(cones[0].sliceNormal, swingAxis);
                    }
                    if (p >= 0 && p1 < 0) return i;
                }
                return -1;
            }
        }

    }
}