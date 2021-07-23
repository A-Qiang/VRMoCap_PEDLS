using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Mocap
{
    namespace IK
    {
        #region Struct

        public struct Circle
        {
            public Vector3 c;
            public Vector3 n;
            public float r;
            public Circle(Vector3 c, Vector3 n, float r)
            {
                this.c = c;
                this.n = n;
                this.r = r;
            }
        }

        public struct Plane
        {
            public Vector3 n;
            public float d;
            public Plane(Vector3 n, float d)
            {
                this.n = n;
                this.d = d;
            }
        }

        public struct Sphere
        {
            public Vector3 c;
            public float r;
            public Sphere(Vector3 c, float r)
            {
                this.c = c;
                this.r = r;
            }
        }

        public struct Polygon
        {
            public Sphere s;

            //public Vector3 axis;
            public Vector3[] boundPoints;
            public Vector3[] boundNormal;
            public Polygon(Sphere sphere, Vector3[] points)
            {
                int len = points.Length;

                //this.axis = axis;

                s = new Sphere(sphere.c, sphere.r);
                boundPoints = new Vector3[len];
                boundNormal = new Vector3[len];

                Vector3 v1, v2;
                boundPoints[0] = points[0];
                v1 = points[0] - s.c;
                for (int i = 1; i < len; ++i)
                {
                    boundPoints[i] = points[i];

                    v2 = points[i] - s.c;
                    boundNormal[i - 1] = Vector3.Cross(v1, v2).normalized;
                    v1 = v2;
                }
                v2 = points[0] - s.c;
                boundNormal[len - 1] = Vector3.Cross(v1, v2).normalized;
            }
        }

        [System.Serializable]
        public struct Capsule
        {
            public Vector3 a;
            public Vector3 b;
            public float r;
            public Capsule(Vector3 a,Vector3 b,float r)
            {
                this.a = a;
                this.b = b;
                this.r = r;
            }
        }
        
        #endregion
    }
}