using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class EclipseLimit : JointLimit
        {
            [Range(0, 180)]public  float twistLimit;

            public Axis majorAxis;
            [Range(-180,180)]public float majorMin;
            [Range(-180,180)]public float majorMax;
            public Axis minorAxis;
            [Range(-180,180)]public float minorMin;
            [Range(-180,180)]public float minorMax;
            Quaternion lastRotation = Quaternion.identity;
            Vector2 lastVector = Vector2.zero;

            protected override Quaternion LimitRotation(Quaternion rotation)
            {
                //return rotation;
                lastRotation = LimitSwing(rotation);
                //Debug.Log(rotation.eulerAngles);
                //return lastRotation;
                //return LimitTwist(lastRotation, axis, secondAxis, twistLimit);//secondAxis不能与axis或rotation*axis平行
                return LimitTwist(lastRotation, axis, twistLimit);
            }

            Quaternion LimitSwing(Quaternion rotation)
            {
                Vector3 v = Vector3.Cross(axis, rotation * axis).normalized;
                if (v == Vector3.zero) return rotation;//no swing

                float rad =Mathf.Acos(Vector3.Dot(axis, rotation * axis));//(0,Π)

                float x = Vector3.Dot(rad * v, Utilities.GetAxis(majorAxis));
                float y = Vector3.Dot(rad * v, Utilities.GetAxis(minorAxis));
                //Debug.Log("x="+Mathf.Rad2Deg*x + "   y=" +Mathf.Rad2Deg*y);
               
                if (x > 0 && y > 0) return LimitEclipse(rotation,  x, y, Mathf.Deg2Rad * majorMax, Mathf.Deg2Rad * minorMax);
                if (x > 0 && y < 0) return LimitEclipse(rotation,  x, y, Mathf.Deg2Rad * majorMax, Mathf.Deg2Rad * minorMin);
                if (x < 0 && y > 0) return LimitEclipse(rotation,  x, y, Mathf.Deg2Rad * majorMin, Mathf.Deg2Rad * minorMax);
                if (x < 0 && y < 0) return LimitEclipse(rotation,  x, y, Mathf.Deg2Rad * majorMin, Mathf.Deg2Rad * minorMin);

                float xAngle = Mathf.Rad2Deg * x;
                float yAngle = Mathf.Rad2Deg * y;
                if (x == 0 && yAngle > minorMax) return Quaternion.AngleAxis( yAngle - minorMax, -v) * rotation;
                if (x == 0 && yAngle < minorMin) return Quaternion.AngleAxis(yAngle - minorMin, v) * rotation;
                if (y == 0 && xAngle > majorMax) return Quaternion.AngleAxis( xAngle - majorMax, -v) * rotation;
                if (y == 0 && xAngle < majorMin) return Quaternion.AngleAxis( xAngle - majorMin, v) * rotation;
                lastVector = new Vector2(Mathf.Clamp(x,majorMin,majorMax),Mathf.Clamp( y,minorMin,minorMax));
                return rotation;
            }

            Quaternion LimitEclipse(Quaternion rotation,float x,float y,float a,float b)
            {
                float tmp = (x * x) / (a * a) + (y * y) / (b * b);
                if (tmp <= 1)
                {
                    lastVector = new Vector2(x, y);
                    return rotation;
                }
                //Debug.Log("limit swing");
                float k, m;
                k = (y - lastVector.y) / (x - lastVector.x);
                m = y - k * x;

                float p1, p2, p3;
                p1 = a * a * k * k + b * b;
                p2 = 2 * a * a * k * m;
                p3 = a * a *( m * m -  b * b);

                float delta = p2 * p2 - 4 * p1 * p3;
                if (delta >= 0)
                {
                    float x1 = (-p2 + Mathf.Sqrt(delta)) / (2 * p1);
                    float x2 = (-p2 - Mathf.Sqrt(delta)) / (2 * p1);
                    Vector2 v1 = new Vector2(x1, k * x1 + m);
                    Vector2 v2 = new Vector2(x2, k * x2 + m);
                    if ((v1.x - x) * (v1.x - x) + (v1.y - y) * (v1.y - y) >
                        (v2.x - x) * (v2.x - x) + (v2.y - y) * (v2.y - y)) lastVector = v2;
                    else lastVector = v1;
                    
                    return Quaternion.FromToRotation(GetRotation(new Vector2(x,y)),GetRotation(lastVector))*rotation;
                }

                lastVector = new Vector2(x, y);
                return rotation;
            }

            public Vector3 GetRotation(Vector2 vector2)
            {
                Vector3 v = vector2.x * Utilities.GetAxis(majorAxis) + vector2.y * Utilities.GetAxis(minorAxis);
                float angle = Mathf.Rad2Deg * vector2.magnitude;
                return Quaternion.AngleAxis(angle, v)*axis;
            }
        }

    }
}

