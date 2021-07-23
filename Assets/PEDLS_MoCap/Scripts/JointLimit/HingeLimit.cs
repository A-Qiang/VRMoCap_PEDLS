using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class HingeLimit : JointLimit
        {
            [Range(-180,180)]public float min=0;

            [Range(-180,180)]public float max=180;
            [HideInInspector]public float zeroOffset;
            protected override Quaternion LimitRotation (Quaternion rotation)
            {
                //Debug.Log(rotation.eulerAngles);
                lastRotation = GetHingeLimitedRelZero(rotation);
                return lastRotation;
            }

            private Quaternion lastRotation=Quaternion.identity;
            private float lastAngle=0f;

            private Quaternion GetHingeLimitedRelZero(Quaternion rotation)
            {
                Quaternion free1DOF = Limited1DOF(rotation,axis);

                //angle between last and this
                Quaternion addR = free1DOF * Quaternion.Inverse(lastRotation);
                float addA = Quaternion.Angle(lastRotation, free1DOF);

                Vector3 sAxis = new Vector3(axis.y, axis.z, axis.x);
                Vector3 cross = Vector3.Cross(sAxis, axis);
                if (Vector3.Dot(addR * sAxis, cross) > 0f) addA = -addA;

                lastAngle += addA;
                if (lastAngle>max || lastAngle < min)
                {
                    lastAngle = Mathf.Clamp(lastAngle, min, max);
                }
                //add default local rotation
                return  Quaternion.AngleAxis(lastAngle, axis);
            }
        }

    }
}
