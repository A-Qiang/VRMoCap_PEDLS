using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Mocap
{
    namespace IK
    {
        public abstract class JointLimit 
        {
            public Vector3 axis=Vector3.forward;

            [HideInInspector]public Vector3 secondAxis { get { return new Vector3(axis.y, axis.z, axis.x); } }

            [HideInInspector]public Vector3 cross { get { return Vector3.Cross(axis, secondAxis); } }

            [HideInInspector]public Quaternion defaultLocalRotation;
            
            public void Initiate(Transform transform)
            {
                StoreDefaultLocalRotation(transform);
            }

            private void StoreDefaultLocalRotation(Transform transform)
            {
                defaultLocalRotation = transform.localRotation;
            }

            protected abstract Quaternion LimitRotation(Quaternion rotation);

            /// <summary>
            /// Get the limited local rotation
            /// </summary>
            /// <param name="localRotation"></param>
            /// <returns></returns>
            public  Quaternion GetLimitedLocalRotation(Quaternion localRotation)
            {
                Quaternion rotation = Quaternion.Inverse(defaultLocalRotation) * localRotation;
                Quaternion limitedRotation=LimitRotation(rotation);
                return defaultLocalRotation*limitedRotation;
            }

            public Quaternion Limited1DOF(Quaternion rotation,Vector3 axis)
            {
                return Quaternion.FromToRotation(rotation * axis, axis) * rotation;
            }

            protected Quaternion LimitTwist(Quaternion rotation, Vector3 axis, Vector3 orthAxis, float twistLimit)
            {
                if (twistLimit >= 180) return rotation;
                Vector3 normal = rotation * axis;
                
                Vector3 orthTangent = orthAxis;
                Vector3.OrthoNormalize(ref normal, ref orthTangent);

                Vector3 rotatedOrthTangent = rotation * orthAxis;
                Vector3.OrthoNormalize(ref normal, ref rotatedOrthTangent);

                Quaternion zeroTwistRotation = Quaternion.FromToRotation(rotatedOrthTangent, orthTangent) * rotation;
                if (twistLimit <= 0) return zeroTwistRotation;
                //Quaternion zero = Quaternion.FromToRotation(axis, rotation * axis);

                return Quaternion.RotateTowards(zeroTwistRotation, rotation, twistLimit);
            }
            protected Quaternion LimitTwist(Quaternion rotation, Vector3 axis, float twistLimit)
            {
                if (twistLimit >= 180) return rotation;
                if (twistLimit <= 0) return Quaternion.FromToRotation(axis, rotation * axis);

                Quaternion zeroTwistRotation = Quaternion.FromToRotation(axis, rotation * axis);
                float angle = Quaternion.Angle(zeroTwistRotation, rotation);
                if (angle <= twistLimit) return rotation;
                
                //Debug.Log("limit twist");
                Quaternion deltaRot = rotation * Quaternion.Inverse(zeroTwistRotation);
                deltaRot.ToAngleAxis(out angle, out Vector3 rotationAxis);
                return Quaternion.AngleAxis(twistLimit, rotationAxis) * zeroTwistRotation;
            }
        }

    }
}
