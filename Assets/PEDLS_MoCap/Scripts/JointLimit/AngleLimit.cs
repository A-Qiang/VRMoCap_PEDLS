using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class AngleLimit : JointLimit
        {
            [Range(0,180)]public float swingLimit;
            [Range(0,180)]public float twistLimit;

            private Quaternion LimitSwing(Quaternion rotation)
            {
                Vector3 swingAxis = rotation * axis;
                Quaternion swingLimitedRotation = Quaternion.FromToRotation(axis, swingAxis);
                swingLimitedRotation = Quaternion.RotateTowards(Quaternion.identity, swingLimitedRotation, swingLimit);

                Quaternion toLimit = Quaternion.FromToRotation(swingAxis, swingLimitedRotation * axis);
                return toLimit*rotation;
            }
            
            protected override Quaternion LimitRotation(Quaternion rotation)
            {
                Quaternion swingRot = LimitSwing(rotation);
                return LimitTwist(swingRot, axis,secondAxis, twistLimit);
            }

            
        }
    }
}
