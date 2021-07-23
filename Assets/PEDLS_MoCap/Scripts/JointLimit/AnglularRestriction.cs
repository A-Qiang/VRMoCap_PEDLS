using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public struct AngularRestriction
        {
            public Interval x;
            public Interval y;
            public Interval z;
            public Vector3 rotationAxis;
            public AngularRestriction(Vector3 axis,Interval x_axis,Interval y_axis,Interval z_axis)
            {
                rotationAxis = axis;
                x = x_axis;
                y = y_axis;
                z = z_axis;
            }
        }
    }
}
