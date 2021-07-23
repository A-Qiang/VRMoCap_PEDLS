using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Mocap
{
    namespace IK
    {
        public class JointLimitEclipse : JointLimitManager
        {
            public EclipseLimit limitSolver;
            public override JointLimit GetLimitSolver()
            {
                return limitSolver;
            }
        }

    }
}

