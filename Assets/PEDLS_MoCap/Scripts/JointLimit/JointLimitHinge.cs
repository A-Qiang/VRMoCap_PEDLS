using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Mocap
{
    namespace IK
    {
        public class JointLimitHinge : JointLimitManager
        {
            public HingeLimit limitSolver;
            public override JointLimit GetLimitSolver()
            {
                return limitSolver;
            }
            
        }

    }
}
