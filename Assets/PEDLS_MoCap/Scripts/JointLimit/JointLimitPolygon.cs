using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        public class JointLimitPolygon : JointLimitManager
        {
            public PolygonLimit limitSolver;

            public override JointLimit GetLimitSolver()
            {
                return limitSolver;
            }

            private void OnGUI()
            {
                //Debug.Log("limited swing="+limitSolver.swingIsLimited);
                //    for (int i = 1; i < limitSolver.boundPoints.Length; ++i)
                //    {
                //        Debug.DrawRay(transform.position, limitSolver.boundPoints[i], Color.green);
                //    }

                //    GUI.Label(new Rect(10,10,100,100), "sfa");
            }

            private void OnDrawGizmos()
            {
                //Gizmos.DrawRay(transform.position, limitSolver.axis);
            }
        }

    }
}
