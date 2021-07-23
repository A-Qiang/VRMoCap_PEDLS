using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        public abstract class JointLimitManager : MonoBehaviour
        {
            public abstract JointLimit GetLimitSolver();

            [HideInInspector]public bool initiated=false;

            private void Awake()
            {
                GetLimitSolver().Initiate(transform);
            }

            private void LateUpdate()
            {
                transform.localRotation = GetLimitSolver().GetLimitedLocalRotation(transform.localRotation);

            }
        }

    }
}
