using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataProcess;

namespace Mocap
{
    namespace IK
    {
        public abstract class IKSolverManager : MonoBehaviour
        {
            protected abstract void InitiateSolver();

            protected abstract void UpdateSolver();
            
            void Start()
            {
                InitiateSolver();
            }

            private void LateUpdate()
            {
                UpdateSolver();
            }
            
        }
    }
}
