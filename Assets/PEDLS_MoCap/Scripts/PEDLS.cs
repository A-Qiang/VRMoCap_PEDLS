using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Mocap
{
    namespace IK
    {
        public class PEDLS : IKSolverManager
        {
            public IKSkeleton skeleton;
            public PEDLS_Solver solver;

            protected override void InitiateSolver()
            {
                solver.SetBone(skeleton);
                solver.Initiate(transform);
            }

            protected override void UpdateSolver()
            {
                solver.Update();
            }

        }

    }
}

