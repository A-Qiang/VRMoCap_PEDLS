using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataProcess;
namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class PEDLS_Solver : FBIKSolver
        {
            public Transform pelvisTarget;
            public Transform headTarget;
            public Transform leftArmTarget;
            public Transform rightArmTarget;
            public Transform leftLegTarget;
            public Transform rightLegTarget;

            public IKType iKType=IKType.PEDLS_IK;
            [Range(0, 1)] public float estimatedWeight=0.8f;
            [Range(0,250)]public int maxIteration=100;

            public HybridUpperSolver upperBody;
            public HybridLegSolver leftLeg;
            public HybridLegSolver rightLeg;

            bool limitJoint=true;
            Transform[] targets;
            HybridLegSolver[] legs;

            #region override

            /// <summary>
            /// Initialize when the program starts
            /// </summary>
            protected override void OnInitiate()
            {
                targets = new Transform[6] { pelvisTarget, headTarget, leftArmTarget, rightArmTarget, leftLegTarget, rightLegTarget };
                upperBody.InitiateLimit(solverTransforms);

                legs = new HybridLegSolver[2] { leftLeg, rightLeg };
                for(int i = 0; i < 2; ++i)
                {
                    legs[i].InitiateLimit(solverTransforms, i * 4 + 14);
                }
            }

            protected override void Read()
            {
                upperBody.Read(readPositions, readRotations, 1, 2);
                leftLeg.Read(readPositions, readRotations, 1, 14);
                rightLeg.Read(readPositions, readRotations, 1, 18);
            }
            
            /// <summary>
            /// analytical solver
            /// </summary>
            protected override void Solve()
            {
                Reset();
                upperBody.SolveAnalyticalIK(limitJoint);
                for (int i = 0; i < 2; i++)
                {
                    legs[i].SolveAnalyticalIK(limitJoint);
                }
            }

            protected override void Write()
            {
                upperBody.Write(ref solvedPositions, ref solvedRotations);
                leftLeg.Write(ref solvedPositions, ref solvedRotations);
                rightLeg.Write(ref solvedPositions, ref solvedRotations);
            }

            /// <summary>
            /// Get bones and initiate solvers
            /// </summary>
            /// <param name="root"></param>
            public new void Initiate(Transform root)
            {
                OnInitiate();
                StoreDefaultPose();
                UpdateTransforms();
                Read();
            }

            protected new void WriteTransforms()
            {
                for (int i = 1; i < solverTransforms.Length; ++i)
                {
                    if (i < 2)
                    {
                        solverTransforms[i].position = solvedPositions[i];
                    }
                    solverTransforms[i].rotation = solvedRotations[i];
                }
            }

            public new void Update()
            {
                switch (iKType)
                {
                    case IKType.DLS_IK:
                        SolveDLS_IK();
                        break;
                    default:
                        SolvePEDLS_IK();
                        break;
                }
            }

            #endregion

            /// <summary>
            /// Initiate bone transforms
            /// </summary>
            /// <param name="boneRoot"></param>
            /// <param name="targetRoot"></param>
            /// 
            public void SetBone(IKSkeleton skeleton)
            {
                solverTransforms = skeleton.GetBoneTransforms();
                readPositions = new Vector3[solverTransforms.Length];
                readRotations = new Quaternion[solverTransforms.Length];
            }

            /// <summary>
            /// Initialize the by pelvis
            /// </summary>
            void Reset()
            {
                upperBody.iterCount = 0;
                upperBody.locked = false;

                upperBody.pelvisTargetPos = targets[0].position;
                upperBody.pelvisTargetRot = targets[0].rotation;

                upperBody.headTargetPos = targets[1].position;
                upperBody.headTargetRot = targets[1].rotation;

                upperBody.leftArmTargetPos = targets[2].position;
                upperBody.leftArmTargetRot = targets[2].rotation;

                upperBody.rightArmTargetPos = targets[3].position;
                upperBody.rightArmTargetRot = targets[3].rotation;

                upperBody.TranslatePelvis(targets[0].position, targets[0].rotation);
                
                for (int i = 0; i < 2; ++i)
                {
                    legs[i].iterCount = 0;
                    legs[i].locked = false;

                    legs[i].targetPos = targets[i + 4].position;
                    legs[i].targetRot = targets[i + 4].rotation;

                    legs[i].TranslateRoot(upperBody.baseBone.solvedPosition, upperBody.baseBone.solvedRotation);
                    
                }
            }
            
            #region IK Solver

            /// <summary>
            /// Jacobian-based solver
            /// </summary>
            private void SolveDLS_IK()
            {
                if (fixTranstorms)
                {
                    UseDefaultPose();
                }
                UpdateTransforms();
                Read();
                Reset();
                Write();
                WriteTransforms();

                for (int j = 0; j < maxIteration; ++j)
                {
                    if (leftLeg.locked && rightLeg.locked && upperBody.locked)
                    {
                        break;
                    }
                   
                    UpdateTransforms();
                    Read();

                    if (!upperBody.locked)
                    {
                        upperBody.CalcError(targets);
                        if (upperBody.positionErr > upperBody.posTolerance || upperBody.rotationErr > upperBody.rotTolerance)
                        {
                            upperBody.Solve();
                            upperBody.iterCount++;
                        }
                        else
                        {
                            upperBody.locked = true;
                        }
                    }
                    
                    for (int i = 0; i < 2; ++i)
                    {
                        if (!legs[i].locked)
                        {
                            legs[i].positionErr = (legs[i].targetPos - legs[i].bones[2].solvedPosition).magnitude;
                            legs[i].rotationErr = Mathf.Abs(Quaternion.Angle(legs[i].targetRot, legs[i].bones[2].solvedRotation));
                            if (legs[i].positionErr > legs[i].positionTolerance || legs[i].rotationErr > legs[i].rotationTolerance)
                            {
                                legs[i].Solve();
                                legs[i].iterCount++;
                            }
                            else
                            {
                                legs[i].locked = true;
                            }
                        }
                    }
                    
                    Write();
                    WriteTransforms();
                }
            }

            /// <summary>
            /// Solve analytically
            /// </summary>
            /// <param name="isWeight"></param>
            private void PoseEstimate(float estimatedWeight)
            {
                if (fixTranstorms)
                {
                    UseDefaultPose();
                }
             
                UpdateTransforms();
                Read();

                Reset();
                upperBody.SolveHeuristicsAnalyticalIK(estimatedWeight,limitJoint);
                for (int i = 0; i < 2; i++)
                {
                    legs[i].SolveHeuristicAnalyticalIK(estimatedWeight,limitJoint);
                }
                
                Write();
                WriteTransforms();
            }

            /// <summary>
            /// PEDLS_IK,obtain reference posture by pose estimation and optimize further by DLS
            /// </summary>
            void SolvePEDLS_IK()
            {
                //Analytical pose estimation
                PoseEstimate(estimatedWeight);

                //Numerical optimize
                for (int j = 0; j < maxIteration; ++j)
                {
                    if (leftLeg.locked && rightLeg.locked && upperBody.locked)
                    {
                        break;
                    }
                    
                    UpdateTransforms();
                    Read();

                    if (!upperBody.locked)
                    {
                        upperBody.CalcError(targets);
                        if (upperBody.positionErr > upperBody.posTolerance || upperBody.rotationErr > upperBody.rotTolerance)
                        {
                            upperBody.Solve();
                            upperBody.iterCount++;
                        }
                        else
                        {
                            upperBody.locked = true;
                        }
                    }
                    
                    for (int i = 0; i < 2; ++i)
                    {
                        if (!legs[i].locked)
                        {
                            float positionErr = (legs[i].targetPos - legs[i].bones[2].solvedPosition).magnitude;
                            float rotationErr = Mathf.Abs(Quaternion.Angle(legs[i].targetRot, legs[i].bones[2].solvedRotation));
                            if (positionErr > legs[i].positionTolerance || rotationErr > legs[i].rotationTolerance)
                            {
                                legs[i].Solve();
                                legs[i].iterCount++;
                            }
                            else
                            {
                                legs[i].locked = true;
                            }
                        }
                    }
                    
                    Write();
                    WriteTransforms();
                }
                
                Write();
                WriteTransforms();
            }

            #endregion

        }

    }
}

