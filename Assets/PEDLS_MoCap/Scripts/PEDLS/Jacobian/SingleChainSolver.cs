using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Mocap.DataProcess;

namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class SingleChainSolver : IKSolver
        {
            [Range(0.001f, 0.01f)] public float positionTolerance=0.01f;
            [HideInInspector][Range(0.1f, 1.0f)] public float rotationTolerance=0.1f;
            public JacobianSolver jacobianSolver;
            public JacobianJointLimit jacobianLimit;

            public Vector3 kneeRotationAxis=new Vector3(1,0,0);
            protected Vector3 hipRotationAxis=Vector3.one;
            protected Vector3[] rotationAxis;

            protected int[] chain = new int[3] { 0, 1, 2};


            [HideInInspector] public bool locked = false;

            [HideInInspector] public Vector3 targetPos;
            [HideInInspector] public Quaternion targetRot;

            /// <summary>
            /// Read pose and initiate
            /// </summary>
            /// <param name="positions"></param>
            /// <param name="rotations"></param>
            /// <param name="rootIndex"></param>
            /// <param name="index"></param>
            protected override void OnRead(Vector3[] positions, Quaternion[] rotations, int rootIndex, int index)
            {
                if (!Initiated)
                {
                    //joint limit
                    rotationAxis = new Vector3[2] { hipRotationAxis, kneeRotationAxis };

                    bones = new IKBone[4];
                    for(int i = 0; i < bones.Length; ++i)
                    {
                        if (i < rotationAxis.Length ) bones[i] = new IKBone(positions[index + i], rotations[index + i], rotationAxis[i]);
                        else bones[i] = new IKBone(positions[index + i], rotations[index + i]);
                    }
                    
                    Initiated = !Initiated;
                }
                for(int i = 0; i < bones.Length; ++i)
                {
                    bones[i].solvedPosition = positions[i + index];
                    bones[i].solvedRotation = rotations[i + index];
                }
            }
            
            public void Solve()
            {
                if ((targetPos - bones[2].solvedPosition).magnitude > positionTolerance)
                {
                    SolveJacobianIK();
                }
                if (Mathf.Abs(Quaternion.Angle(bones[2].solvedRotation, targetRot)) > rotationTolerance)
                {
                    IKBone.RotateAroundPoint(bones, 2, bones[2].solvedPosition, targetRot * Quaternion.Inverse(bones[2].solvedRotation));
                }
            }

            /// <summary>
            /// Solve Chain and update bones
            /// </summary>
            /// <param name="targetPos"></param>
            /// <param name="targetRot"></param>
            protected void SolveJacobianIK()
            {
                //error
                Vector3 errorVector = targetPos - bones[2].solvedPosition;
                double[] errorArray = new double[3] { errorVector.x, errorVector.y, errorVector.z };
                
                //Jacobian
                Matrix<double> mat = JacobianSolver.CalcJacobian(bones, chain, new int[] { 3, 4 });

                //update angle
                Vector<double> rot = jacobianSolver.SolveJacobian(mat, errorArray);
                float[] rotations = new float[4];
                for (int j = 0; j < rot.Count; ++j)
                {
                    rotations[j] = Mathf.Rad2Deg* (float)rot[j];
                }
                
                //limited
                Quaternion[] localRotations=jacobianLimit.GetLimitedRotation(bones, rotations,rootRotation);
                
                IKBone.RotateAroundPoint(bones, 0, bones[0].solvedPosition, rootRotation * localRotations[0] * Quaternion.Inverse(bones[0].solvedRotation));
                IKBone.RotateAroundPoint(bones, 1, bones[1].solvedPosition, bones[0].solvedRotation * localRotations[1] * Quaternion.Inverse(bones[1].solvedRotation));
            }
        }
    }
}