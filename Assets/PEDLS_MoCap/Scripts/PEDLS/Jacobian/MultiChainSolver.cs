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
        public class MultiChainSolver : IKSolver
        {
            [Range(0.001f,0.01f)]public float posTolerance=0.01f;
            [HideInInspector][Range(0.1f,1)]public float rotTolerance=0.1f;
            public JacobianSolver jacobianSolver;
            public JacobianJointLimit spineJacobianLimit;
            public JacobianJointLimit lArmJacobianLimit;
            public JacobianJointLimit rArmJacobianLimit;

            public Vector3 leftClavicleRotationAxis=new Vector3(0,1,1);
            public Vector3 leftElbowRotationAxis=new Vector3(0,1,0);
            public Vector3 rightClavicleRotationAxis=new Vector3(0,1,1);
            public Vector3 rightElbowRotationAxis=new Vector3(0,1,0);

            public IKBone baseBone;
            //public bool usAngularLimit;

            [HideInInspector]public bool locked = false;
            
            [HideInInspector]public float positionErr;
            [HideInInspector]public float rotationErr;
            [HideInInspector]public int iterCount;
            [HideInInspector] public Vector3 headTargetPos;
            [HideInInspector] public Quaternion headTargetRot;

            [HideInInspector] public Vector3 leftArmTargetPos;
            [HideInInspector] public Quaternion leftArmTargetRot;

            [HideInInspector] public Vector3 rightArmTargetPos;
            [HideInInspector] public Quaternion rightArmTargetRot;

            [HideInInspector] public Vector3 pelvisTargetPos;
            [HideInInspector] public Quaternion pelvisTargetRot;

            protected Vector3 spineRotationAxis=Vector3.one;
            protected Vector3 chestRotationAxis=Vector3.one;
            protected Vector3 neckRotationAxis =Vector3.one;
            protected Vector3 leftShoulderRotationAxis=Vector3.one;
            protected Vector3 rightShoulderRotationAxis=Vector3.one;
            protected Vector3[] rotationAxis;

            protected IKBone[] spine;
            protected IKBone[] lArm;
            protected IKBone[] rArm;
            int[] effectors = new int[3] { 3, 7, 11 };

            protected List<int[]> chainList = new List<int[]>
            {
                new int[4]{0,1,2,3},
                new int[6]{0,1,4,5,6,7},
                new int[6]{0,1,8,9,10,11}
            };
            protected int[] jointList = new int[9] { 0, 1, 2, 4, 5, 6, 8, 9, 10 };
            protected double[] errorVector = new double[9];

            protected override void OnRead(Vector3[] positions, Quaternion[] rotations, int rootIndex, int index)
            {
                if (!Initiated)
                {
                    jacobianSolver = new JacobianSolver();
                    rotationAxis = new Vector3[9] { spineRotationAxis, chestRotationAxis, neckRotationAxis, leftClavicleRotationAxis,
                        leftShoulderRotationAxis, leftElbowRotationAxis, rightClavicleRotationAxis, rightShoulderRotationAxis, rightElbowRotationAxis };

                    //bones
                    baseBone = new IKBone(positions[rootIndex], rotations[rootIndex]);
                    bones = new IKBone[12];
                    for (int i = 0; i < bones.Length; ++i)
                    {
                        bones[i] = new IKBone(positions[i + index], rotations[i + index]);
                    }
                    for (int i = 0; i < jointList.Length; ++i)
                    {
                        bones[jointList[i]].rotaionAxis = rotationAxis[i];
                    }

                    spine = new IKBone[4] { bones[0], bones[1], bones[2], bones[3] };
                    lArm = new IKBone[4] { bones[4], bones[5], bones[6], bones[7] };
                    rArm = new IKBone[4] { bones[8], bones[9], bones[10], bones[11] };
                    

                    Initiated = !Initiated;
                }

                baseBone.solvedPosition = positions[rootIndex];
                baseBone.solvedRotation = rotations[rootIndex];
                for(int i = 0; i < bones.Length; ++i)
                {
                    bones[i].solvedPosition = positions[i + index];
                    bones[i].solvedRotation = rotations[i + index];
                }
            }
            
            public void Solve()
            {
                //position
                if (positionErr > posTolerance)
                {
                    SolveJacobianIK(errorVector);
                }

                //rotation
                if (rotationErr > rotTolerance)
                {
                    bones[effectors[0]].solvedRotation = headTargetRot;
                    bones[effectors[1]].solvedRotation = leftArmTargetRot;
                    bones[effectors[2]].solvedRotation = rightArmTargetRot;
                }
            }

            void SolveJacobianIK(double[] errorVector)
            {
                //Jacobian matrix
                Matrix<double> mat = JacobianSolver.CalcJacobian(bones, chainList,  9,21);
                
                //update angle
                Vector<double> rotationVector = jacobianSolver.SolveJacobian(mat, errorVector);
                float[] rotations = new float[21];
                for(int i = 0; i < 21; ++i)
                {
                    rotations[i] =Mathf.Rad2Deg*(float) rotationVector[i];
                }

                //joint limit                  
                float[] spineRotations = ReadRotations(rotations, 0, 9);
                float[] leftArmRotations = ReadRotations(rotations, 9, 6);
                float[] rightArmRotations = ReadRotations(rotations, 15, 6);

                Quaternion[] spineLocalRotations = spineJacobianLimit.GetLimitedRotation(spine, spineRotations, rootRotation);
                Quaternion[] lArmLocalRotations = lArmJacobianLimit.GetLimitedRotation(lArm, leftArmRotations, spine[1].solvedRotation);
                Quaternion[] rArmLocalRotations = rArmJacobianLimit.GetLimitedRotation(rArm, rightArmRotations, spine[1].solvedRotation);

                IKBone.RotateAroundPoint(bones, 0, bones[0].solvedPosition, rootRotation * spineLocalRotations[0] * Quaternion.Inverse(bones[0].solvedRotation));
                IKBone.RotateAroundPoint(bones, 1, bones[1].solvedPosition, bones[0].solvedRotation * spineLocalRotations[1] * Quaternion.Inverse(bones[1].solvedRotation));
                RotateBones(bones, 2, 3, bones[1].solvedRotation * spineLocalRotations[2] * Quaternion.Inverse(bones[2].solvedRotation));
                RotateBones(bones, 4, 7, bones[1].solvedRotation * lArmLocalRotations[0] * Quaternion.Inverse(bones[4].solvedRotation));
                RotateBones(bones, 8, 11, bones[1].solvedRotation * rArmLocalRotations[0] * Quaternion.Inverse(bones[8].solvedRotation));
                for (int i = 1; i < 3; ++i)
                {
                    RotateBones(bones, i + 4, 7, bones[i + 3].solvedRotation * lArmLocalRotations[i] * Quaternion.Inverse(bones[i + 4].solvedRotation));
                    RotateBones(bones, i + 8, 11, bones[i + 7].solvedRotation * rArmLocalRotations[i] * Quaternion.Inverse(bones[i + 8].solvedRotation));
                }

            }

            public void CalcError(Transform[] targets)
            {
                positionErr = 0;
                rotationErr = 0;
                for (int i = 0; i < 3; ++i)
                {
                    Vector3 err = targets[i + 1].position - bones[effectors[i]].solvedPosition;
                    errorVector[i * 3] = err.x;
                    errorVector[i * 3 + 1] = err.y;
                    errorVector[i * 3 + 2] = err.z;

                    positionErr += err.magnitude;
                    rotationErr += Mathf.Abs(Quaternion.Angle(targets[i + 1].rotation, bones[effectors[i]].solvedRotation));
                }
            }
            
            protected void RotateBones(IKBone[] bones,int start,int end,Quaternion q)
            {
                for(int i = start; i <= end; ++i)
                {
                    bones[i].solvedRotation = q * bones[i].solvedRotation;
                    if(i>start) bones[i].solvedPosition = bones[start].solvedPosition + q * (bones[i].solvedPosition - bones[start].solvedPosition);
                }
            }

            protected float[] ReadRotations(float[] rotations, int beginIndex, int num)
            {
                float[] res = new float[num];
                for (int i = 0; i < num; ++i)
                {
                    res[i] = rotations[i + beginIndex];
                }
                return res;
            }
            
            public new void Write(ref Vector3[] positions, ref Quaternion[] rotations)
            {
                positions[1] = baseBone.solvedPosition;
                rotations[1] = baseBone.solvedRotation;
                for (int i = 0; i < bones.Length; ++i)
                {
                    positions[i + index] = bones[i].solvedPosition;
                    rotations[i + index] = bones[i].solvedRotation;
                }
            }

            /// <summary>
            /// Move pelvis to satisfy pelvis target
            /// </summary>
            /// <param name="newPelvisPosition"></param>
            /// <param name="newPelvisRotation"></param>
            public void TranslatePelvis(Vector3 newPelvisPosition, Quaternion newPelvisRotation)
            {
                //position
                Vector3 delta = newPelvisPosition - baseBone.solvedPosition;
                baseBone.solvedPosition += delta;
                MoveTo(bones[0].solvedPosition+delta);

                //rotation
                Quaternion q = newPelvisRotation * Quaternion.Inverse(baseBone.solvedRotation);
                baseBone.solvedRotation = q * baseBone.solvedRotation;
                IKBone.RotateAroundPoint(bones, 0, baseBone.solvedPosition, q);
            }

        }
    }
}
