using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class HybridLegSolver : SingleChainSolver
        {
            #region Pose estimation

            public AnalyticalIK analyticalIK;
            protected JointLimitPolygon hipJoint;
            protected JointLimitPolygon ankleJoint;
            PolygonLimit hipPolygonLimit;
            PolygonLimit anklePolygonLimit;
            Quaternion hipDefaultRelRootRotation;
            Quaternion ankleDefaultRelRootRotation;

            Vector3 defaultBnedNormal=Vector3.left;
            Vector3 bendNormalRelRoot;
            
            #endregion

            [HideInInspector]public int iterCount;
            [HideInInspector]public float positionErr;
            [HideInInspector]public float rotationErr;

            Quaternion toeLocalRotation;
            Vector3 lastBendNormal;

            float[] desiredThetas = new float[4];
            Quaternion[] disiredRot = new Quaternion[2];
            Quaternion defaultLocalHipRot;
            Quaternion defaultLocalKneeRot;

            protected override void OnRead(Vector3[] positions, Quaternion[] rotations, int rootIndex, int index)
            {
                if (!Initiated)
                {
                    #region Jacobian 
                    rotationAxis = new Vector3[2] { hipRotationAxis, kneeRotationAxis };
                    
                    bones = new IKBone[4];
                    for (int i = 0; i < bones.Length; ++i)
                    {
                        if (i < rotationAxis.Length) bones[i] = new IKBone(positions[index + i], rotations[index + i], rotationAxis[i]);
                        else bones[i] = new IKBone(positions[index + i], rotations[index + i]);
                    }
                    defaultLocalHipRot = Quaternion.Inverse(rootRotation) * bones[0].solvedRotation;
                    defaultLocalKneeRot = Quaternion.Inverse(bones[0].solvedRotation) * bones[1].solvedRotation;

                    #endregion

                    #region Pose estimation
                    
                    hipPolygonLimit = new PolygonLimit();
                    JointLimitTool.CloneLimit(hipJoint, hipPolygonLimit, (bones[1].solvedPosition - bones[0].solvedPosition).magnitude);
                    hipDefaultRelRootRotation = Quaternion.Inverse(rootRotation) * bones[0].solvedRotation;

                    anklePolygonLimit = new PolygonLimit();
                    JointLimitTool.CloneLimit(ankleJoint, anklePolygonLimit, (bones[2].solvedPosition - bones[1].solvedPosition).magnitude);
                    ankleDefaultRelRootRotation = Quaternion.Inverse(bones[1].solvedRotation) * bones[2].solvedRotation;
                    
                    //defaultBnedNormal = GetBendNormal();
                    bendNormalRelRoot = Quaternion.Inverse(rootRotation) * defaultBnedNormal;
                    toeLocalRotation = Quaternion.Inverse(bones[2].solvedRotation) * bones[3].solvedRotation;

                    #endregion
                    
                    Initiated = !Initiated;
                }

                for (int i = 0; i < bones.Length; ++i)
                {
                    bones[i].solvedPosition = positions[i + index];
                    bones[i].solvedRotation = rotations[i + index];
                }
            }

            #region Analytical solver

            /// <summary>
            /// Estimate leg
            /// </summary>
            /// <param name="isWeight"></param>
            public void SolveHeuristicAnalyticalIK(float estimatedWeight,bool limitJoint)
            {
                if (estimatedWeight==1)
                {
                    SolveAnalyticalIK(limitJoint);
                }
                else
                {
                    Quaternion[] initRot = new Quaternion[4];
                    Vector3[] initPos = new Vector3[4];
                    for(int i = 0; i < 4; ++i)
                    {
                        initPos[i] = bones[i].solvedPosition;
                        initRot[i] = bones[i].solvedRotation;
                    }
                    SolveAnalyticalIK(limitJoint);
                    Quaternion[] desiredRot = new Quaternion[2]
                    {
                        bones[0].solvedRotation,
                        bones[1].solvedRotation
                    };
                    for(int i = 0; i < 2; ++i)
                    {
                        bones[i].solvedPosition = initPos[i];
                        bones[i].solvedRotation = initRot[i];
                    }

                    for(int i = 0; i < 2; ++i)
                    {
                        Quaternion q = Quaternion.Slerp(Quaternion.identity, desiredRot[i] * Quaternion.Inverse(bones[i].solvedRotation), estimatedWeight);
                        IKBone.RotateAroundPoint(bones, i, bones[i].solvedPosition, q);
                    }
                }
            }
            
            /// <summary>
            /// Solve 7DOF limb
            /// </summary>
            /// <param name="limitJoint"></param>
            public void SolveAnalyticalIK(bool limitJoint)
            {
                analyticalIK.firstPolygon = new PolygonLimit();
                JointLimitTool.TranslateJointLimit(hipPolygonLimit, analyticalIK.firstPolygon, bones[0].solvedPosition, rootRotation * hipDefaultRelRootRotation);

                analyticalIK.thirdPolygon = new PolygonLimit();
                JointLimitTool.TranslateJointLimit(anklePolygonLimit, analyticalIK.thirdPolygon, bones[2].solvedPosition, bones[1].solvedRotation * ankleDefaultRelRootRotation);

                Vector3 bendNormal = rootRotation * bendNormalRelRoot;
                
                analyticalIK.limitJoint = limitJoint;
                analyticalIK.SolveLimited7DOF(bones, 0, 1, 2, targetPos, targetRot,bendNormal);
            }

            /// <summary>
            /// Trigometric
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="first"></param>
            /// <param name="second"></param>
            /// <param name="third"></param>
            /// <param name="bendNormal"></param>
            /// <param name="targetPos"></param>
            /// <param name="weight"></param>
            /// 
            private Vector3 SolveTrigometric(IKBone[] bones, int first, int second, int third, Vector3 targetPos, Vector3 bendNormal)
            {
                Vector3 dir = targetPos - bones[first].solvedPosition;

                float sqrtMag1 = (bones[second].solvedPosition - bones[first].solvedPosition).sqrMagnitude;
                float sqrtMag2 = (bones[third].solvedPosition - bones[second].solvedPosition).sqrMagnitude;

                Vector3 bendDir = Vector3.Cross(dir, bendNormal);
                Vector3 midPos = GetMidDir(dir, bendDir, sqrtMag1, sqrtMag2) + bones[0].solvedPosition;
                return midPos;
            }

            //private Vector3 GussBendNormal()
            //{
            //    Vector3 bendNormalGuss = Vector3.Cross((targetPos - bones[0].solvedPosition).normalized, (targetRot * toeLocalRotation * Vector3.right).normalized);

            //    if (bendNormalGuss.magnitude < 0.17)
            //    {
            //        bendNormalGuss = lastBendNormal;
            //    }

            //    if (Vector3.Dot(lastBendNormal, bendNormalGuss) < 0)
            //    {
            //        bendNormalGuss = -bendNormalGuss;
            //    }
            //    lastBendNormal = bendNormalGuss;
            //    bendNormalGuss = rootRotation * bendNormalRelRoot;

            //    return bendNormalGuss.normalized;
            //}

            /// <summary>
            /// Initiate default bend normal
            /// </summary>
            /// <returns></returns>
            private Vector3 GetBendNormal()
            {
                Vector3 normal = Vector3.Cross(bones[2].solvedPosition - bones[0].solvedPosition, bones[1].solvedPosition - bones[0].solvedPosition);
                if (normal.magnitude <Mathf.Epsilon)
                {
                    normal = defaultBnedNormal;
                }
                else
                {
                    normal.Normalize();
                }
                //normal = defaultBnedNormal;
                return normal;
            }

            private Vector3 GetMidDir(Vector3 dir, Vector3 benDir, float a, float b)
            {
                float x = (a + dir.sqrMagnitude - b) / dir.magnitude / 2;
                float y = Mathf.Sqrt(Mathf.Clamp(a - x * x, 0, Mathf.Infinity));
                return Quaternion.LookRotation(dir, benDir) * new Vector3(0, y, x);
            }
       
            /// <summary>
            /// Get joint limit range
            /// </summary>
            /// <param name="solverTransforms"></param>
            /// <param name="index"></param>
            public void InitiateLimit(Transform[] solverTransforms, int index)
            {
                if (index == 14)
                {
                    Transform[] legJoints = new Transform[2] {
                    solverTransforms[(int)BodyIndex.LeftUpperLeg], solverTransforms[(int)BodyIndex.LeftLowerLeg] };
                    jacobianLimit=new JacobianJointLimit(legJoints );
                }
                else
                {
                    Transform[] legJoints = new Transform[2] {
                    solverTransforms[(int)BodyIndex.RightUpperLeg], solverTransforms[(int)BodyIndex.RightLowerLeg] };
                    jacobianLimit=new JacobianJointLimit( legJoints);
                }
                if (solverTransforms[index].GetComponent<JointLimitPolygon>() != null)
                {
                    hipJoint = solverTransforms[index].GetComponent<JointLimitPolygon>();
                }

                if (solverTransforms[index + 2].GetComponent<JointLimitPolygon>() != null)
                {
                    ankleJoint = solverTransforms[index + 2].GetComponent<JointLimitPolygon>();
                }
            }
               
            #endregion

            #region PIK

            //public void SolvePIK()
            //{
            //    Vector3 errorVector = targetPos - bones[2].solvedPosition;

            //    double[] errorArray = new double[3] { errorVector.x, errorVector.y, errorVector.z };
            
            //    Matrix<double> mat = JacobianSolver.CalcJacobian(bones, chain, new int[] { 3, 4 });
            
            //    Matrix<double> matPI = mat.PseudoInverse();
            
            //    Matrix<double> matNullSpace = DenseMatrix.CreateIdentity(mat.ColumnCount) - matPI * mat;
            
            //    Vector<double> rot = jacobianSolver.SolveJacobian(mat, errorArray);

            //    float[] rotations = new float[4];
            //    for (int j = 0; j < rot.Count; ++j)
            //    {
            //        rotations[j] = Mathf.Rad2Deg * (float)rot[j];
            //    }

            //    float[] curTheta = CalcCurrentDeg();
            
            //    double[] desireV = new double[4];
            //    for (int i = 0; i < 4; ++i)
            //    {
            //        desireV[i] = desiredThetas[i]  -(rotations[i]+ curTheta[i]);
            //    }
            //    Vector<double> z = new DenseVector(desireV);
            //    Vector<double> adRot = matNullSpace * z;

            //    for (int j = 0; j < rot.Count; ++j)
            //    {
            //        rotations[j] = (Mathf.Rad2Deg * (float)rot[j] + (float)adRot[j]);
            //    }
            //    Quaternion[] localRotations = jacobianLimit.GetLimitedRotation(bones, rotations, rootRotation);

            //    IKBone.RotateAroundPoint(bones, 0, bones[0].solvedPosition, rootRotation * localRotations[0] * Quaternion.Inverse(bones[0].solvedRotation));
            //    IKBone.RotateAroundPoint(bones, 1, bones[1].solvedPosition, bones[0].solvedRotation * localRotations[1] * Quaternion.Inverse(bones[1].solvedRotation));
            //}
  
            //public float[] CalcCurrentDeg()
            //{
            //    float[] thetas = new float[4];
            //    Quaternion localRot = Quaternion.Inverse(defaultLocalHipRot) * (Quaternion.Inverse(rootRotation) * bones[0].solvedRotation);
            //    Vector3 angle = localRot.eulerAngles;
            //    thetas[0] = angle.x;
            //    thetas[1] = angle.y;
            //    thetas[2] = angle.z;

            //    localRot = Quaternion.Inverse(defaultLocalKneeRot) * (Quaternion.Inverse(bones[0].solvedRotation) * bones[1].solvedRotation);
            //    angle = localRot.eulerAngles;
            //    //thetas[3] = angle.y;
            //    thetas[3] = GetAngle(angle, kneeRotationAxis);

            //    for (int i = 0; i < 4; ++i)
            //    {
            //        if (thetas[i] > 180) thetas[i] -= 360;
            //    }
            //    return thetas;
            //}

            //Vector3 GetAngle(float rotation, Vector3 rotationAxis)
            //{
            //    if (rotationAxis.x == 1) return new Vector3(rotation, 0, 0);
            //    if (rotationAxis.y == 1) return new Vector3(0, rotation, 0);
            //    if (rotationAxis.z == 1) return new Vector3(0, 0, rotation);
            //    return Vector3.zero;
            //}

            //float GetAngle(Vector3 rotation, Vector3 rotationAxis)
            //{
            //    if (rotationAxis.x == 1) return rotation.x;
            //    if (rotationAxis.y == 1) return rotation.y;
            //    if (rotationAxis.z == 1) return rotation.z;
            //    return 0;
            //}

            #endregion

        }
    }
}
