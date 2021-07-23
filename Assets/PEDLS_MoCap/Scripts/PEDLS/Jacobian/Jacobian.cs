using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System;

/*Jacobian IK solver
 * 
 * */
namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class JacobianSolver
        {
            [Range(0.001f,0.1f)] public float step = 0.01f;//0.01cm
            [Range(0, 1)] public float lambda = 0.1f;
            IKMode iKMode = IKMode.DLS;
            [HideInInspector]public double[] z;
            public void AdjustStep(float vectorMag,int maxIteration)
            {
                //step = vectorMag / maxIteration;
            }
            public Vector<double>SolveJPWithNullspace(Matrix<double>mat,double[] error, Matrix<double> subMat, double[] z)
            {
                Matrix<double> pInverse = mat.PseudoInverse();
                Matrix<double> nullMat = DenseMatrix.CreateIdentity(mat.ColumnCount) - pInverse * mat;
                Matrix<double> subInverse = (nullMat * subMat).PseudoInverse();
                Vector errorV = new DenseVector(error);
                Vector optV = new DenseVector(z);
                return pInverse * errorV + subInverse * optV;
            }

            public Vector<double> SolveJacobian(Matrix<double> mat, double[] error)
            {
                double mag,sqrtMag=0;
                for(int i = 0; i < error.Length; ++i)sqrtMag += error[i] * error[i];
                mag = Math.Sqrt(sqrtMag);
                if (mag > step)
                {
                    for (int i = 0; i < error.Length; ++i) error[i] *= step/mag;
                }
           
                switch (iKMode)
                {

                    case IKMode.JPI:
                        return SolveJP( mat, error);
                    case IKMode.JT:
                        return SolveJT( mat, error);
                    case IKMode.DLS:
                        return SolveDLS( mat, error,lambda);
                    default:
                        return SolveJP( mat, error);
                }
            }

            #region Calculate Jacobian matrix

            public static Matrix<double>CalcElbowJacobian(IKBone[] bones,List<int[]>chainList,int row,int col)
            {
                double[,] jacobianMat = new double[row, col];
                //leftArm
                int deg = 0, joint = 0;
                while (joint < 6)
                {
                    if (joint == 2)
                    {
                        deg += 3;
                        joint += 2;
                    }
                    WriteJacobian(bones, 6, joint, 0, ref deg, ref jacobianMat);
                    joint++;
                }
                deg = 0; joint = 0;
                while (joint < 10)
                {
                    if (joint == 2)
                    {
                        deg += 9;
                        joint += 6;
                    }
                    WriteJacobian(bones, 10, joint, 3, ref deg, ref jacobianMat);
                    joint++;
                }
                return DenseMatrix.OfArray(jacobianMat);
            }

            public static Matrix<double> ComputeMultiChain(IKBone[] bones, List<int[]> chainList,List<int[]>degList)
            {
                int row = 3 * chainList.Count;
                int col = 3 * (bones.Length - chainList.Count);
                double[,] array = new double[row, col];

                for(int i = 0; i < chainList.Count; ++i)
                {
                    for (int j = 0; j < degList[i].Length; ++j)
                    {
                        Vector3 rad = bones[chainList[i][chainList[i].Length-1]].solvedPosition - bones[chainList[i][j]].solvedPosition;

                        Vector3 partialDif = Vector3.Cross(Vector3.forward, rad);//z
                        array[i * 3, degList[i][j] * 3] = partialDif.x;
                        array[i*3+1, degList[i][j] * 3] = partialDif.y;
                        array[i*3+2, degList[i][j] * 3] = partialDif.z;

                        partialDif = Vector3.Cross(Vector3.right, rad);//x
                        array[i*3, degList[i][j] * 3 + 1] = partialDif.x;
                        array[1+i*3, degList[i][j] * 3 + 1] = partialDif.y;
                        array[2+i*3, degList[i][j] * 3 + 1] = partialDif.z;

                        partialDif = Vector3.Cross(Vector3.up, rad);//y
                        array[i*3, degList[i][j] * 3 + 2] = partialDif.x;
                        array[1+i*3, degList[i][j] * 3 + 2] = partialDif.y;
                        array[2+i*3, degList[i][j] * 3 + 2] = partialDif.z;
                    }
                }
                return DenseMatrix.OfArray(array);
            }

            public static Matrix<double> ComputeSingleChain(IKBone[] bones,int[] chain)
            {
                double[,] array = new double[3, (chain.Length - 1) * 3];
                for (int i = 0; i < chain.Length - 1; ++i)
                {
                    Vector3 rad = bones[chain[chain.Length - 1]].solvedPosition - bones[chain[i]].solvedPosition;

                    Vector3 partialDif = Vector3.Cross(Vector3.forward, rad);//z
                    array[0, i * 3] = partialDif.x;
                    array[1, i * 3] = partialDif.y;
                    array[2, i * 3] = partialDif.z;

                    partialDif = Vector3.Cross(Vector3.right, rad);//x
                    array[0, i * 3 + 1] = partialDif.x;
                    array[1, i * 3 + 1] = partialDif.y;
                    array[2, i * 3 + 1] = partialDif.z;

                    partialDif = Vector3.Cross(Vector3.up, rad);//y
                    array[0, i * 3 + 2] = partialDif.x;
                    array[1, i * 3 + 2] = partialDif.y;
                    array[2, i * 3 + 2] = partialDif.z;
                }
                return DenseMatrix.OfArray(array);
            }

            /// <summary>
            /// Calculate single elementary
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="effector"></param>
            /// <param name="joint"></param>
            /// <param name="row"></param>
            /// <param name="deg"></param>
            /// <param name="jacobianMatrix"></param>
            private static void WriteJacobian(IKBone[] bones,int effector,int joint,int row,ref int deg,ref double[,] jacobianMatrix)
            {
                Vector3 partDif;
                Vector3 rad = bones[effector].solvedPosition - bones[joint].solvedPosition;
                if (bones[joint].rotaionAxis.x == 1)
                {
                    partDif = Vector3.Cross(bones[joint].solvedRotation * Vector3.right, rad);
                    for (int i = 0; i < 3; ++i) jacobianMatrix[i+row, deg] = partDif[i];
                    deg++;
                }
                if (bones[joint].rotaionAxis.y == 1)
                {
                    partDif = Vector3.Cross(bones[joint].solvedRotation * Vector3.up, rad);
                    for (int i = 0; i < 3; ++i) jacobianMatrix[i+row, deg] = partDif[i];
                    deg++;
                }
                if (bones[joint].rotaionAxis.z == 1)
                {
                    partDif = Vector3.Cross(bones[joint].solvedRotation * Vector3.forward, rad);
                    for (int i = 0; i < 3; ++i) jacobianMatrix[i+row, deg] = partDif[i];
                    deg++;
                }
            }

            /// <summary>
            /// cacculate mutichains
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="chainList"></param>
            /// <param name="dim"></param>
            /// <returns></returns>
            public static Matrix<double> CalcJacobian(IKBone[] bones,List<int[]>chainList,int row,int col)
            {
                double[,] jacobianMatrix = new double[row, col];

                //spine
                int deg = 0, joint = 0;
                while (joint <3)
                {
                    WriteJacobian(bones, 3, joint, 0,ref deg, ref jacobianMatrix);
                    joint++;
                }

                //leftArm
                joint = 0;deg=0 ;
                while (joint <7)
                {
                    if (joint ==2 ) {
                        deg += 3;
                        joint += 2;
                    }
                    WriteJacobian(bones, 7, joint, 3, ref deg, ref jacobianMatrix);
                    joint++;
                }

                //rightArm
                joint = 0;deg = 0;
                while (joint <11)
                {
                    if (joint ==2)
                    {
                        deg += 9;
                        joint += 6;
                    }
                    WriteJacobian(bones, 11, joint, 6, ref deg, ref jacobianMatrix);
                    joint++;
                }
                return DenseMatrix.OfArray(jacobianMatrix);
            }

            /// <summary>
            /// Calculate single chain
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="chain"></param>
            /// <param name="dim"></param>
            /// <returns></returns>
            public static Matrix<double> CalcJacobian(IKBone[] bones,int[]chain, int[] dim)
            {
                int row = dim[0], col = dim[1];
                double[,] jacobianMatrix = new double[row, col];
                int deg = 0, joint = 0;

                while (deg < col && joint < 2)
                {
                    Vector3 rad = bones[2].solvedPosition - bones[joint].solvedPosition;
                    Vector3 partDif;
                    if (bones[joint].rotaionAxis.x == 1)
                    {
                        partDif = Vector3.Cross(bones[joint].solvedRotation * Vector3.right, rad);
                        for (int i = 0; i < 3; ++i) jacobianMatrix[i, deg] = partDif[i];
                        deg++;
                    }
                    if (bones[joint].rotaionAxis.y == 1)
                    {
                        partDif = Vector3.Cross(bones[joint].solvedRotation * Vector3.up, rad);
                        for (int i = 0; i < 3; ++i) jacobianMatrix[i, deg] = partDif[i];
                        deg++;
                    }
                    if (bones[joint].rotaionAxis.z == 1)
                    {
                        partDif = Vector3.Cross(bones[joint].solvedRotation * Vector3.forward, rad);
                        for (int i = 0; i < row; ++i) jacobianMatrix[i, deg] = partDif[i];
                        deg++;
                    }
                    joint++;
                }
                return DenseMatrix.OfArray(jacobianMatrix);
            }

            #endregion

            /// <summary>
            /// DLS
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="mat"></param>
            /// <param name="error"></param>
            /// <returns></returns>
            static public Vector<double> SolveDLS( Matrix<double> mat, double[] error,float lambda)
            {
                Matrix<double> matPMI =mat.Transpose()*(mat*mat.Transpose()+lambda*lambda*DenseMatrix.CreateIdentity(mat.RowCount)).Inverse();
                Vector errorVector = new DenseVector(error);

                return matPMI* errorVector;
            }

            /// <summary>
            /// JP
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="mat"></param>
            /// <param name="error"></param>
            /// <returns></returns>
            static public Vector<double> SolveJP(Matrix<double> mat, double[] error)
            {
                //Matrix<double> matPMI = (mat.Transpose()*mat).Inverse()*mat.Transpose();

                Matrix<double> matPMI = mat.PseudoInverse();
                Vector errorVector = new DenseVector(error);

                return matPMI * errorVector;
            }

            /// <summary>
            /// JI
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="mat"></param>
            /// <param name="error"></param>
            /// <returns></returns>
            static public Vector<double> SolveJInverse( Matrix<double> mat, double[] error)
            {
                Matrix<double> matPMI = mat.Inverse();
                Vector errorVector = new DenseVector(error);

                return matPMI * errorVector;
            }
            
            /// <summary>
            /// JT
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="mat"></param>
            /// <param name="error"></param>
            /// <returns></returns>
            static public Vector<double> SolveJT( Matrix<double> mat, double[] error)
            {
                Matrix<double> matT = mat.Transpose();
                Vector errorVector = new DenseVector(error);
               
                double alfa = VectorDot(errorVector, mat * matT * errorVector)/VectorDot(mat * matT * errorVector, mat * matT * errorVector) ;
                return alfa * matT * errorVector;
            }

            static double VectorDot(Vector<double> x,Vector<double> y)
            {
                double alfa = 0;
                for(int i = 0; i < x.Count; ++i)
                {
                    alfa += x[i] * y[i];
                }
                return alfa;
            }
        }

        public enum IKMode
        {
            JPI,
            JT,
            DLS,
            SVD,
            SVD_DLS,
            SDLS,
            JI,
            JPN
        }
        
        public class JacobianJointLimit
        {
            JointLimit[] limits;

            public  JacobianJointLimit(Transform[] joints)
            {
                limits = new JointLimit[joints.Length];
                for (int i = 0; i < limits.Length; ++i)
                {
                    if (joints[i].GetComponent<JointLimitEclipse>()!= null)
                    {
                        limits[i] = joints[i].GetComponent<JointLimitEclipse>().GetLimitSolver();
                        limits[i].Initiate(joints[i]);
                    }
                    else if (joints[i].GetComponent<JointLimitManager>() != null)
                    {
                        limits[i] = joints[i].GetComponent<JointLimitManager>().GetLimitSolver();
                        limits[i].Initiate(joints[i]);
                    }
                }
            }

            public Quaternion[] GetLimitedRotation(IKBone[] bones,float[] rotations,Quaternion rootRotation)
            {
                int deg = 0;
                Quaternion[] localRotations = new Quaternion[limits.Length];
                for (int i = 0; i < limits.Length; ++i)
                {
                    Vector3 angle = Vector3.zero;
                    Quaternion rot, localRot, limitedLocalRot;
                    if (deg < rotations.Length)
                    {
                        if (bones[i].rotaionAxis.x == 1) angle.x = rotations[deg++];

                        if (bones[i].rotaionAxis.y == 1) angle.y = rotations[deg++];

                        if (bones[i].rotaionAxis.z == 1) angle.z = rotations[deg++];
                    }

                    //rot = bones[i].solvedRotation * Quaternion.Euler(angle) * Quaternion.Inverse(bones[i].solvedRotation);
                    //rot = Quaternion.Euler(angle);
                    rot = bones[i].solvedRotation * Quaternion.Euler(angle);

                    if (i == 0) localRot = Quaternion.Inverse(rootRotation) * rot;
                    else localRot = Quaternion.Inverse(bones[i - 1].solvedRotation) * rot;

                    limitedLocalRot = limits[i].GetLimitedLocalRotation(localRot);
                    //limitedLocalRot = localRot;

                    localRotations[i] = limitedLocalRot;
                }
                return localRotations;
            }
        }
    }
}


