using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Mocap
{
    namespace IK
    {
        public class IKBone
        {
            public Vector3 readPosition;

            public Quaternion readRotation;
            
            public Vector3 solvedPosition;

            public Quaternion solvedRotation;

            public float length;

            public Vector3 poleAxis;

            public float sqrtMag;
            public Vector3 rotaionAxis;
            public AngularRestriction limit;

            /// <summary>
            /// Initiate bone
            /// </summary>
            /// <param name="pos"></param>
            /// <param name="rot"></param>
            public IKBone(Vector3 pos,Quaternion rot)
            {
                Read(pos, rot);
            }
            public IKBone(Vector3 pos,Quaternion rot,Vector3 axis)
            {
                Read(pos, rot);
                rotaionAxis = axis;
            }
            public IKBone(Vector3 pos,Quaternion rot,AngularRestriction jointLimit)
            {
                Read(pos, rot);
                rotaionAxis = jointLimit.rotationAxis;
                limit = jointLimit;
            }

            /// <summary>
            /// Read position and rotation 
            /// </summary>
            /// <param name="pos"></param>
            /// <param name="rot"></param>
            public void Read(Vector3 pos,Quaternion rot)
            {
                readPosition = pos;
                readRotation = rot;
                solvedPosition = pos;
                solvedRotation = rot;
            }

            /// <summary>
            /// PreSolve the pole axis and mag of bones
            /// </summary>
            /// <param name="bones"></param>
            public static float PreSolve(IKBone[] bones)
            {
                float length = 0;
                for (int i = 0; i < bones.Length - 1; ++i)
                {
                    Vector3 offset = bones[i + 1].readPosition - bones[i].readPosition;
                    bones[i].length = offset.magnitude;
                    bones[i].sqrtMag = offset.sqrMagnitude;
                    bones[i].poleAxis = Quaternion.Inverse(bones[i].readRotation) * offset;
                    length += bones[i].length;
                }
                bones[bones.Length - 1].length = 0;
                bones[bones.Length - 1].sqrtMag = 0;
                bones[bones.Length - 1].poleAxis = Vector3.zero;

                return length;
            }

            /// <summary>
            /// FABRIK 
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="startPosition"></param>
            /// <param name="targetPosition"></param>
            /// <param name="iteration"></param>
            public static void SolveFABRIK(IKBone[] bones,Vector3 startPosition,Vector3 targetPosition,int iteration)
            {
                //Backward
                bones[bones.Length - 1].solvedPosition = targetPosition;
                for(int i = bones.Length-2; i>-1; --i)
                {
                    bones[i].solvedPosition = MoveFABRIK(bones[i].solvedPosition, bones[i+1].solvedPosition, bones[i].length);
                }

                //Forward
                bones[0].solvedPosition = startPosition;
                for(int i = 0; i < bones.Length - 1; ++i)
                {
                    bones[i + 1].solvedPosition = MoveFABRIK(bones[i + 1].solvedPosition, bones[i].solvedPosition, bones[i].length);
                }

                //rotation
                for(int i = 0; i < bones.Length-1; ++i)
                {
                    Vector3 dir = bones[i + 1].solvedPosition - bones[i].solvedPosition;
                    bones[i].solvedRotation = Quaternion.FromToRotation(bones[i].solvedRotation * bones[i].poleAxis, dir) * bones[i].solvedRotation;
                }
            }

            /// <summary>
            /// Move pos1 toward pos2 by length
            /// </summary>
            /// <param name="pos1"></param>
            /// <param name="pos2"></param>
            /// <param name="length"></param>
            /// <returns></returns>
            private static Vector3 MoveFABRIK(Vector3 pos1,Vector3 pos2,float length)
            {
                return (pos1 - pos2).normalized * length + pos2;
            }

            /// <summary>
            /// Rotate bones chain around point
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="index"></param>
            /// <param name="point"></param>
            /// <param name="q"></param>
            public static void RotateAroundPoint(IKBone[] bones,int index,Vector3 point,Quaternion q)
            {
                for(int i = index; i < bones.Length; ++i)
                {
                    bones[i].solvedPosition =point+ q * (bones[i].solvedPosition - point);
                    bones[i].solvedRotation = q * bones[i].solvedRotation;
                }
            }

            /// <summary>
            /// Get the orth bend normal
            /// </summary>
            /// <param name="normal"></param>
            /// <param name="dir"></param>
            /// <returns></returns>
            static Vector3 AdjustNormal(Vector3 normal,Vector3 dir)
            {
                return normal - dir * Vector3.Dot(normal, dir) / dir.magnitude;
            }
            
            #region Triangulation

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
            public static void SolveTrigometric(IKBone[] bones, int first, int second, int third, Vector3 targetPos, Vector3 bendNormal, float weight)
            {
                targetPos = Vector3.Slerp(bones[bones.Length - 1].solvedPosition, targetPos, weight);

                Vector3 dir = targetPos - bones[first].solvedPosition;

                float sqrtMag1 = (bones[second].solvedPosition - bones[first].solvedPosition).sqrMagnitude;
                float sqrtMag2 = (bones[third].solvedPosition - bones[second].solvedPosition).sqrMagnitude;

                if (dir.magnitude >= bones.Length)
                {
                    Quaternion q = Quaternion.FromToRotation(bones[second].solvedPosition - bones[first].solvedPosition, dir);
                    IKBone.RotateAroundPoint(bones, first, bones[first].solvedPosition, q);

                    q = Quaternion.FromToRotation(bones[third].solvedPosition - bones[second].solvedPosition, dir);
                    IKBone.RotateAroundPoint(bones, second, bones[second].solvedPosition, q);
                }
                else if (dir.magnitude < bones.Length)
                {
                    Vector3 bendDir = Vector3.Cross(dir, bendNormal);
                    Vector3 midPos = GetMidDir(dir, bendDir, sqrtMag1, sqrtMag2);

                    Quaternion q = Quaternion.FromToRotation(bones[second].solvedPosition - bones[first].solvedPosition, midPos);
                    q = Quaternion.Slerp(Quaternion.identity, q, weight);
                    IKBone.RotateAroundPoint(bones, first, bones[first].solvedPosition, q);

                    q = Quaternion.FromToRotation(bones[third].solvedPosition - bones[second].solvedPosition, dir - midPos);
                    q = Quaternion.Slerp(Quaternion.identity, q, weight);
                    IKBone.RotateAroundPoint(bones, second, bones[second].solvedPosition, q);
                }

            }

            public static Vector3 GetMidDir(Vector3 dir, Vector3 benDir, float a, float b)
            {
                float x = (a + dir.sqrMagnitude - b) / dir.magnitude / 2;
                float y = Mathf.Sqrt(Mathf.Clamp(a - x * x,0,Mathf.Infinity));
                return Quaternion.LookRotation(dir, benDir) * new Vector3(0, y, x);
            }

            #endregion

            public static void InitiateBoneChain(IKBone[] chain,IKBone[] bones,int index)
            {
                for(int i = 0; i < chain.Length; ++i)
                {
                    chain[i] =new IKBone( bones[i + index].solvedPosition,bones[i+index].solvedRotation);
                }
            }
        }
    }
}