using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataModel;
using Mocap.IK;
using System.IO;
namespace Mocap
{
    namespace DataProcess
    {
        [System.Serializable]
        public class PostProcess 
        {
            public PoseMode poseMode;

            #region static method

            /// <summary>
            /// Get a string contain frame ,position error and new error
            /// </summary>
            /// <param name="frameCount"></param>
            /// <param name="val"></param>
            /// <param name="modifiedVal"></param>
            /// <param name="trueVal"></param>
            /// <returns></returns>
            public static string GetPositionErrString(int frameCount, Transform val, Transform newVal, Transform trueVal)
            {
                float err;
                string t = frameCount.ToString() + "\t";

                err = GetPositionError(val, trueVal);
                t += err.ToString() + "\t";

                err = GetPositionError(newVal, trueVal);
                t += err.ToString() + "\n";
                return t;
            }

            /// <summary>
            /// Get a string contain frame ,rotation error and new error
            /// </summary>
            /// <param name="frameCount"></param>
            /// <param name="val"></param>
            /// <param name="newVal"></param>
            /// <param name="trueVal"></param>
            /// <returns></returns>
            public static string GetRotationErrString(int frameCount, Transform val, Transform newVal,Transform trueVal)
            {
                float err;
                string t = frameCount.ToString() + "\t";

                err = GetRotationError(val, trueVal);
                t += err.ToString() + "\t";

                err = GetRotationError(newVal, trueVal);
                t += err.ToString() + "\n";
                return t;
            }
            #endregion

            private float CalculatePositionError(PoseData trueVal,PoseData testVal)
            {
                float err=0f;
                for(int i = 0; i < trueVal.length; ++i)
                {
                    err += (trueVal.joints[i].position - testVal.joints[i].position).magnitude;
                }
                return err / testVal.joints.Length;
            }

            private void GetResult(Transform[] targets,Transform[] effectors,out PoseData targetVal,out PoseData testVal)
            {
                targetVal = new PoseData(poseMode);
                testVal = new PoseData(poseMode);
                for (int i = 0; i < targetVal.joints.Length; ++i)
                {
                    Vector3 pos = targets[i].position;
                    Quaternion rot = targets[i].rotation;
                    targetVal.joints[i] = new JointData(pos, rot);

                    pos = effectors[i].position;
                    rot = effectors[i].rotation;
                    testVal.joints[i] = new JointData(pos, rot);
                }
            }

            #region private method

            /// <summary>
            /// Get position error between two vector3
            /// </summary>
            /// <param name="fileName"></param>
            /// <param name="val"></param>
            /// <param name="trueVal"></param>
            private static float GetPositionError(Transform val,Transform trueVal)
            {
                return (val.position - trueVal.position).magnitude;
            }
            
            /// <summary>
            /// Get rotation angle between two quaternion
            /// </summary>
            /// <param name="val"></param>
            /// <param name="trueVal"></param>
            /// <returns></returns>
            private static float GetRotationError(Transform val,Transform trueVal)
            {
                return Mathf.Abs(Quaternion.Angle(val.rotation, trueVal.rotation));
            }

            #endregion

        }

    }
}