using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataProcess;
using System.Diagnostics;
namespace Mocap
{
    namespace IK
    {
        public class Utilities
        {
            public static void WriteFrameResult(string path,int frameCount,int iterCount,float value)
            {
                if (frameCount == 1)
                {
                    FileProcess.WriteHeader(path, "frame\titerCount\tvalue\r\n");
                }
                string res = frameCount + "\t" + iterCount + "\t" + value + "\r\n";
                FileProcess.Write(path, res);
            }

            public static Vector3 GetAxis(Axis axis)
            {
                switch (axis)
                {
                    case Axis.X:
                        return Vector3.right;
                    case Axis.Y:
                        return Vector3.up;
                    case Axis.Z:
                        return Vector3.forward;
                    default:
                        return Vector3.zero;
                }
            }
            
        }

        public enum Axis
        {
            X,
            Y,
            Z
        }

        [System.Serializable]
        public class IKResult
        {
            public bool recordResult;
            [HideInInspector]public float weight;
            public int frameCount;
            public int maxFrame;
            public int iterCount;
            public Transform actualBonesRoot;
            public HumanSkeleton actualBones;
            public Animator animator;

            Transform[] resultBones;
            Stopwatch sw=new Stopwatch();

            //time cost
            float costTime;
            float totleCostTime = 0;
            int iterCountTotle = 0;
            
            #region MAE of joint and effectors

            int[] effectorIndex = new int[5] { 3, 7, 11, 15, 18 };

            float eposPerFrameTotle=0;
            float eposTotal = 0;
            
            float erotTotal = 0;
            float erotPerFrameTotle = 0;

            #endregion
            
            bool findactualBone = false;
            float upperTotleErr=0;
            float upperFrameErr = 0;
            float leftLegTotleErr=0;
            float leftFrameErr = 0;
            float rightLegTotleErr=0;
            float rightFrameErr = 0;

            public void WriteResult(Transform[] solverTransforms, string ikName,float weight)
            {
                this.weight = weight;
                int motionName = animator.GetInteger("name");
                MotionType motion = FileProcess.GetMotionType(motionName);

                if (!findactualBone)
                {
                    findactualBone = true;
                    actualBones.FindTransforms(actualBonesRoot);
                }
                GetTransforms(solverTransforms);

                FactorAnalysis(solverTransforms, motion, ikName);
                //AppendStatistics(solverTransforms, motion, ikName);
                //WriteFrameResult(solverTransforms, motion, ikName);
                //AppendBodyPartResult(solverTransforms, motion, ikName);
                //WriteSmoothness(motion, ikName);
            }

            public void ReadHandGesture()
            {

            }

            void CalcAcceration()
            {

            }

            #region smoothness

            Vector3[] lastPositions = new Vector3[19];
            Quaternion[] lastRotations = new Quaternion[19];
            Vector3[] previousPositions = new Vector3[19];
            Quaternion[] previousRotations = new Quaternion[19];
            float smoothnessFrameTotle=0;
            float smooothnessTotle = 0;
            
            void WriteSmoothness(MotionType motion,string ikName)
            {
                if (recordResult)
                {
                    frameCount++;
                    string str;
                    string path = @"D:\zq\report\doctor\周报告\result\" +"_smoothness_results" + ".txt";
                    string perFramePath= @"D:\zq\report\doctor\周报告\result\" +  weight+"perframe_smoothness_results" + ".txt";
                    if (frameCount == 1)
                    {
                        Init();
                        return;
                    }
                    CalcSmoothness();
                    //str = frameCount + "\t" + smoothnessFrameTotle / 19 + "\r\n";
                    //FileProcess.Write(perFramePath, str);
                    if (!GetAnimatorState(motion))
                    {
                        float smoothness = smooothnessTotle /(19* frameCount);
                        str = FileProcess.GetMotionName(motion) + "\t" + weight +"\t"+ ikName + "\t" + smoothness + "\r\n";
                        FileProcess.Write(path, str);
                        UnityEngine.Debug.Log("write done");
                        recordResult = false;
                    }
                }
            }
            float lastTime=0;
            void Init()
            {
                lastTime = Time.time;
                for (int i = 0; i < 19; ++i)
                {
                    previousRotations[i] = resultBones[i].rotation;
                    previousPositions[i] = resultBones[i].position;
                    lastPositions[i] = resultBones[i].position;
                    lastRotations[i] = resultBones[i].rotation;
                }
            }
            void CalcSmoothness()
            {
                float time = Time.time - lastTime;
                lastTime= Time.time;
                smoothnessFrameTotle = 0;
                for (int i = 0; i < 19; ++i)
                {
                    Vector3 v1 = lastPositions[i] - previousPositions[i];
                    Vector3 v2 = resultBones[i].position - lastPositions[i];
                    float dp = (v2 - v1).magnitude;
                    previousPositions[i] = lastPositions[i];
                    lastPositions[i] = resultBones[i].position;

                    smoothnessFrameTotle += dp / Mathf.Pow(time, 3);
                }
                smooothnessTotle += smoothnessFrameTotle;
            }

            #endregion

            #region 计算膝盖和手肘位置误差

            /// <summary>
            /// 计算膝盖和手肘位置误差
            /// </summary>
            void CalcBodyPartResult()
            {
                leftFrameErr = 0;
                rightFrameErr = 0;
                upperFrameErr = 0;
                leftFrameErr += (actualBones.Transforms[14].position - resultBones[14].position).magnitude;
                rightFrameErr += (actualBones.Transforms[17].position - resultBones[17].position).magnitude;
                upperFrameErr += (actualBones.Transforms[11].position - resultBones[11].position).magnitude
                    + (actualBones.Transforms[10].position - resultBones[10].position).magnitude
                    + (actualBones.Transforms[6].position - resultBones[6].position).magnitude
                    + (actualBones.Transforms[7].position - resultBones[7].position).magnitude;
                leftLegTotleErr += leftFrameErr;
                rightLegTotleErr += rightFrameErr;
                upperTotleErr += upperFrameErr;
            }

            void AppendBodyPartResult(Transform[] solverTransforms, MotionType motion, string ikName)
            {
                if (recordResult)
                {
                    string path= @"D:\zq\report\doctor\周报告\result\" + "body_result" + ".txt";
                    frameCount++;
                    CalcBodyPartResult();
                    if (!GetAnimatorState(motion))
                    {
                        float leftLegMAE = leftLegTotleErr / frameCount;
                        float rightLegMAE = rightLegTotleErr / frameCount;
                        float upperMAE = upperTotleErr / (3*frameCount);
                        string str = FileProcess.GetMotionName(motion) + "\t" + ikName + "\t" + leftLegMAE + "\t" + rightLegMAE + "\t" + upperMAE + "\r\n";
                        FileProcess.Write(path, str);
                        UnityEngine.Debug.Log("write done");
                        recordResult = false;
                    }
                }
            }
            #endregion

            void WriteAllResults(Transform[] solverTransforms, MotionType motion, string ikName)
            {
                if (recordResult)
                {
                    frameCount++;
                    GetTransforms(solverTransforms);
                    string path = @"D:\zq\report\doctor\周报告\result\" + FileProcess.GetMotionName(motion) + "_PerFrame_" + ikName + ".txt";
                    string statisticsPath = @"D:\zq\report\doctor\周报告\result\" + FileProcess.GetMotionName(motion) + "_Statistics_" + ikName + ".txt";
                    CalcResultErr();
                    string str;
                    if (frameCount == 1)
                    {
                        str = "Frame\tepos\terot\tjoint_MAE\teffector_MAE\tIterations\r\n";
                        FileProcess.WriteHeader(path, str);
                        str = "Name\tValue\r\n";
                        FileProcess.WriteHeader(statisticsPath, str);
                    }
                    float eposPerFrame = eposTotal / actualBones.Transforms.Length;
                    float erotPerFrame = erotTotal / actualBones.Transforms.Length;

                    str = frameCount + "\t" + eposPerFrame + "\t" + erotPerFrame + "\t" + iterCount + "\r\n";
                    FileProcess.Write(path, str);

                    if (!GetAnimatorState(motion))
                    {
                        int num = frameCount * actualBones.Transforms.Length;
                        float epos_MAE = eposTotal / num;
                        float erot_MAE = erotTotal / num;
                        float costTimeAvg = totleCostTime / frameCount;
                        float iterCountAvg = iterCountTotle / frameCount;

                        str = "iteration_Avg\t" + iterCountAvg + "\r\n"
                            + "cost_AVG\t" + costTimeAvg + "\r\n"
                            + "epos_MAE\t" + epos_MAE + "\r\n"
                            + "erot_MAE\t" + erot_MAE;
                        FileProcess.Write(statisticsPath, str);
                        UnityEngine.Debug.Log("write done");

                        recordResult = false;
                    }
                }
            }

            void WriteFrameResult(Transform[] solverTransforms, MotionType motion, string ikName)
            {
                if (recordResult)
                {
                    frameCount++;
                    string path = @"D:\zq\report\doctor\周报告\result\" + FileProcess.GetMotionName(motion) + "_PerFrame_" + ikName + ".txt";
                    if(ikName=="Our-IK")path= @"D:\zq\report\doctor\周报告\result\" + FileProcess.GetMotionName(motion) + "_PerFrame_" + ikName +"_"+weight+"_"+ ".txt";
                    CalcResultErr();
                    CalcLocalRotationErr();
                    string str;
                    if (frameCount == 1)
                    {
                        str = "Frame\tepos_MAE\terot_MAE\tcost_time\r\n";
                        FileProcess.WriteHeader(path, str);
                    }
                    float eposPerFrame = eposPerFrameTotle / actualBones.Transforms.Length;
                    float erotPerFrame = erotPerFrameTotle / actualBones.Transforms.Length;

                    str = frameCount + "\t" + eposPerFrame + "\t" + erotPerFrame + "\r\n";
                    FileProcess.Write(path, str);

                    if (!GetAnimatorState(motion))
                    {
                        UnityEngine.Debug.Log("write a period");
                        recordResult = false;
                    }
                }

            }

            /// <summary>
            /// 记录重建统计结果
            /// </summary>
            /// <param name="solverTransforms"></param>
            /// <param name="motion"></param>
            /// <param name="ikName"></param>
            void AppendStatistics(Transform[] solverTransforms,MotionType motion, string ikName)
            {
                if (recordResult)
                {
                    frameCount++;
                    string statisticsPath = @"D:\zq\report\doctor\周报告\result\"  + "comparisons" + ".txt";
                    CalcResultErr();

                    if (!GetAnimatorState(motion))
                    {
                        int num = frameCount * actualBones.Transforms.Length;
                        float epos_MAE = eposTotal / num;
                        float erot_MAE = erotTotal / num;
                        float costTimeAvg = totleCostTime / frameCount;

                        string str =FileProcess.GetMotionName(motion)+"\t"+ikName+"\t"+ costTimeAvg + "\t"+epos_MAE + "\t"+ erot_MAE+"\r\n";
                        FileProcess.Write(statisticsPath, str);
                        UnityEngine.Debug.Log("write done");

                        recordResult = false;
                    }
                }
            }

            void FactorAnalysis(Transform[] solverTransforms, MotionType motion, string ikName)
            {
                if (recordResult)
                {
                    frameCount++;
                    string statisticsPath = @"D:\zq\report\doctor\周报告\result\FactorAnalysis.txt";
                    CalcResultErr();

                    if (!GetAnimatorState(motion))
                    {
                        int num = frameCount * actualBones.Transforms.Length;
                        float epos_MAE = eposTotal / num;
                        float erot_MAE = erotTotal / num;
                        float costTimeAvg = totleCostTime / frameCount;

                        string str = FileProcess.GetMotionName(motion) + "\t" + ikName +"\t"+weight+ "\t" + costTimeAvg + "\t" + epos_MAE + "\t" + erot_MAE + "\r\n";
                        FileProcess.Write(statisticsPath, str);
                        UnityEngine.Debug.Log("write done");

                        recordResult = false;
                    }
                }
            }

            public void StartWatch()
            {
                sw.Start();
            }

            public void StopWatch()
            {
                sw.Stop();
                costTime = sw.ElapsedMilliseconds;
                sw.Reset();
                
            }

            private bool GetAnimatorState(MotionType motion)
            {
                AnimatorStateInfo stateInfo = animator.GetCurrentAnimatorStateInfo(0);
                if (stateInfo.normalizedTime > 1.0f && stateInfo.IsName(FileProcess.GetEndMotionName(motion)))
                {
                    return false;
                }
                return true;
            }

            private void GetTransforms(Transform[] solverTransforms)
            {
                resultBones = new Transform[19];
                for(int i = 0; i < resultBones.Length; ++i)
                {
                    if (i < 16) resultBones[i] = solverTransforms[i + 1];
                    else resultBones[i] = solverTransforms[i + 2];
                }
            }
            
            private void CalcResultErr()
            {
                eposPerFrameTotle = 0;
                erotPerFrameTotle = 0;
                for(int i = 0; i < actualBones.Transforms.Length; ++i)
                {
                    float epos = (resultBones[i].position - actualBones.Transforms[i].position).magnitude*100;
                    eposPerFrameTotle += epos;

                    float erot = Mathf.Abs(Quaternion.Angle(resultBones[i].rotation, actualBones.Transforms[i].rotation));
                    erotPerFrameTotle += erot;
                }
                eposTotal += eposPerFrameTotle;
                erotTotal += erotPerFrameTotle;
                totleCostTime += costTime;
                iterCountTotle += iterCount;
            }

            void CalcLocalRotationErr()
            {
                erotPerFrameTotle = 0;
                for (int i = 1; i < actualBones.Transforms.Length; ++i)
                {
                    Quaternion actualLocalRotation =Quaternion.Inverse(actualBones.Transforms[i-1].rotation)* actualBones.Transforms[i].rotation;
                    if (i == 5) actualLocalRotation = Quaternion.Inverse(actualBones.Transforms[2].rotation) * actualBones.Transforms[i].rotation;
                    if (i == 9) actualLocalRotation = Quaternion.Inverse(actualBones.Transforms[2].rotation) * actualBones.Transforms[i].rotation;
                    if (i == 13) actualLocalRotation = Quaternion.Inverse(actualBones.Transforms[0].rotation) * actualBones.Transforms[i].rotation;
                    if (i == 16) actualLocalRotation = Quaternion.Inverse(actualBones.Transforms[0].rotation) * actualBones.Transforms[i].rotation;
                    
                    float erot = Mathf.Abs(Quaternion.Angle(resultBones[i].localRotation,actualLocalRotation));
                    erotPerFrameTotle += erot;
                }
                erotTotal += erotPerFrameTotle;
            }
        }

        [System.Serializable]
        public class HumanSkeleton
        {
            public Transform hip;
            public Transform spine;
            public Transform chest; // Optional
            public Transform neck; // Optional
            public Transform head;
            public Transform leftShoulder; // Optional
            public Transform leftUpperArm;
            public Transform leftLowerArm;
            public Transform leftHand;
            public Transform rightShoulder; // Optional
            public Transform rightUpperArm;
            public Transform rightLowerArm;
            public Transform rightHand;
            public Transform leftUpperLeg;
            public Transform leftLowerLeg;
            public Transform leftFoot;
            public Transform rightUpperLeg;
            public Transform rightLowerLeg;
            public Transform rightFoot;

            public Transform[] Transforms
            {
                get
                {
                    return new Transform[19]{
                     hip, spine, chest, neck, head, leftShoulder, leftUpperArm, leftLowerArm, leftHand, rightShoulder, rightUpperArm, rightLowerArm, rightHand, leftUpperLeg, leftLowerLeg, leftFoot, rightUpperLeg, rightLowerLeg, rightFoot
                    };
                }
            }

            public void GetTransforms(Transform root)
            {
                Animator animator = root.GetComponent<Animator>();
                if (animator != null)
                {
                    hip = animator.GetBoneTransform(HumanBodyBones.Hips);
                    spine = animator.GetBoneTransform(HumanBodyBones.Spine);
                    chest = animator.GetBoneTransform(HumanBodyBones.Chest);
                    neck = animator.GetBoneTransform(HumanBodyBones.Neck);
                    head = animator.GetBoneTransform(HumanBodyBones.Head);
                    leftShoulder = animator.GetBoneTransform(HumanBodyBones.LeftShoulder);
                    leftUpperArm = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
                    leftLowerArm = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
                    leftHand = animator.GetBoneTransform(HumanBodyBones.LeftHand);
                    rightShoulder = animator.GetBoneTransform(HumanBodyBones.RightShoulder);
                    rightUpperArm = animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
                    rightLowerArm = animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
                    rightHand = animator.GetBoneTransform(HumanBodyBones.RightHand);
                    leftUpperLeg = animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
                    leftLowerLeg = animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
                    leftFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
                    rightUpperLeg = animator.GetBoneTransform(HumanBodyBones.RightUpperLeg);
                    rightLowerLeg = animator.GetBoneTransform(HumanBodyBones.RightLowerLeg);
                    rightFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
                }
            }

            public void FindTransforms(Transform root)
            {
                if (root.Find("hip") != null) hip = root.Find("hip");
                if (root.Find("spine") != null) spine = root.Find("spine");
                if (root.Find("chest") != null) chest = root.Find("chest");
                if (root.Find("neck") != null) neck = root.Find("neck");
                if (root.Find("head") != null) head = root.Find("head");
                if (root.Find("leftShoulder") != null) leftShoulder = root.Find("leftShoulder");
                if (root.Find("leftUpperArm") != null) leftUpperArm = root.Find("leftUpperArm");
                if (root.Find("leftLowerArm") != null) leftLowerArm = root.Find("leftLowerArm");
                if (root.Find("leftHand") != null) leftHand = root.Find("leftHand");
                if (root.Find("rightShoulder") != null) rightShoulder = root.Find("rightShoulder");
                if (root.Find("rightUpperArm") != null) rightUpperArm = root.Find("rightUpperArm");
                if (root.Find("rightLowerArm") != null) rightLowerArm = root.Find("rightLowerArm");
                if (root.Find("rightHand") != null) rightHand = root.Find("rightHand");
                if (root.Find("leftUpperLeg") != null) leftUpperLeg = root.Find("leftUpperLeg");
                if (root.Find("leftLowerLeg") != null) leftLowerLeg = root.Find("leftLowerLeg");
                if (root.Find("leftFoot") != null) leftFoot = root.Find("leftFoot");
                if (root.Find("rightUpperLeg") != null) rightUpperLeg = root.Find("rightUpperLeg");
                if (root.Find("rightLowerLeg") != null) rightLowerLeg = root.Find("rightLowerLeg");
                if (root.Find("rightFoot") != null) rightFoot = root.Find("rightFoot");
            }
        }

        public enum BodyIndex
        {
            Root=0,
            Hip=1,
            Spine=2,
            Chest=3,
            Neck=4,
            Head=5,
            LeftShoulder=6,
            LeftUpperArm=7,
            LeftLowerArm=8,
            LeftHand=9,
            RightShoulder=10,
            RightUpperArm=11,
            RightLowerArm=12,
            RightHand=13,
            LeftUpperLeg=14,
            LeftLowerLeg=15,
            LeftFoot=16,
            LeftToe=17,
            RightUpperLeg=18,
            RightLowerLeg=19,
            RightFoot=20,
            rightToe=21
        }
    }
}
