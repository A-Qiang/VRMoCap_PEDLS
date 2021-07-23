using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class HybridUpperSolver : MultiChainSolver
        {
            #region Analytical
            public ArmIK leftArm;
            public ArmIK rightArm;

            protected JointLimitPolygon leftShoulderJoint;
            protected JointLimitPolygon leftWristJoint;
            protected JointLimitPolygon rightShoulderJoint;
            protected JointLimitPolygon rightWristJoint;
            JointLimitPolygon[] polygonLimits;

            ArmIK[] arms=new ArmIK[2];
            [HideInInspector] public IKBone[] spineBones;
            #endregion
            
            Quaternion[] desiredRot = new Quaternion[12];
            Quaternion[] initRot = new Quaternion[12];
            Vector3[] initPos = new Vector3[12];

            Vector3 spineDefaultBendNormal=Vector3.right;
            Quaternion anchoreRelHeadRot;
            protected override void OnRead(Vector3[] positions, Quaternion[] rotations, int rootIndex, int index)
            {
                if (!Initiated)
                {
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

                    #region Analytical

                    spineBones = new IKBone[4];
                    leftArm.bones = new IKBone[4];
                    rightArm.bones = new IKBone[4];

                    for (int i = 0; i < spineBones.Length; ++i)
                    {
                        spineBones[i] = new IKBone(positions[i + 2], rotations[i + 2]);
                        leftArm.bones[i] = new IKBone(positions[i + 6], rotations[i + 6]);
                        rightArm.bones[i] = new IKBone(positions[i + 10], rotations[i + 10]);
                    }

                    leftArm.rootPos = spineBones[1].solvedPosition;
                    leftArm.rootRot = spineBones[1].solvedRotation;
                    rightArm.rootPos = spineBones[1].solvedPosition;
                    rightArm.rootRot = spineBones[1].solvedRotation;

                    IKBone.PreSolve(spineBones);
                    arms = new ArmIK[2] { leftArm, rightArm };
                    
                    for (int i = 0; i < 2; ++i)
                    {
                        arms[i].shoulderPolygonLimit = new PolygonLimit();
                        JointLimitTool.CloneLimit(polygonLimits[i*2], arms[i].shoulderPolygonLimit, (arms[i].bones[2].solvedPosition - arms[i].bones[1].solvedPosition).magnitude);

                        arms[i].wristPolygonLimit = new PolygonLimit();
                        JointLimitTool.CloneLimit(polygonLimits[i*2+1], arms[i].wristPolygonLimit, (arms[i].bones[3].solvedPosition - arms[i].bones[2].solvedPosition).magnitude);

                        arms[i].shoulderLocalRotation = Quaternion.Inverse(arms[i].bones[0].solvedRotation) * arms[i].bones[1].solvedRotation;
                        arms[i].wristLocalRotation = Quaternion.Inverse(arms[i].bones[2].solvedRotation) * arms[i].bones[3].solvedRotation;

                        arms[i].bendNormalRelClavicle = Quaternion.Inverse(arms[i].rootRot) * arms[i].GetBendNormal();
                        //arms[i].defaultRootRotation = arms[i].bones[0].solvedRotation;
                    }

                    #endregion
                    
                    anchoreRelHeadRot = Quaternion.Inverse(bones[3].solvedRotation) * rotations[0];

                    Initiated = !Initiated;
                }

                baseBone.solvedPosition = positions[rootIndex];
                baseBone.solvedRotation = rotations[rootIndex];

                for (int i = 0; i < bones.Length; ++i)
                {
                    bones[i].solvedPosition = positions[i + index];
                    bones[i].solvedRotation = rotations[i + index];
                }
                ReadBone();
            }
            
            #region Analytical IK
            
            void SolveAngle(float R, float yaw, float elevation, out float theta, out float beta, out float yelta, out float alfa)
            {
                theta = -4 + 1.1f * R + 0.9f * elevation;
                beta = 39.4f + 0.54f * R - 1.06f * elevation;
                yelta = 13.2f + 0.86f * yaw + 0.11f * elevation;
                alfa = -10.0f + 1.08f * yaw - 0.35f * elevation;
            }
            
            public void SolveHeuristicsAnalyticalIK(float estimatedWeight,bool limitJoint)
            {
                if (estimatedWeight==1)
                {
                    ReadBone();
                    SolveSpine(headTargetPos, headTargetRot);

                    for (int i = 0; i < 2; ++i)
                    {
                        TranslateArm(arms[i], spineBones[1].solvedPosition, spineBones[1].solvedRotation);
                        Solve7DOFArm(arms[i], arms[i].targetPos, arms[i].targetRot, limitJoint);
                    }
                    WriteBone();
                }
                else
                {
                    ReadBone();
                    GetInitPose();
                    SolveSpine(headTargetPos, headTargetRot);

                    for (int i = 0; i < 2; ++i)
                    {
                        TranslateArm(arms[i], spineBones[1].solvedPosition, spineBones[1].solvedRotation);
                        Solve7DOFArm(arms[i], arms[i].targetPos, arms[i].targetRot, limitJoint);
                    }

                    GetDesiredRot();
                    WriteInitPose();
                    RotateBone(desiredRot, estimatedWeight);

                    WriteBone();
                }
            }

            public void SolveAnalyticalIK(bool limitJoint)
            {
                ReadBone();
                SolveSpine(headTargetPos, headTargetRot);

                for (int i = 0; i < 2; ++i)
                {
                    TranslateArm(arms[i], spineBones[1].solvedPosition, spineBones[1].solvedRotation);
                }

                Solve7DOFArm(leftArm, leftArmTargetPos, leftArmTargetRot, limitJoint);
                Solve7DOFArm(rightArm, rightArmTargetPos, rightArmTargetRot, limitJoint);

                WriteBone();
            }

            /// <summary>
            /// Get joint constraint
            /// </summary>
            /// <param name="solverTransforms"></param>
            /// <param name="index"></param>
            public void InitiateLimit(Transform[] solverTransforms)
            {
                Transform[]spineJoints = new Transform[3] {
                    solverTransforms[(int)BodyIndex.Spine], solverTransforms[(int)BodyIndex.Chest], solverTransforms[(int)BodyIndex.Neck] };
                Transform[] leftArmJoints = new Transform[3] {
                    solverTransforms[(int)BodyIndex.LeftShoulder], solverTransforms[(int)BodyIndex.LeftUpperArm], solverTransforms[(int)BodyIndex.LeftLowerArm] };
                Transform[] rightArmJoints = new Transform[3] {
                    solverTransforms[(int)BodyIndex.RightShoulder], solverTransforms[(int)BodyIndex.RightUpperArm], solverTransforms[(int)BodyIndex.RightLowerArm] };
                spineJacobianLimit = new JacobianJointLimit(spineJoints);
                lArmJacobianLimit = new JacobianJointLimit(leftArmJoints);
                rArmJacobianLimit = new JacobianJointLimit(rightArmJoints);
                if (solverTransforms[7].GetComponent<JointLimitPolygon>() != null)
                {
                    leftShoulderJoint = solverTransforms[7].GetComponent<JointLimitPolygon>();
                }

                if (solverTransforms[9].GetComponent<JointLimitPolygon>() != null)
                {
                    leftWristJoint = solverTransforms[9].GetComponent<JointLimitPolygon>();
                }

                if (solverTransforms[11].GetComponent<JointLimitPolygon>() != null)
                {
                    rightShoulderJoint = solverTransforms[11].GetComponent<JointLimitPolygon>();
                }

                if (solverTransforms[13].GetComponent<JointLimitPolygon>() != null)
                {
                    rightWristJoint = solverTransforms[13].GetComponent<JointLimitPolygon>();
                }
                polygonLimits = new JointLimitPolygon[4] { leftShoulderJoint, leftWristJoint, rightShoulderJoint, rightWristJoint };
            }

            private void ReadBone()
            {
                for(int i = 0; i < 4; ++i)
                {
                    spineBones[i].solvedPosition = bones[i].solvedPosition;
                    spineBones[i].solvedRotation = bones[i].solvedRotation;
                    leftArm.bones[i].solvedPosition = bones[i+4].solvedPosition;
                    leftArm.bones[i].solvedRotation = bones[i + 4].solvedRotation;
                    rightArm.bones[i].solvedPosition = bones[i + 8].solvedPosition;
                    rightArm.bones[i].solvedRotation = bones[i + 8].solvedRotation;
                }
                leftArm.rootPos = spineBones[1].solvedPosition;
                leftArm.rootRot = spineBones[1].solvedRotation;
                rightArm.rootPos = spineBones[1].solvedPosition;
                rightArm.rootRot = spineBones[1].solvedRotation;

                leftArm.targetPos = leftArmTargetPos;
                leftArm.targetRot = leftArmTargetRot;
                rightArm.targetPos = rightArmTargetPos;
                rightArm.targetRot = rightArmTargetRot;
            }
            
            private void WriteBone()
            {
                for(int i = 0; i < 4; ++i)
                {
                    bones[i].solvedPosition = spineBones[i].solvedPosition;
                    bones[i].solvedRotation = spineBones[i].solvedRotation;
                    bones[i + 4].solvedPosition = leftArm.bones[i].solvedPosition;
                    bones[i + 4].solvedRotation = leftArm.bones[i].solvedRotation;
                    bones[i + 8].solvedPosition = rightArm.bones[i].solvedPosition;
                    bones[i + 8].solvedRotation = rightArm.bones[i].solvedRotation;
                }
            }
            
            void GetInitPose()
            {
                for (int i = 0; i < 4; ++i)
                {
                    initPos[i] = spineBones[i].solvedPosition;
                    initRot[i] = spineBones[i].solvedRotation;
                    initPos[i+4] = leftArm.bones[i].solvedPosition;
                    initRot[i+4] = leftArm.bones[i].solvedRotation;
                    initPos[i+8] = rightArm.bones[i].solvedPosition;
                    initRot[i+8] = rightArm.bones[i].solvedRotation;
                }
            }

            void WriteInitPose()
            {
                for (int i = 0; i < 4; ++i)
                {
                    spineBones[i].solvedPosition = initPos[i];
                    spineBones[i].solvedRotation = initRot[i];
                    leftArm.bones[i].solvedPosition = initPos[i+4];
                    leftArm.bones[i].solvedRotation = initRot[i+4];
                    rightArm.bones[i].solvedPosition = initPos[i+8];
                    rightArm.bones[i].solvedRotation = initRot[i+8];
                }
            }

            void GetDesiredRot()
            {
                for (int i = 0; i < 4; ++i)
                {
                    desiredRot[i] = spineBones[i].solvedRotation;
                    desiredRot[i + 4] = leftArm.bones[i].solvedRotation;
                    desiredRot[i + 8] = rightArm.bones[i].solvedRotation;
                }
            }

            private void RotateBone(Quaternion[] desiredRot,float weight)
            {
                Quaternion q ;
                for(int i = 0; i < 4; ++i)
                {
                    q = Quaternion.Slerp(Quaternion.identity, desiredRot[i]*Quaternion.Inverse(spineBones[i].solvedRotation), weight);
                    IKBone.RotateAroundPoint(spineBones, i,spineBones[i].solvedPosition, q);

                    q = Quaternion.Slerp(Quaternion.identity,desiredRot[i+4]*Quaternion.Inverse(leftArm.bones[i].solvedRotation), weight);
                    IKBone.RotateAroundPoint(leftArm.bones, i, leftArm.bones[i].solvedPosition, q);

                    q = Quaternion.Slerp(Quaternion.identity,desiredRot[i+8]*Quaternion.Inverse(rightArm.bones[i].solvedRotation), weight);
                    IKBone.RotateAroundPoint(rightArm.bones, i, rightArm.bones[i].solvedPosition, q);
                }
            }
            
            private void Solve7DOFArm(ArmIK arm, Vector3 targetPos, Quaternion targetRot, bool limitJoint)
            {
                //Vector3 shoulderNormal = Vector3.Cross(arm.bones[1].solvedPosition - arm.bones[0].solvedPosition, targetPos - arm.bones[0].solvedPosition).normalized;
                //IKBone.SolveTrigometric(arm.bones, 0, 2, 3, targetPos, shoulderNormal, 0.5f);
                arm.analyticalIK.firstPolygon = new PolygonLimit();
                JointLimitTool.TranslateJointLimit(arm.shoulderPolygonLimit, arm.analyticalIK.firstPolygon, arm.bones[1].solvedPosition, arm.bones[0].solvedRotation * arm.shoulderLocalRotation);

                arm.analyticalIK.thirdPolygon = new PolygonLimit();
                JointLimitTool.TranslateJointLimit(arm.wristPolygonLimit, arm.analyticalIK.thirdPolygon, arm.bones[3].solvedPosition, arm.bones[2].solvedRotation * arm.wristLocalRotation);

                Vector3 bendNormal = arm.rootRot * arm.bendNormalRelClavicle;

                arm.analyticalIK.limitJoint = limitJoint;
                arm.analyticalIK.SolveLimited7DOF(arm.bones, 1, 2, 3, targetPos, targetRot, bendNormal);
            }
            
            private void SolveSpine(Vector3 headPos, Quaternion headRot)
            {
                IKBone.SolveFABRIK(spineBones, spineBones[0].solvedPosition, headPos, 1);
                BendSpine();

                Vector3 spineBendNormal = headRot * anchoreRelHeadRot * spineDefaultBendNormal;
                IKBone.SolveTrigometric(spineBones, 0, 2, 3, headPos, spineBendNormal, 0.75f);
                IKBone.SolveTrigometric(spineBones, 1, 2, 3, headPos, spineBendNormal, 1);

                Quaternion q = headRot *Quaternion.Inverse( spineBones[3].solvedRotation);
                IKBone.RotateAroundPoint(spineBones, 3, spineBones[3].solvedPosition, q);
            }

            private void BendSpine()
            {
                Quaternion q = headTargetRot * Quaternion.Inverse(spineBones[spineBones.Length - 1].solvedRotation);
                float step = 1.0f / (spineBones.Length - 1);

                for (int i = 1; i < spineBones.Length; ++i)
                {
                    IKBone.RotateAroundPoint(spineBones, i, spineBones[i].solvedPosition, Quaternion.Slerp(Quaternion.identity, q, step));
                }

            }

            private void TranslateArm(ArmIK arm, Vector3 newRootPos, Quaternion newRootRot)
            {
                //position
                Vector3 deltaPos = newRootPos - arm.rootPos;
                arm.rootPos = newRootPos;
                foreach (var bone in arm.bones) bone.solvedPosition += deltaPos;

                //rotation
                Quaternion deltaRot = newRootRot * Quaternion.Inverse(arm.rootRot);
                arm.rootRot = newRootRot;
                IKBone.RotateAroundPoint(arm.bones, 0, arm.rootPos, deltaRot);
            }

            #endregion

        }

        [System.Serializable]
        public class ArmIK
        {
            public Vector3 defaultBendNormal=Vector3.up;
            public AnalyticalIK analyticalIK;

            [HideInInspector] public Vector3 targetPos;
            [HideInInspector] public Quaternion targetRot;

            [HideInInspector] public Vector3 rootPos;
            [HideInInspector] public Quaternion rootRot;
            [HideInInspector] public IKBone[] bones;

            [HideInInspector] public PolygonLimit shoulderPolygonLimit;
            [HideInInspector] public PolygonLimit wristPolygonLimit;

            [HideInInspector] public Quaternion shoulderLocalRotation;
            [HideInInspector] public Quaternion wristLocalRotation;

            [HideInInspector] public Vector3 bendNormalRelClavicle;
            //public Quaternion defaultRootRotation;
            public Vector3 GetBendNormal()
            {
                Vector3 normal = Vector3.Cross(bones[3].solvedPosition - bones[1].solvedPosition, bones[2].solvedPosition - bones[1].solvedPosition).normalized;
                if (normal == Vector3.zero)
                {
                    normal = defaultBendNormal;//left=down,right=up
                }
                else
                {
                    normal = normal.normalized;
                }
                return normal;
            }

            //public Vector3 wristToPalmAxis;
            //public Vector3 palmToThumbAxis;
            //public Vector3 GussBendNormal(Quaternion chestRotation)
            //{
            //    Vector3 dir = targetPos - bones[1].solvedPosition;
            //    Vector3 armDir = bones[0].solvedRotation * Vector3.left;

            //    Vector3 f = Vector3.down;
            //    Vector3 t = Quaternion.Inverse(chestRotation) * dir.normalized + Vector3.forward;
            //    Quaternion q = Quaternion.FromToRotation(f, t);

            //    Vector3 b = q * Vector3.back;

            //    f = Quaternion.Inverse(chestRotation) * armDir;
            //    t = Quaternion.Inverse(chestRotation) * dir;
            //    q = Quaternion.FromToRotation(f, t);
            //    b = q * b;

            //    b = chestRotation * b;

            //    b += armDir;
            //    b -= targetRot * wristToPalmAxis;
            //    b -= targetRot * palmToThumbAxis * 0.5f;

            //    return Vector3.Cross(b, dir);
            //}
        }

    }
}
