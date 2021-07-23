using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class IKSkeleton
        {
            public Transform root;
            public Transform pelvis;
            public Transform spine;
            public Transform chest; // Optional
            public Transform neck; // Optional
            public Transform head;
            public Transform leftShoulder; // Optional
            public Transform leftUpperArm;
            public Transform leftForearm;
            public Transform leftHand;
            public Transform rightShoulder; // Optional
            public Transform rightUpperArm;
            public Transform rightForearm;
            public Transform rightHand;
            public Transform leftThigh;
            public Transform leftCalf;
            public Transform leftFoot;
            public Transform leftToes; // Optional
            public Transform rightThigh;
            public Transform rightCalf;
            public Transform rightFoot;
            public Transform rightToes; // Optional

            public bool AutoDetectTransforms(Transform root)
            {
                Animator animator = root.GetComponent<Animator>();
                if (animator != null)
                {
                    this.root = root;
                    this.pelvis = animator.GetBoneTransform(HumanBodyBones.Hips);
                    this.spine = animator.GetBoneTransform(HumanBodyBones.Spine);
                    this.chest = animator.GetBoneTransform(HumanBodyBones.Chest);
                    this.neck = animator.GetBoneTransform(HumanBodyBones.Neck);
                    this.head = animator.GetBoneTransform(HumanBodyBones.Head);
                    this.leftShoulder = animator.GetBoneTransform(HumanBodyBones.LeftShoulder);
                    this.leftUpperArm = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
                    this.leftForearm = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
                    this.leftHand = animator.GetBoneTransform(HumanBodyBones.LeftHand);
                    this.rightShoulder = animator.GetBoneTransform(HumanBodyBones.RightShoulder);
                    this.rightUpperArm = animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
                    this.rightForearm = animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
                    this.rightHand = animator.GetBoneTransform(HumanBodyBones.RightHand);
                    this.leftThigh = animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
                    this.leftCalf = animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
                    this.leftFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
                    this.leftToes = animator.GetBoneTransform(HumanBodyBones.LeftToes);
                    this.rightThigh = animator.GetBoneTransform(HumanBodyBones.RightUpperLeg);
                    this.rightCalf = animator.GetBoneTransform(HumanBodyBones.RightLowerLeg);
                    this.rightFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
                    this.rightToes = animator.GetBoneTransform(HumanBodyBones.RightToes);
                    return true;
                }else
                {
                    return false;
                }
            }

            public Transform[] GetBoneTransforms()
            {
                return new Transform[22] {
                    root, pelvis, spine, chest, neck, head, leftShoulder, leftUpperArm, leftForearm, leftHand, rightShoulder, rightUpperArm, rightForearm, rightHand, leftThigh, leftCalf, leftFoot, leftToes, rightThigh, rightCalf, rightFoot, rightToes
                };
            }
        }

    }
}
