using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataModel;

namespace Mocap
{
    namespace DataProcess
    {
        public class Motioner
        {
            Transform head;
            Transform leftHand;
            Transform rightHand;
            Transform pelvis;
            Transform leftFoot;
            Transform rightFoot;

            public Transform[] sensors;//The markers of vive sensors in vr scene
            public Transform[] bodyMarkers;//The bodyMarkers for ik
            Transform[] bodyTransforms;
            Vector3[] positionsOff;
            Quaternion[] rotationsOff;
            Quaternion[] sensorInitalRotations;

            public Motioner(Transform characterModel, Transform trackerRoot, Transform targetRoot)
            {
                bodyTransforms = GetTransforms(characterModel);
                sensors = GetMarkers(trackerRoot);
                bodyMarkers = GetMarkers(targetRoot);
            }

            /// <summary>
            /// calculate offset by initPose state
            /// </summary>
            public void Calibrate()
            {
                positionsOff = new Vector3[6];
                rotationsOff = new Quaternion[6];
                sensorInitalRotations = new Quaternion[6];
                for (int i = 0; i < 6; ++i)
                {
                    sensorInitalRotations[i] = sensors[i].rotation;
                    bodyMarkers[i].position = bodyTransforms[i].position;
                    bodyMarkers[i].rotation = bodyTransforms[i].rotation;
                    rotationsOff[i] =Quaternion.Inverse(sensorInitalRotations[i])*bodyMarkers[i].rotation;
                    positionsOff[i] = bodyMarkers[i].position - sensors[i].position;
                }
            }
            public void Update()
            {
                for(int i = 0; i < 6; ++i)
                {
                    bodyMarkers[i].rotation = sensors[i].rotation * rotationsOff[i];
                    bodyMarkers[i].position = sensors[i].position + sensors[i].rotation * Quaternion.Inverse(sensorInitalRotations[i]) * positionsOff[i];
                }
            }

            /// <summary>
            /// Get the transforms 
            /// </summary>
            /// <param name="root"></param>
            /// <returns></returns>
            Transform[] GetMarkers(Transform root)
            {
                Transform[] parts = new Transform[6];
                
                if (root.Find("head") != null)
                {
                    parts[0] = root.Find("head");
                }
                if (root.Find("leftHand") != null)
                {
                    parts[1] = root.Find("leftHand");
                }
                if (root.Find("rightHand") != null)
                {
                    parts[2] = root.Find("rightHand");
                }
                if (root.Find("pelvis") != null)
                {
                    parts[3] = root.Find("pelvis");
                }
                if (root.Find("leftFoot") != null)
                {
                    parts[4] = root.Find("leftFoot");
                }
                if (root.Find("rightFoot") != null)
                {
                    parts[5] = root.Find("rightFoot");
                }
                return parts;
            }

            Transform[] GetTransforms(Transform root)
            {
                Transform[] transforms = new Transform[6];
                Animator animator = root.GetComponent<Animator>();
                if (animator != null)
                {
                    head = animator.GetBoneTransform(HumanBodyBones.Head);
                    leftHand = animator.GetBoneTransform(HumanBodyBones.LeftHand);
                    rightHand = animator.GetBoneTransform(HumanBodyBones.RightHand);
                    pelvis = animator.GetBoneTransform(HumanBodyBones.Hips);
                    leftFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
                    rightFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
                }
                return new Transform[6] { head, leftHand, rightHand, pelvis,leftFoot, rightFoot };
            }
        }
        
    }
}
