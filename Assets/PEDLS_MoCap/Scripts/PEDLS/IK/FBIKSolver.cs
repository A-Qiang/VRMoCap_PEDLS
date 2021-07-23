using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataProcess;

namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public abstract class FBIKSolver
        {
            //The transforms of skelton
            [HideInInspector]public IKSkeleton skeleton;
            [HideInInspector]public Transform[] solverTransforms;
            
            protected Vector3[] solvedPositions=new Vector3[22];
            protected Quaternion[] solvedRotations=new Quaternion[22];
            protected Vector3[] readPositions;
            protected Quaternion[] readRotations;
            public bool fixTranstorms=true;

            Vector3 defaultPelvisLocalPosition;
            Quaternion[] defaultLocalRotation=new Quaternion[21];

            protected void StoreDefaultPose()
            {
                defaultPelvisLocalPosition = solverTransforms[1].localPosition;
                for (int i = 1; i < solverTransforms.Length; ++i)
                {
                    defaultLocalRotation[i - 1] = solverTransforms[i].localRotation;
                }
            }

            protected void UseDefaultPose()
            {
                solverTransforms[1].localPosition = defaultPelvisLocalPosition;
                for (int i = 1; i < solverTransforms.Length; ++i)
                {
                    solverTransforms[i].localRotation = defaultLocalRotation[i - 1];
                }
            }
            
            public bool GetBone(Transform root)
            {
                bool isDetected = false;
                if (skeleton.AutoDetectTransforms(root))
                {
                    solverTransforms = skeleton.GetBoneTransforms();
                    isDetected = true;
                }

                readPositions = new Vector3[solverTransforms.Length];
                readRotations = new Quaternion[solverTransforms.Length];
                return isDetected;
            }

            protected void WriteTransforms()
            {
                for (int i = 1; i < solverTransforms.Length; ++i)
                {
                    if (i < 2)
                    {
                        solverTransforms[i].position = solvedPositions[i];
                    }
                    solverTransforms[i].rotation = solvedRotations[i];
                }
            }

            /// <summary>
            /// Read positions and rotations from transforms
            /// </summary>
            protected void UpdateTransforms()
            {
                for (int i = 0; i < solverTransforms.Length; ++i)
                {
                    readPositions[i] = solverTransforms[i].position;
                    readRotations[i] = solverTransforms[i].rotation;
                }
            }

            /// <summary>
            /// Get bones and initiate solvers
            /// </summary>
            /// <param name="root"></param>
            public void Initiate(Transform root)
            {
                if (GetBone(root))
                {
                    StoreDefaultPose();
                    UpdateTransforms();
                    Read();
                    OnInitiate();
                }
            }

            /// <summary>
            /// Solve and update bones states
            /// </summary>
            public void Update()
            {
                if (fixTranstorms)
                {
                    UseDefaultPose();
                }
                
                UpdateTransforms();
                Read();
                Solve();
                
                Write();
                WriteTransforms();

            }

            protected abstract void OnInitiate();

            protected abstract void Read();

            protected abstract void Solve();

            protected abstract void Write();
        }

    }
}
