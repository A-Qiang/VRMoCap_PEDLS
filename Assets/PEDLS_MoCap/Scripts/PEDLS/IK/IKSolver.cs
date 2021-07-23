using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataProcess;
using System.IO;
namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public abstract class IKSolver
        {
            public IKBone[] bones;

            protected float mag;

            protected float sqrtMag;

            protected int index;

            protected bool Initiated=false;

            protected Vector3 rootPosition;

            protected Quaternion rootRotation;

            //public abstract void Write(ref Vector3[] positions, ref Quaternion[] rotations);
            public void Read(Vector3[] positions, Quaternion[] rotations, int rootIndex, int index)
            {
                this.index = index;
                rootPosition = positions[rootIndex];
                rootRotation = rotations[rootIndex];
                OnRead(positions, rotations, rootIndex, index);
            }

            protected abstract void OnRead(Vector3[] positions, Quaternion[] rotations, int rootIndex, int index);

            public  void MoveTo(Vector3 position)
            {
                Vector3 delta = position - bones[0].solvedPosition;
                foreach (var bone in bones) bone.solvedPosition += delta;
            }

            /// <summary>
            /// Translate rootPosition and rootRotation
            /// </summary>
            /// <param name="newRootPos"></param>
            /// <param name="newRootRot"></param>
            public void TranslateRoot(Vector3 newRootPos,Quaternion newRootRot)
            {
                Vector3 deltaPos = newRootPos - rootPosition;
                rootPosition = newRootPos;
                foreach (var bone in bones) bone.solvedPosition += deltaPos;

                Quaternion deltaRot =newRootRot* Quaternion.Inverse(rootRotation);
                rootRotation = newRootRot;
                IKBone.RotateAroundPoint(bones, 0, rootPosition, deltaRot);
            }

            public void Write(ref Vector3[] positions,ref Quaternion[] rotations)
            {
                if (index == 1)
                {
                    positions[0] = rootPosition;
                    rotations[0] = rootRotation;
                    positions[1] = bones[0].solvedPosition;
                }
                for (int i = 0; i < bones.Length; ++i)
                {
                    positions[i + index] = bones[i].solvedPosition;
                    rotations[i + index] = bones[i].solvedRotation;
                }
            }
            
        }
    }
}
