using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace Mocap
{
    namespace IK
    {
        [CustomEditor(typeof(PEDLS))]
        [CanEditMultipleObjects]
        public class IKSolverManagerEditor : Editor
        {
            private PEDLS script { get { return target as PEDLS; } }

            void OnEnable()
            {
                if (serializedObject == null) return;

                // Changing the script execution order
                if (!Application.isPlaying)
                {
                    if (script.transform == null) Debug.Log("transform null");

                    script.skeleton.AutoDetectTransforms(script.transform);

                    // TODO Set dirty
                }
            }
        }

    }
}
