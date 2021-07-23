using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace Mocap
{
    namespace IK
    {
        [CustomEditor(typeof(JointLimitHinge))]
        [CanEditMultipleObjects]
        public class JointLimitHingeEditor : JointLimitEditor
        {
            private JointLimitHinge script { get { return target as JointLimitHinge; } }
            private Vector3 axis { get { return script.limitSolver.axis; } }
            private Vector3 cross { get { return script.limitSolver.cross; } }

            private float degree=90;
            
            public override void OnInspectorGUI()
            {
                GUI.changed = false;

                // Draw the default inspector
                DrawDefaultInspector();

                if (GUI.changed) EditorUtility.SetDirty(script);
            }

            private void OnSceneGUI()
            {
                if (!Application.isPlaying) script.limitSolver.defaultLocalRotation = script.transform.localRotation;
                if (axis == Vector3.zero) return;
                //quick tool
                Handles.BeginGUI();
                GUILayout.BeginArea(new Rect(10, 10, 200, 100), "rotate","Window");
                EditorGUILayout.FloatField("angle", degree);
                if (GUILayout.Button("Rotate around axis"))
                {
                    script.limitSolver.zeroOffset += 90;
                    if (script.limitSolver.zeroOffset >= 360) script.limitSolver.zeroOffset = 0;
                }
                GUILayout.EndArea();
                Handles.EndGUI();
                Color color = LimitColor.colorDefault;
                DrawLine(script.transform.position, GetWorldDirection(axis), color, "axis");
                Vector3 zero= Quaternion.AngleAxis(script.limitSolver.zeroOffset, axis) * cross.normalized;
                //min,max,zero,arc
                //if (script.limitSolver.isLimit)
                {
                    DrawArrow(script.transform.position, 0.65f*GetWorldDirection(zero),color, "zero");

                    Quaternion q = Quaternion.AngleAxis(script.limitSolver.min, axis);
                    DrawArrow(script.transform.position,  GetWorldDirection(q*zero), color, "min",0.04f);

                    q = Quaternion.AngleAxis(script.limitSolver.max, axis);
                    DrawArrow(script.transform.position,  GetWorldDirection(q*zero), color, "max",0.04f);

                    Handles.color = LimitColor.colorDefaultTransparent;
                    Handles.color= Color.red;

                    Handles.DrawSolidArc(script.transform.position, GetWorldDirection(axis),GetWorldDirection( zero), script.limitSolver.min, 0.5f);
                    Handles.DrawSolidArc(script.transform.position, GetWorldDirection(axis),GetWorldDirection( zero), script.limitSolver.max, 0.5f);

                }
                //circle
                Handles.CircleHandleCap(0, script.transform.position, Quaternion.LookRotation(GetWorldDirection(axis), GetWorldDirection(zero)), 0.5f, EventType.Repaint);
            }

            private Vector3 GetWorldDirection(Vector3 direction)
            {
                if (script.transform.parent == null) return script.limitSolver.defaultLocalRotation * direction;
                else return script.transform.parent.rotation * script.limitSolver.defaultLocalRotation * direction;
            }
        }
    }
}
