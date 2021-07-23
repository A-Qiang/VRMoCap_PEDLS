using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
namespace Mocap
{
    namespace IK
    {
        [CustomEditor(typeof(JointLimitEclipse))]
        public class JointLimitEclipseEditor : JointLimitEditor
        {
            JointLimitEclipse script { get { return target as JointLimitEclipse; } }
            Vector3 axis { get { return script.limitSolver.axis; } }
            float majorMin { get { return Mathf.Deg2Rad * script.limitSolver.majorMin; } }
            float majorMax { get { return Mathf.Deg2Rad * script.limitSolver.majorMax; } }
            float minorMax { get { return Mathf.Deg2Rad * script.limitSolver.minorMax; } }
            float minorMin { get { return Mathf.Deg2Rad * script.limitSolver.minorMin; } }
            private void OnSceneGUI()
            {
                if (!Application.isPlaying) script.limitSolver.defaultLocalRotation = script.transform.localRotation;
                if (script.limitSolver.axis == Vector3.zero) return;

                DrawArrow(script.transform.position, 0.5f*GetWorldDirection(axis), LimitColor.colorDefault, "axis", 0.02f);

                //1
                int n =(int) script.limitSolver.majorMax/5;
                float a = majorMax, b = minorMax;
                Vector3 v1=Vector2.zero,v2=Vector2.zero;
                v1 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(majorMax, 0)));
                DrawArrow(script.transform.position, v1, LimitColor.colorDefault, "major max", 0.02f);
                for (int i = 1; i < n; ++i)
                {
                    float x = Mathf.Lerp(a, 0, (float) i/ n);
                    v2 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(x, Mathf.Sqrt(b * b - (b * b * x * x) / (a * a)))));
                    DrawLine(script.transform.position, v2, LimitColor.colorDefault, "");
                    DrawLine(script.transform.position+v1, v2-v1, LimitColor.colorDefault, "");
                    v1 = v2;
                }

                //2
                n =-(int) script.limitSolver.majorMin/5;
                a = majorMin; b = minorMax;
                v2 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(0,minorMax)));
                DrawArrow(script.transform.position, v1, LimitColor.colorDefault, "minor max", 0.02f);
                DrawLine(script.transform.position + v1, v2 - v1, LimitColor.colorDefault, "");
                v1 = v2;
                for (int i = 1; i <n; ++i)
                {
                    float x = Mathf.Lerp(0, a, (float)i / n);
                    v2 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(x, Mathf.Sqrt(b * b - (b * b * x * x) / (a * a)))));
                    DrawLine(script.transform.position, v2, LimitColor.colorDefault, "");
                    DrawLine(script.transform.position + v1, v2 - v1, LimitColor.colorDefault,"");
                    v1 = v2;
                }
                //3
                n = -(int)script.limitSolver.majorMin/5;
                a = majorMin; b = minorMin;
                v2 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(majorMin, 0)));
                DrawLine(script.transform.position + v1, v2 - v1, LimitColor.colorDefault, "");
                v1 = v2;

                DrawArrow(script.transform.position, v1, LimitColor.colorDefault, "major min", 0.02f);
                for (int i = 1; i <n; ++i)
                {
                    float x = Mathf.Lerp(a, 0, (float)i / n);
                    v2 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(x, -Mathf.Sqrt(b * b - (b * b * x * x) / (a * a)))));
                    DrawLine(script.transform.position, v2, LimitColor.colorDefault, "");
                    DrawLine(script.transform.position + v1, v2 - v1, LimitColor.colorDefault, "");
                    v1 = v2;
                }

                //4
                n = (int)script.limitSolver.majorMax/5;
                a = majorMax; b = minorMin;
                v2 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(0,minorMin)));
                DrawArrow(script.transform.position, v1, LimitColor.colorDefault, "minor min", 0.02f);
                DrawLine(script.transform.position + v1, v2 - v1, LimitColor.colorDefault, "");
                v1 = v2;
                for (int i = 1; i < n; ++i)
                {
                    float x = Mathf.Lerp(0, a, (float)i / n);
                    v2 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(x, -Mathf.Sqrt(b * b - (b * b * x * x) / (a * a)))));
                    DrawLine(script.transform.position, v2, LimitColor.colorDefault, "");
                    DrawLine(script.transform.position + v1, v2 - v1, LimitColor.colorDefault, "");
                    v1 = v2;
                }
                v2 = 0.5f * GetWorldDirection(script.limitSolver.GetRotation(new Vector2(majorMax, 0)));
                DrawLine(script.transform.position + v1, v2 - v1, LimitColor.colorDefault, "");
            }
            private Vector3 GetWorldDirection(Vector3 direction)
            {
                if (script.transform.parent == null) return script.limitSolver.defaultLocalRotation * direction;
                else return script.transform.parent.rotation * script.limitSolver.defaultLocalRotation * direction;
            }

            private Vector3 GetLocalDirection(Vector3 direction)
            {
                if (script.transform.parent == null) return Quaternion.Inverse(script.limitSolver.defaultLocalRotation) * direction;
                else return Quaternion.Inverse(script.limitSolver.defaultLocalRotation) * Quaternion.Inverse(script.transform.parent.rotation) * direction;
            }
        }
    }
}
