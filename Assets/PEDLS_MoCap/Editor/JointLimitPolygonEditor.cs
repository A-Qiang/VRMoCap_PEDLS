using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace Mocap
{
    namespace IK
    {
        [CustomEditor(typeof(JointLimitPolygon))]
        [CanEditMultipleObjects]
        public class JointLimitPolygonEditor : JointLimitEditor
        {
            JointLimitPolygon script { get { return target as JointLimitPolygon; } }

            Vector3 axis { get { return script.limitSolver.axis; } }
            
            Vector3[] points { get { return script.limitSolver.originPoints; }set { script.limitSolver.originPoints = value; } }
            Vector3[] polygonPoints { get { return script.limitSolver.boundPoints; } set { script.limitSolver.boundPoints = value; } }
            private int selectedPoint = -1, addPoint = -1, deletePoint = -1;

            private float degree = 90;
            public override void OnInspectorGUI()
            {
                base.OnInspectorGUI();
                if (GUILayout.Button("Reset")) script.limitSolver.SetDefaultCones();
            }

            private void OnSceneGUI()
            {
                //Set default if not initiated
                if (script.limitSolver == null)
                {
                    Debug.Log("???");
                    return;
                }
                else if (!script.initiated)
                {
                    script.limitSolver.SetDefaultCones();
                    Debug.Log("initiated");
                    script.initiated = true;
                }

                GUI.changed = false;

                //Set default local rotation
                if (!Application.isPlaying) script.limitSolver.defaultLocalRotation = script.transform.localRotation;
                if (script.limitSolver.axis == Vector3.zero) return;

                //Quick tool
                Handles.BeginGUI();
                GUILayout.BeginArea(new Rect(10, 10, 200, 100),"Quick edit tool", "Window");
                GUILayout.BeginHorizontal();

                if (GUILayout.Button("Rotate X")) Rotate(Vector3.right, degree);
                if (GUILayout.Button("Rotate Y")) Rotate(Vector3.up, degree);
                if (GUILayout.Button("Rotate Z")) Rotate(Vector3.forward, degree);

                GUILayout.EndHorizontal();

                GUILayout.EndArea();
                Handles.EndGUI();

                //Sphere
                DrawSphere(script.transform.position, LimitColor.colorRotationSphere, 2.0f);

                //Axis
                //DrawLine(script.transform.position, GetWorldDirection(axis),LimitColor.colorDefault , "axis");
                DrawArrow(script.transform.position, GetWorldDirection(axis), LimitColor.colorDefault, "axis",  0.02f);
                //Point
                for(int i = 0; i < points.Length; ++i)
                {
                    //line and label 
                    Color color = GetColor(i);
                    DrawLine(script.transform.position,GetWorldDirection(points[i]),color, i.ToString());
                    
                    //button
                    Handles.color = LimitColor.colorHandles;

                    if (Handles.Button(script.transform.position + GetWorldDirection(points[i]),script.transform.rotation, 0.02f, 0.02f, Handles.DotHandleCap))
                    {
                        selectedPoint = i;
                    }

                    if (i == selectedPoint)
                    {
                        //selected tool
                        Handles.color = Color.white;
                        GUI.color = Color.white;
                        Handles.BeginGUI();
                        GUILayout.BeginArea(new Rect(10, Screen.height-200, 200, 120), "position " + i.ToString(),"Window");
                        GUILayout.BeginHorizontal();

                        if (GUILayout.Button("add point"))  addPoint = i;
                        if (GUILayout.Button("delete point"))
                        {
                            if (points.Length > 3)
                            {
                                deletePoint = i;
                            }
                        }
                        GUILayout.EndHorizontal();
                        EditorGUILayout.Vector3Field("point", script.transform.position + GetWorldDirection(points[i]));
                        GUILayout.EndArea();
                        Handles.EndGUI();

                        //position handles
                        Vector3 posWorld = Handles.PositionHandle(script.transform.position + GetWorldDirection(points[i]), Quaternion.identity);
                        Vector3 newPos = GetLocalDirection(posWorld - script.transform.position);
                        if (points[i] != newPos)
                        {
                            if (!Application.isPlaying)
                            {
                                Undo.RecordObject(script, "move limit point");
                                points[i] = newPos;
                            }
                        }
                        points[i].Normalize();
                    }

                }

                //delete
                if (deletePoint != -1)
                {
                    DeletePoint(deletePoint);
                    deletePoint = -1;
                }
                
                //add
                if (addPoint != -1)
                {
                    AddPoint(addPoint);
                    addPoint = -1;
                }

                script.limitSolver.BuildReachCones();

                //polygon
                for (int i = 0; i < polygonPoints.Length; ++i)
                {
                    Color color = GetColor(i);
                    DrawLine(script.transform.position, GetWorldDirection(polygonPoints[i]), LimitColor.colorPolygon, "");
                    if (i < polygonPoints.Length - 1)
                    {
                        DrawLine(script.transform.position + GetWorldDirection(polygonPoints[i]), GetWorldDirection(polygonPoints[i + 1] - polygonPoints[i]), LimitColor.colorDefault, "");
                    }
                    else
                    {
                        DrawLine(script.transform.position + GetWorldDirection(polygonPoints[i]), GetWorldDirection(polygonPoints[0] - polygonPoints[i]), LimitColor.colorDefault, "");
                    }
                }

                if (GUI.changed) EditorUtility.SetDirty(script);
            }

            private void DeletePoint(int index)
            {
                Vector3[] newPoints = new Vector3[points.Length - 1];
                for(int i = 0; i < newPoints.Length; ++i)
                {
                    if (i < index)
                    {
                        newPoints[i] = points[i];
                    }else if (i >= index)
                    {
                        newPoints[i] = points[i + 1];
                    }
                }
                points= newPoints;
            }

            private void AddPoint(int index)
            {
                Vector3[] newPoint = new Vector3[points.Length + 1];
                for (int i = 0; i < newPoint.Length; ++i)
                {
                    if (i < index + 1)
                    {
                        newPoint[i] = points[i];
                    }
                    else if (i == index + 1)
                    {
                        Vector3 tmp;
                        if (index == points.Length-1) tmp = (points[index] + points[0]) / 2;
                        else tmp = (points[index] + points[index + 1]) / 2;
                        newPoint[i] = tmp.normalized;
                    }
                    else
                    {
                        newPoint[i] = points[i - 1];
                    }
                }
                points = newPoint;
            }

            private void Rotate(Vector3 axis, float degree)
            {
                for (int i = 0; i < points.Length; ++i)
                {
                    points[i] = Quaternion.AngleAxis(degree, axis) * points[i];
                }
            }

            private Color GetColor(int index)
            {
                if (script.limitSolver.cones[index].vol > 0) return LimitColor.colorDefault;
                else return LimitColor.colorInvalid;
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