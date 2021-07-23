using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
namespace Mocap
{
    namespace IK
    {
        public class JointLimitEditor : Editor
        {
            public static class LimitColor
            {
                // Universal color pallettes
                public static Color colorDefault { get { return new Color(0.0f, 1.0f, 1.0f, 1.0f); } }

                public static Color colorDefaultTransparent
                {
                    get
                    {
                        Color d = colorDefault;
                        return new Color(d.r, d.g, d.b, 0.2f);
                    }
                }
               public static Color colorPolygon { get { return new Color(colorDefault.r, colorDefault.g, colorDefault.b, 0.25f); } }
                public static Color colorHandles { get { return new Color(1.0f, 0.5f, 0.25f, 1.0f); } }
                public static Color colorRotationSphere { get { return new Color(1.0f, 1.0f, 1.0f, 0.1f); } }
                public static Color colorInvalid { get { return new Color(1.0f, 0.3f, 0.3f, 1.0f); } }
                public static Color colorValid { get { return new Color(0.2f, 1.0f, 0.2f, 1.0f); } }
            }
          
            protected void DrawSphere(Vector3 center,Color color,float size)
            {
                Handles.color = color;
                Handles.SphereHandleCap(0, center, Quaternion.identity, size, EventType.Repaint);
            }

            protected void DrawLine(Vector3 position, Vector3 direction, Color color, string label)
            {
                Handles.color = color;
                Handles.DrawLine(position, position+direction);
                GUI.color = color;
                Handles.Label(position + direction, label);
            }

            protected void DrawArrow(Vector3 position,Vector3 direction,Color color,string label = "",float size=0.01f)
            {
                Handles.color = color;
                Handles.DrawLine(position, position + direction);
                Handles.SphereHandleCap(0, position + direction, Quaternion.identity, size, EventType.Repaint);
                if (label != "")
                {
                    GUI.color = color;
                    Handles.Label(position + direction, label);
                }
            }

        }
    }
}
