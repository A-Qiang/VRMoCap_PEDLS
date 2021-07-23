using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
namespace Mocap
{
    namespace DataProcess
    {
        public class DrawLine
        {
            public LineRenderer lr;
            public Vector3 offset;
            public float ratio;
            /// <summary>
            /// draw a line by y which x is fixed and uniform
            /// </summary>
            /// <param name="y"></param>
            public void Draw(float[] y)
            {
                float x = 0;
                Vector3[] positions = new Vector3[y.Length];

                float delta = 0.01f;

                for (int i = 0; i < y.Length; i++)
                {
                    positions[i] = new Vector3(x + delta, ratio*y[i], 0)+offset;
                    x += delta;
                }
                lr.positionCount = y.Length;
                lr.SetPositions(positions);
            }


            public void Initialize(float yScale,Vector3 offset)
            {
                this.offset = offset;
                ratio = yScale;
                if (Shader.Find("Sprites/Default") != null)
                {
                    lr.material = new Material(Shader.Find("Sprites/Default"));
                }
                lr.startColor = Color.red;
                lr.endColor = Color.red;
                lr.startWidth = 0.01f;
                lr.endWidth = 0.01f;
            }

        }
    }
}