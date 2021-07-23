using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace DataProcess
    {
        public abstract class DrawTexture
        {
            public Texture2D texture;
            //public Vector2[] datas;
            //public PixelInfo[] dataPixels;

            /// <summary>
            /// Draw the background texture by width and height
            /// </summary>
            /// <param name="width"></param>
            /// <param name="height"></param>
            public void InitializeTexture(float width, float height)
            {
                height = Mathf.Clamp(height, 0.1f, 1.0f);
                width = Mathf.Clamp(width, 0.1f, 1);
                texture = new Texture2D((int)(Screen.width * width), (int)(Screen.height * height));
            }

            /// <summary>
            /// Draw pixel on the background texture
            /// </summary>
            /// <param name="pixelList"></param>
            protected void DrawPixel(List<PixelInfo> pixelList)
            {
                Color32[] color = texture.GetPixels32();

                for (int i = 0; i < pixelList.Count; ++i)
                {
                    int index = Mathf.Clamp(pixelList[i].index, 0, texture.width * texture.height-1);
                    color[index] = pixelList[i].color;
                }

                texture.SetPixels32(color);
                texture.Apply();
            }

            public abstract void Draw(List<Vector2> datas,Color32 color);

        }
        public class PixelInfo
        {
            public int index;
            public Color32 color;
            public PixelInfo(int index, Color32 color)
            {
                this.index = index;
                this.color = color;
            }
        }
    }
}
