using UnityEngine;
using System.Collections.Generic;
using Mocap.DataProcess;

namespace GraphicsDraw.OnTexture2D
{
    public class DrawLineOnTexture : DrawTexture
    {

        /// <summary>
        /// Draw line betweent datas by specified color
        /// </summary>
        /// <param name="datas"></param>
        /// <param name="color"></param>
        public override void Draw(List<Vector2> datas,Color32 color)
        {
            if (datas.Count > 1)
            {
                List<PixelInfo> pixels = new List<PixelInfo>();

                //add the start  data into pixels
                pixels.Add(new PixelInfo(texture.width * Mathf.FloorToInt(datas[0].y) + Mathf.FloorToInt(datas[0].x), color));

                //add the interval data
                for (int i = 0; i < datas.Count - 1; ++i)
                {
                    Vector2 start = datas[i];
                    Vector2 end = datas[i + 1];
                    pixels.AddRange(GetLinePixels(datas[i], datas[i + 1], color));
                }
                
                DrawPixel(pixels);
            }
        }

        /// <summary>
        /// Draw a line between start and end in specified color
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="color"></param>
        /// <returns></returns>
        private List<PixelInfo> GetLinePixels(Vector2 start,Vector2 end,Color32 color)
        {
            List<PixelInfo> pixelList = new List<PixelInfo>();

            //add the start point in the interval

            start.x = Mathf.Clamp(start.x, 0, texture.width);
            end.x = Mathf.Clamp(end.x, 0, texture.width);
            start.y = Mathf.Clamp(start.y, 0, texture.height);
            end.y = Mathf.Clamp(end.y, 0, texture.height);

            int xStart =Mathf.FloorToInt(start.x);
            int xEnd = Mathf.FloorToInt(end.x);
            int yStart = Mathf.FloorToInt(start.y);
            int yEnd = Mathf.FloorToInt(end.y);
            float k = (float)(yEnd - yStart) /(float)(xEnd - xStart);
            
            int count=0,index=0,x=xStart;
            while (x<xEnd)
            {
                ++count;
                ++x;
                int y = Mathf.FloorToInt(yStart+(x-xStart) * k);
                index = y * texture.width + x;
                pixelList.Add(new PixelInfo(index, color));
            }
            return pixelList;
        }

    }
}
