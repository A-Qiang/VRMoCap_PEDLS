using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace DataModel
    {
        /// <summary>
        /// Joint data contain a position and rotation 
        /// </summary>
        public class JointData
        {
            public Vector3 position;
            public Quaternion rotation;

            public JointData(Vector3 position,Quaternion rotation)
            {
                this.position = position;
                this.rotation = rotation;
            }
        }
    }
}
