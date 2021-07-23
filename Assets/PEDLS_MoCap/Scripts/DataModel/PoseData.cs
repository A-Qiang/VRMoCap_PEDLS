using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataProcess;
namespace Mocap
{
    namespace DataModel
    {
        /// <summary>
        /// pose data which contain joints data of trackers
        /// </summary>
        public class PoseData
        {
            public int index;
            public JointData[] joints;
            public string[] name;
            public int length;

            public PoseData(PoseMode pat)
            {
                switch (pat)
                {
                    case PoseMode.SixPoint:
                        joints = new JointData[6];
                        name = new string[6] { "head", "lHand", "rHand", "pelvis", "lFoot", "rFoot" };
                        length = 6;
                        break;
                    case PoseMode.EightPoint:
                        joints = new JointData[8];
                        name = new string[8] { "head", "lHand", "rHand", "pelvis", "lFoot", "rFoot", "lController", "rController"};
                        length = 8;
                        break;
                    case PoseMode.NinePoint:
                        joints = new JointData[9];
                        name = new string[9] { "head", "lHand", "rHand", "pelvis", "lFoot", "rFoot", "lController", "rController", "chest" };
                        length = 9;
                        break;
                    default:
                        joints = new JointData[6];
                        name = new string[6] { "head", "lHand", "rHand", "pelvis", "lFoot", "rFoot" };
                        length = 6;
                        break;
                }
            }
        }
    }
}
