using UnityEngine;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using System.IO;
using Mocap.DataModel;

namespace Mocap
{
    namespace DataProcess
    {
        /// <summary>
        /// A class used to read(write) pose data from(into) file 
        /// </summary>
        public class FileProcess
        {
            public float scale=1.0f;
            private PoseMode mode;
            private string path;
            
            #region main interface

            public static void Write(string path, string str)
            {
                if (!File.Exists(path))
                {
                    File.Create(path).Close();
                    StreamWriter sw = new StreamWriter(path);
                    sw.Write(str);
                    sw.Close();
                    sw.Dispose();
                }
                else
                {

                    StreamWriter sw = File.AppendText(path);
                    sw.Write(str);
                    sw.Close();
                    sw.Dispose();
                }
            }

            public static void WriteHeader(string path, string str)
            {
                StreamWriter sw = new StreamWriter(path);
                sw.Write(str);
                sw.Close();
                sw.Dispose();
            }

            #endregion

            /// <summary>
            /// Create a file directory in specified path
            /// </summary>
            /// <param name="directoryPath"></param>
            public FileProcess(string directoryPath)
            {
                path = directoryPath;
                if (!Directory.Exists(directoryPath))
                {
                    Directory.CreateDirectory(directoryPath);
                }
            }

            /// <summary>
            /// Append string in specified file
            /// </summary>
            /// <param name="path"></param>
            /// <param name="val"></param>
            public void AppendString(string fileName,string val)
            {
                if (!File.Exists(path+fileName))
                {
                    File.Create(path+fileName).Close();
                }
                StreamWriter sw = File.AppendText(path+fileName);
                sw.Write(val);
                sw.Close();
                sw.Dispose();
            }
            
            /// <summary>
            /// Clear file and write a new string
            /// </summary>
            /// <param name="fileName"></param>
            /// <param name="val"></param>
            public void ReWriteString(string fileName,string val)
            {
                if (!File.Exists(path + fileName))
                {
                    File.Create(path + fileName).Close();
                }
                StreamWriter sw = new StreamWriter(path+fileName);
                sw.Write(val);
                sw.Close();
                sw.Dispose();
            }

            /// <summary>
            /// Write the pose data in a frame into specified path
            /// </summary>
            /// <param name="path"></param>
            /// <param name="data"></param>
            public void WritePose(string path,int index,Transform[] joints)
            {
                PoseData data = GetPose(index,joints);
                if (!File.Exists(path))
                {
                    File.Create(path).Close();
                }
                StreamWriter sw = File.AppendText(path);
                //StreamWriter sw = new StreamWriter(path + name);
                string dataString = data.index.ToString() + "\t\n";
                for (int i = 0; i < data.length; ++i)
                {
                    dataString += data.name[i] + " " + data.joints[i].position.x.ToString() + " " 
                        + data.joints[i].position.y.ToString() + " " 
                        + data.joints[i].position.z.ToString()   + " " 
                        + data.joints[i].rotation.x.ToString() + " " 
                        + data.joints[i].rotation.y.ToString() + " " 
                        + data.joints[i].rotation.z.ToString() + " " 
                        + data.joints[i].rotation.w.ToString() + "\t\n";
                }
                   
                sw.Write(dataString);
                sw.Close();
                sw.Dispose();
            }

            public FileProcess(string directoryPath,PoseMode poseMode)
            {
                mode = poseMode;
                if (!Directory.Exists(directoryPath))
                {
                    Directory.CreateDirectory(directoryPath);
                }
            }
            

            /// <summary>
            /// Read a list of "PoseData" from a specified path
            /// </summary>
            /// <param name="path"></param>
            /// <param name="motion"></param>
            /// <returns></returns>
            public bool ReadMotion(string path,ref List<PoseData> motion)
            {
                if (!File.Exists(path))
                {
                    return false;
                }
                else
                {
                    StreamReader sr = new StreamReader(path);
                    string tmp = sr.ReadToEnd();
                    motion=ParseData(tmp);
                    sr.Close();
                    sr.Dispose();
                    return true;
                }
            }

            public static string GetMotionName(MotionType type)
            {
                switch (type)
                {
                    case MotionType.Boxing:
                        return "Boxing";
                    case MotionType.Jump:
                        return "Jump";
                    case MotionType.Run:
                        return "Run";
                    case MotionType.RunBackwards:
                        return "RunBackwards";
                    case MotionType.SoccerKick:
                        return "SoccerKick";
                    case MotionType.Sprint:
                        return "Sprint";
                    case MotionType.Walk:
                        return "Walk";
                    case MotionType.BM:
                        return "Basic_Motion";
                    case MotionType.HI:
                        return "Human_Inter";
                    case MotionType.EI:
                        return "Env_Inter";
                    case MotionType.AS:
                        return "Activity_Sport";
                    case MotionType.SS:
                        return "Situation";
                    case MotionType.Idle:
                        return "Idle";
                    default:
                        return "Free";
                }
            }
            public static string GetMotionName(int type)
            {
                switch (type)
                {
                    case 0:
                        return "Boxing";
                    case 1:
                        return "Jump";
                    case 2:
                        return "Run";
                    case 3:
                        return "RunBackwards";
                    case 4:
                        return "SoccerKick";
                    case 5:
                        return "Sprint";
                    case 6:
                        return "Walk";
                    case 7:
                        return "Basic_Motion";
                    case 8:
                        return "Human_Inter";
                    case 9:
                        return "Env_Inter";
                    case 10:
                        return "Activity_Sport";
                    case 11:
                        return "Situation";
                    case 12:
                        return "Idle";
                    default:
                        return "Free";
                }
            }
            public static MotionType GetMotionType(int name)
            {
                switch (name)
                {
                    case 0:
                        return MotionType.Boxing;
                    case 1:
                        return MotionType.Jump;
                    case 2:
                        return MotionType.Run;
                    case 3:
                        return MotionType.RunBackwards;
                    case 4:
                        return MotionType.SoccerKick;
                    case 5:
                        return MotionType.Sprint;
                    case 6:
                        return MotionType.Walk;
                    case 7:
                        return MotionType.BM;
                    case 8:
                        return MotionType.HI;
                    case 9:
                        return MotionType.EI;
                    case 10:
                        return MotionType.AS;
                    case 11:
                        return MotionType.SS;
                    case 12:
                        return MotionType.Idle;
                    default:
                        return MotionType.Idle;
                }
            }

            public static string GetEndMotionName(MotionType motion)
            {
                switch (motion)
                {
                    case MotionType.BM:
                        return "BasicMotions@Walk01 [RootMotion]";
                    case MotionType.HI:
                        return "18_01";
                    case MotionType.EI:
                        return "01_09";
                    case MotionType.AS:
                        return "06_15";
                    case MotionType.SS:
                        return "15_12";
                    case MotionType.Idle:
                        return "To Idle01";
                    case MotionType.Jump:
                        return "Jump";
                    case MotionType.Run:
                        return "Run";
                    case MotionType.RunBackwards:
                        return "Run Backwards";
                    case MotionType.Sprint:
                        return "Sprint";
                    case MotionType.Walk:
                        return "Walk";
                    case MotionType.Boxing:
                        return "Boxing";
                    case MotionType.SoccerKick:
                        return "SoccerKick";
                    default:
                        return "end";
                }
            }

            #region private class method

            /// <summary>
            /// Parse data to a pose list
            /// </summary>
            /// <param name="data"></param>
            /// <returns></returns>
            private List<PoseData> ParseData(string data)
            {
                List<PoseData> poseList = new List<PoseData>();

                //scale
                string headerPattern = @"scale(.*)\n[01]\t\nhead";
                Regex regexHeader = new Regex(headerPattern);
                Match headerMatch = regexHeader.Match(data);
                scale =float.Parse( headerMatch.Groups[1].ToString());

                //pose
                string posePattern;
                switch (mode)
                {
                    case PoseMode.EightPoint:
                        posePattern = @"([0-9]+)\t\nhead(.*)\nlHand(.*)\nrHand(.*)\npelvis(.*)\nlFoot(.*)\nrFoot(.*)\nlController(.*)\nrController(.*)\n";
                        break;
                    default:
                        posePattern = @"([0-9]+)\t\nhead(.*)\nlHand(.*)\nrHand(.*)\npelvis(.*)\nlFoot(.*)\nrFoot(.*)\n";
                        break;
                }
                Regex regex = new Regex(posePattern);
                MatchCollection matches;
                matches = regex.Matches(data);

                for(int i = 0; i < matches.Count; ++i)
                {
                    PoseData pose = new PoseData(mode);
                    pose.index = int.Parse(matches[i].Groups[1].ToString());

                    for(int j = 2; j < matches[i].Groups.Count; ++j)
                    {
                        string[] str = matches[i].Groups[j].ToString().Split(' ');
                        
                        Vector3 pos = new Vector3(float.Parse(str[1]), float.Parse(str[2]), float.Parse(str[3]));
                        Quaternion quat = new Quaternion(float.Parse(str[4]), float.Parse(str[5]), float.Parse(str[6]), float.Parse(str[7]));

                        pose.joints[j-2] = new JointData(pos,quat);
                    }
                    poseList.Add(pose);
                }

                return poseList;
            }

            /// <summary>
            /// Get pose data from the transforms
            /// </summary>
            /// <param name="joints"></param>
            /// <returns></returns>
            private PoseData GetPose(int index,Transform[] joints)
            {
                PoseData pose = new PoseData(mode)
                {
                    index = index
                };

                for (int i = 0; i < pose.joints.Length; ++i)
                {
                    pose.joints[i] = new JointData(joints[i].position, joints[i].rotation);
                }
                return pose;
            }

            #endregion
        }
        
        public enum  MotionType{
            Idle,
            Jump,
            Run,
            RunBackwards,
            Sprint,
            Walk,
            Boxing,
            SoccerKick,
            BM,
            HI,
            EI,
            AS,
            SS
        }
    }
}