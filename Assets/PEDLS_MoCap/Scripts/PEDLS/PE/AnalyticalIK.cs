using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        public class AnalyticalIK
        {
            #region joint limit interface
            
            public Axis axis=Axis.X;//twist axis

            //public bool displayPolygon;
            //public bool displayReversePolygon;
            //public Mesh sphereMesh;
            //public Material mat;

             //valid interval
            [HideInInspector]public Interval[] validInterval;
            [HideInInspector]public Interval firstTwistSwivel;
            [HideInInspector]public Interval[] firstSwingSwivel;
            [HideInInspector]public Interval[] firstInterval;
            [HideInInspector]public Interval reverseTwistSwivel;
            [HideInInspector]public Interval[] thirdSwingSwivel;
            [HideInInspector]public Interval[] thirdInterval;

            [Range(-180, 180)]float swivelOffset;
            float lastSwivelOffset=0;
            
            [HideInInspector]public bool limitJoint;
            [HideInInspector]public PolygonLimit firstPolygon;
            [HideInInspector]public PolygonLimit thirdPolygon;

            PolygonLimit reversePolygon;
            [HideInInspector]public Vector3 zeroSwivel, reverseZeroSwivel;
            Quaternion qFirst, qThird;
            [HideInInspector]public Circle swivel;
            float angle1, angle2;

            #endregion
            
            //public void SamplePose(float step,IKBone[] bones,int first,int last,Quaternion targetRot,bool isInverse)
            //{
            //    float tmp=optSwivel;
            //    if (!isInverse)
            //    {
            //        tmp = optSwivel + step;

            //        if (tmp > 180) tmp -= 360;
                    
            //        if (Interval.IsValid(validInterval, ref tmp) == -1)
            //        {
            //            if (validInterval.Length == 1) tmp = validInterval[optIntervalIndex].start;
            //            else if (validInterval.Length > 1)
            //            {
            //                if (optIntervalIndex == validInterval.Length - 1)
            //                {
            //                    optIntervalIndex = 0;
            //                    tmp = validInterval[optIntervalIndex].start;
            //                }
            //                else
            //                {
            //                    optIntervalIndex += 1;
            //                    tmp = validInterval[optIntervalIndex].start;
            //                }
            //            }
            //        }

            //        if (tmp >= swivelOffset && optSwivel <swivelOffset) return;//一个周期
            //    }
            //    else
            //    {
            //        tmp = optSwivel - step;

            //        if (tmp > 180) tmp += 360;

            //        if (Interval.IsValid(validInterval, ref tmp) == -1)
            //        {
            //            if (validInterval.Length == 1) tmp = validInterval[optIntervalIndex].end;
            //            else if (validInterval.Length > 1)
            //            {
            //                if (optIntervalIndex == 0)
            //                {
            //                    optIntervalIndex = validInterval.Length-1;
            //                    tmp = validInterval[optIntervalIndex].end;
            //                }
            //                else
            //                {
            //                    optIntervalIndex -= 1;
            //                    tmp = validInterval[optIntervalIndex].end;
            //                }
            //            }
            //        }
            //        if (tmp <= swivelOffset && optSwivel > swivelOffset) return;//一个周期
            //    }

            //    Quaternion q = Quaternion.AngleAxis(tmp-optSwivel, swivel.n);
            //    IKBone.RotateAroundPoint(bones, first, bones[first].solvedPosition, q);
            //    bones[last].solvedRotation = targetRot;
            //    optSwivel = tmp;
            //}
            
            /// <summary>
            /// limited 7DOF analytical IK
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="first"></param>
            /// <param name="second"></param>
            /// <param name="third"></param>
            /// <param name="targetPos"></param>
            /// <param name="targetRot"></param>
            /// <param name="bendNormal"></param>
            public void SolveLimited7DOF(IKBone[] bones, int first, int second, int third, Vector3 targetPos, Quaternion targetRot, Vector3 bendNormal)
            {
                Vector3 dir = targetPos - bones[first].solvedPosition;

                float dirLen = dir.magnitude;
                float len1 = (bones[second].solvedPosition - bones[first].solvedPosition).magnitude;
                float len2 = (bones[third].solvedPosition - bones[second].solvedPosition).magnitude;

                #region validation of twist angle limit

                //Quaternion firstRot = bones[first].solvedRotation;
                //Vector3 firstPoleAxis = bones[second].solvedPosition - bones[first].solvedPosition;

                #endregion

                if (len1 + len2 > dirLen)
                {
                    Interval[] reverseIntervals = SolveReverseIKChain(bones,  first,  second,  third, bendNormal, targetPos, targetRot);
                    firstInterval = SolveValidSwivel(bones, first, second, third, targetPos, bendNormal, false);

                    float offset = Vector3.SignedAngle(zeroSwivel-swivel.c, reverseZeroSwivel-swivel.c, swivel.n);
                    thirdInterval = JointLimitTool.ConvertReverseInterval(reverseIntervals, offset);
                    //thirdInterval = new Interval[1] { new Interval(-180, 180) };
                    validInterval = Interval.Intersection(firstInterval, thirdInterval);
                    
                    //optimize 
                    if (limitJoint) swivelOffset = GetOptSwivel(validInterval);
                    else
                    {
                        validInterval = new Interval[1] { new Interval(-180, 180) };
                    }

                    //stage3
                    Quaternion q = Quaternion.AngleAxis(swivelOffset, swivel.n);
                    IKBone.RotateAroundPoint(bones, first, bones[first].solvedPosition, q);

                    #region validation of twist angle limit

                    //Quaternion thirdRot = bones[third].solvedRotation;
                    //Vector3 thirdPoleAxis;
                    //switch (axis)
                    //{
                    //    case Axis.Y:
                    //        thirdPoleAxis = bones[third].solvedRotation * Vector3.down;
                    //        break;
                    //    case Axis.Z:
                    //        thirdPoleAxis = bones[third].solvedRotation * Vector3.back;
                    //        break;
                    //    default:
                    //        thirdPoleAxis = bones[third].solvedRotation * Vector3.left;
                    //        break;
                    //}

                    #endregion

                    //stage4
                    IKBone.RotateAroundPoint(bones, third, bones[third].solvedPosition, targetRot * Quaternion.Inverse(bones[third].solvedRotation));

                    #region  validation 
                    //Debug.DrawRay(bones[second].solvedPosition, q * qFirst * bendNormal);
                    //thirdTwistSwivel = JointLimitTool.ConvertReverseInterval(new Interval[1] { reverseTwistSwivel }, offset);

                    //Debug.DrawLine(swivel.c, zeroSwivel,Color.blue);
                    //Debug.DrawLine(swivel.c, reverseZeroSwivel,Color.blue);
                    //if (displayPolygon)
                    //{
                    //    DrawTool.DrawSphere(firstPolygon.s, sphereMesh, mat, 0.5f);
                    //    DrawTool.DrawPolygon(firstPolygon.s.c, firstPolygon.boundPoints, Color.red);

                    //    DrawTool.DrawArc(swivel, new Interval(-180, 180), zeroSwivel, Color.grey);
                    //    Debug.DrawLine(bones[first].solvedPosition, bones[second].solvedPosition, Color.red);
                    //    for (int i = 0; i < validInterval.Length; ++i)
                    //    {
                    //        DrawTool.DrawArc(swivel, validInterval[i], zeroSwivel, Color.green);
                    //    }
                    //}
                    //if (displayPolygon)
                    //{
                    //    DrawTool.DrawSphere(firstPolygon.s, sphereMesh, mat, 0.5f);
                    //    DrawTool.DrawPolygon(firstPolygon.s.c, firstPolygon.boundPoints, Color.red);

                    //    DrawTool.DrawArc(swivel, new Interval(-180, 180), zeroSwivel, Color.grey);
                    //    Debug.DrawLine(bones[first].solvedPosition, bones[second].solvedPosition, Color.red);
                    //    for (int i = 0; i < firstSwingSwivel.Length; ++i)
                    //    {
                    //        DrawTool.DrawArc(swivel, firstSwingSwivel[i], zeroSwivel, Color.green);
                    //    }
                    //}
                    //if (displayReversePolygon)
                    //{
                    //    thirdSwingSwivel = JointLimitTool.ConvertReverseInterval(thirdSwingSwivel, offset);

                    //    DrawTool.DrawArc(swivel, new Interval(-180, 180), zeroSwivel, Color.grey);
                    //    Debug.DrawLine(bones[second].solvedPosition, bones[third].solvedPosition, Color.red);
                    //    for (int i = 0; i < thirdSwingSwivel.Length; ++i)
                    //    {
                    //        DrawTool.DrawArc(swivel, thirdSwingSwivel[i], zeroSwivel, Color.green);
                    //    }
                    //}
                    
                    //firstTwist = JointLimitTool.GetTwist(Quaternion.Inverse(firstRot) * qFirst * firstRot, Quaternion.Inverse(firstRot) * swivel.n, optSwivel, axis);
                    //thirdTwist = JointLimitTool.GetTwist(Quaternion.Inverse(targetRot) * qThird * targetRot, Quaternion.Inverse(targetRot) * (-swivel.n), offset - optSwivel, axis);

                    ////actual twist angle
                    //Quaternion tmp = Quaternion.FromToRotation(firstPoleAxis, bones[second].solvedPosition - bones[first].solvedPosition);
                    //realFirstTwist = Quaternion.Angle(tmp, bones[first].solvedRotation * Quaternion.Inverse(firstRot));

                    //Vector3 newPoleAxis;
                    //switch (axis)
                    //{
                    //    case Axis.Y:
                    //        newPoleAxis = bones[third].solvedRotation * Vector3.down;
                    //        break;
                    //    case Axis.Z:
                    //        newPoleAxis = bones[third].solvedRotation * Vector3.back;
                    //        break;
                    //    default:
                    //        newPoleAxis = bones[third].solvedRotation * Vector3.left;
                    //        break;
                    //}
                    //Quaternion tmp1 = Quaternion.FromToRotation(thirdPoleAxis, newPoleAxis);
                    //realThirdTwist = Quaternion.Angle(tmp1, bones[third].solvedRotation * Quaternion.Inverse(thirdRot));

                    #endregion
                }
                else
                {
                    Quaternion q = Quaternion.FromToRotation(bones[third].solvedPosition - bones[first].solvedPosition, dir);
                    IKBone.RotateAroundPoint(bones, first, bones[first].solvedPosition, q);
                    IKBone.RotateAroundPoint(bones, third, bones[third].solvedPosition, targetRot * Quaternion.Inverse(bones[third].solvedRotation));
                }
            }

            #region joint limit method

            /// <summary>
            /// Valid reversed valid
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="first"></param>
            /// <param name="second"></param>
            /// <param name="third"></param>
            /// <param name="bendNormal"></param>
            /// <param name="targetPos"></param>
            /// <param name="targetRot"></param>
            /// <returns></returns>
            Interval[] SolveReverseIKChain(IKBone[] bones, int first, int second, int third, Vector3 bendNormal, Vector3 targetPos, Quaternion targetRot)
            {
                //reverse bones
                IKBone[] reverseBones = new IKBone[3]{
                new IKBone(bones[third].solvedPosition, bones[third].solvedRotation),
                new IKBone(bones[second].solvedPosition, bones[second].solvedRotation),
                new IKBone(bones[first].solvedPosition, bones[first].solvedRotation)
                };

                Vector3 offset = targetPos - reverseBones[0].solvedPosition;
                for (int i = 0; i < reverseBones.Length; ++i) reverseBones[i].solvedPosition += offset;//position

                Quaternion q = targetRot * Quaternion.Inverse(reverseBones[0].solvedRotation);//rotation
                IKBone.RotateAroundPoint(reverseBones, 0, reverseBones[0].solvedPosition, q);

                Vector3 reverseTarget = bones[first].solvedPosition;//reverse target
                Vector3 reverseBnedNormal = q * bendNormal;//reverse bend normal

                //reverse polygon
                Vector3 v1 = reverseBones[1].solvedPosition - reverseBones[0].solvedPosition;
                Vector3 v2;
                switch (axis)
                {
                    case Axis.Y:
                        v2= reverseBones[0].solvedRotation * Vector3.up;
                        break;
                    case Axis.Z:
                        v2 = reverseBones[0].solvedRotation * Vector3.forward;
                        break;
                    default:
                        v2 = reverseBones[0].solvedRotation * Vector3.right;
                        break;
                } 
                Vector3 mid = v2.normalized - v1.normalized;
                Vector3 mirNormal = Vector3.Cross(mid, Vector3.Cross(v1, v2));//normal of symmetrical face
                
                reversePolygon = new PolygonLimit();
                JointLimitTool.ReversePolygon(thirdPolygon, reversePolygon, targetPos, q, mirNormal.normalized);

                //if (displayReversePolygon)
                //{
                //    Debug.DrawRay(reversePolygon.s.c, reversePolygon.axis, Color.yellow);
                //    DrawTool.DrawSphere(reversePolygon.s, sphereMesh, mat,0.5f);
                //    DrawTool.DrawPolygon(reversePolygon.s.c, reversePolygon.boundPoints, Color.red);
                //}
                return SolveValidSwivel(reverseBones, 0, 1, 2, reverseTarget, -reverseBnedNormal, true);
            }

            /// <summary>
            /// Get valid interval
            /// </summary>
            /// <param name="bones"></param>
            /// <param name="first"></param>
            /// <param name="second"></param>
            /// <param name="third"></param>
            /// <param name="targetPos"></param>
            /// <param name="bendNormal"></param>
            /// <param name="isReverse"></param>
            /// <returns></returns>
            Interval[] SolveValidSwivel(IKBone[] bones, int first, int second, int third, Vector3 targetPos, Vector3 bendNormal,bool isReverse)
            {
                Vector3 dir = targetPos - bones[first].solvedPosition;
                float dirLen = dir.magnitude;
                float dirSqrtLen = dir.sqrMagnitude;
                Vector3 dirN = dir / dirLen;

                Vector3 curDir = bones[third].solvedPosition - bones[first].solvedPosition;
                float curDirLen = curDir.magnitude;
                float curSqrtDirLen = curDir.sqrMagnitude;

                float sqrt1 = (bones[second].solvedPosition - bones[first].solvedPosition).sqrMagnitude;
                float sqrt2 = (bones[third].solvedPosition - bones[second].solvedPosition).sqrMagnitude;
                float len1 = Mathf.Sqrt(sqrt1);
                float len2 = Mathf.Sqrt(sqrt2);

                Interval[] intervals = new Interval[0];

                if (len1 + len2 > dirLen && dirLen > Mathf.Abs(len1 - len2))
                {
                    Quaternion q1, q2, q;

                    //current angle
                    float curAngle1 = Mathf.Acos((sqrt1 + curSqrtDirLen - sqrt2) / (2 * curDirLen * len1));
                    float curAngle2 = Mathf.Acos((sqrt1 + sqrt2 - curSqrtDirLen) / (2 * len1 * len2));

                    //desired angle
                    angle1 = Mathf.Acos((sqrt1 + dirSqrtLen - sqrt2) / (2 * dirLen * len1));
                    angle2 = Mathf.Acos((sqrt1 + sqrt2 - dirSqrtLen) / (2 * len1 * len2));

                    Quaternion firstRot = bones[first].solvedRotation;

                    //stage 1
                    q1 = Quaternion.AngleAxis(Mathf.Rad2Deg * (angle1 - curAngle1), bendNormal);//q1 = Quaternion.identity;
                    IKBone.RotateAroundPoint(bones, first, bones[first].solvedPosition, q1);

                    q = Quaternion.AngleAxis(Mathf.Rad2Deg * (angle2 - curAngle2), bendNormal);
                    IKBone.RotateAroundPoint(bones, second, bones[second].solvedPosition, q);

                    //stage 2
                    q2 = Quaternion.FromToRotation(bones[third].solvedPosition - bones[first].solvedPosition, dirN);
                    IKBone.RotateAroundPoint(bones, first, bones[first].solvedPosition, q2);

                    List<Interval[]> intervalsList = new List<Interval[]>();
                    swivel = new Circle(len1 * Mathf.Cos(angle1) * dirN + bones[first].solvedPosition, dirN, len1 * Mathf.Sin(angle1));
                    if (!isReverse)
                    {
                        qFirst = q2 * q1;
                        zeroSwivel = bones[second].solvedPosition;

                        firstTwistSwivel = new Interval();
                        JointLimitTool.GetTwistSwivel(Quaternion.Inverse(firstRot) * q2 * q1 * firstRot, Quaternion.Inverse(firstRot) * dirN, firstPolygon.twistAngle, ref firstTwistSwivel, axis);
                        intervalsList.Add(new Interval[] { firstTwistSwivel });

                        firstSwingSwivel = new Interval[0];
                        JointLimitTool.GetSwingSwivel(firstPolygon, swivel, bones[second].solvedPosition, ref firstSwingSwivel);
                        if (firstSwingSwivel.Length > 0) intervalsList.Add(firstSwingSwivel);
                    }
                    else
                    {
                        qThird = q2 * q1;
                        reverseZeroSwivel = bones[second].solvedPosition;
                        reverseTwistSwivel = new Interval();
                        JointLimitTool.GetTwistSwivel(Quaternion.Inverse(firstRot) * q2 * q1 * firstRot, Quaternion.Inverse(firstRot) * dirN, reversePolygon.twistAngle, ref reverseTwistSwivel, axis);
                        intervalsList.Add(new Interval[] { reverseTwistSwivel });

                        thirdSwingSwivel = new Interval[0];
                        JointLimitTool.GetSwingSwivel(reversePolygon, swivel, bones[second].solvedPosition, ref thirdSwingSwivel);
                        if (thirdSwingSwivel.Length > 0) intervalsList.Add(thirdSwingSwivel);
                    }
                    intervals = Interval.GetIntersection(intervalsList);
                }
                return intervals;
            }

            void SolveClavicle(IKBone[] bones)
            {

            }

            /// <summary>
            /// Pick optimized swivel angle
            /// </summary>
            /// <param name="intervals"></param>
            /// <returns></returns>
            float GetOptSwivel(Interval[] intervals)
            {
                if (intervals.Length > 0)
                {
                    float mid = GetMidSwivel(intervals);
                    lastSwivelOffset = (mid + lastSwivelOffset) / 2;
                    return lastSwivelOffset;
                }
                else
                {
                    return lastSwivelOffset;
                    //return (firstTwistSwivel.start + firstTwistSwivel.end) / 2;
                    //if (firstInterval.Length > 0)
                    //{
                    //    float mid = GetMidSwivel(firstInterval);
                    //    lastSwivelOffset = (mid + lastSwivelOffset) / 2;
                    //    return (firstTwistSwivel.start + firstTwistSwivel.end) / 2;
                    //    return lastSwivelOffset;
                    //}
                    //else
                    //{
                    //    isSolved = false;
                    //    return lastSwivelOffset;
                    //}
                    
                }
            }

            /// <summary>
            /// Get mid of intervals
            /// </summary>
            /// <param name="intervals"></param>
            /// <returns></returns>
            float GetMidSwivel(Interval[] intervals)
            {
                int maxIntervalIndex = 0;
                float width = intervals[0].end - intervals[0].start;
                for (int i = 1; i < intervals.Length; ++i)
                {
                    if (intervals[i].end - intervals[i].start > width) maxIntervalIndex = i;
                }
                return (intervals[maxIntervalIndex].start + intervals[maxIntervalIndex].end) / 2;
            }

            /// <summary>
            /// Minimize swivel angle
            /// </summary>
            /// <param name="intervals"></param>
            /// <returns></returns>
            float GetMinSwivel(Interval[] intervals)
            {
                float min = intervals[0].start;
                for (int i = 1; i < intervals.Length; ++i)
                {
                    if (intervals[i].start < min) min = intervals[0].start;
                }
                return min;
            }
            #endregion
        }

    }
}

