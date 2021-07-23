using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mocap
{
    namespace IK
    {
        [System.Serializable]
        /// <summary>
        /// Interval of swivel angle
        /// </summary>
        public struct Interval
        {
            public float start;
            public float end;

            public Interval(float start, float end)
            {
                this.start = start;
                this.end = end;
            }

            /// <summary>
            /// sort by start
            /// </summary>
            /// <param name="intervals"></param>
            public static void Sort(ref Interval[] intervals)
            {
                for (int i = 0; i < intervals.Length - 1; ++i)
                {
                    for (int j = i + 1; j < intervals.Length; ++j)
                    {
                        if (intervals[i].start > intervals[j].start)
                        {
                            Interval tmp = intervals[i];
                            intervals[i] = intervals[j];
                            intervals[j] = tmp;
                        }
                    }
                }
            }

            public static Interval[] Merge(Interval A, Interval B)
            {
                List<Interval> res = new List<Interval>();
                if (B.start > A.end) return new Interval[2] { A, B };
                else if (A.start > B.end) return new Interval[2] { B, A };
                else if (A.start > B.start)
                {
                    if (A.end > B.end) return new Interval[1] { new Interval(B.start, A.end) };
                    else return new Interval[1] { new Interval(B.start, B.end) };
                }
                else
                {
                    if (A.end > B.end) return new Interval[1] { new Interval(A.start, A.end) };
                    else return new Interval[1] { new Interval(A.start, B.end) };
                }
            }

            public  static Interval[] Merge(Interval[] intervals)
            {
                Sort(ref intervals);
                List<Interval> res = new List<Interval>() { intervals[0] };
                for(int i = 1; i < intervals.Length; ++i)
                {
                    Interval[] tmp = Merge(res[res.Count - 1], intervals[i]);
                    res.Remove(res[res.Count - 1]);
                    for (int j = 0; j < tmp.Length;++j) res.Add(tmp[j]);
                }
                return res.ToArray();
            }

            public static Interval[] Merge(Interval[] A,Interval[] B)
            {
                Sort(ref A);
                Sort(ref B);
                List<Interval> res = new List<Interval>();
                int i = 0, j = 0;
                Interval interval;
                while (i < A.Length && j < B.Length)
                {
                    if (A[i].start > B[j].end)
                    {
                        res.Add(B[j]);
                        res.Add(A[i]);
                        j++;
                    }
                    else if (B[j].start > A[i].end)
                    {
                        res.Add(A[i]);
                        res.Add(B[j]);
                        i++;
                    }
                    else if (B[j].start >= A[i].start)
                    {
                        if (A[i].end > B[j].end)
                        {
                            interval = new Interval(A[i].start, A[i].end);
                            j++;
                        }
                        else
                        {
                            interval = new Interval(A[i].start, B[j].end);
                            i++;
                        }
                        res.Add(interval);
                    }
                    else if (A[i].start >= B[j].start)
                    {
                        if (A[i].end > B[j].end)
                        {
                            interval = new Interval(B[j].start, A[i].end);
                            j++;
                        }
                        else
                        {
                            interval = new Interval(B[j].start, B[j].end);
                            i++;
                        }
                        res.Add(interval);
                    }
                }
                return res.ToArray();
            }
            
            public static Interval[] Merge(List<Interval[]> intervalsList)
            {
                Interval[] res =intervalsList[0];
                for(int i = 1; i < intervalsList.Count; ++i)
                {
                    res = Merge(res, intervalsList[i]);
                }
                return res;
            }

            public static Interval[] Intersection(Interval[] A,Interval[]B)
            {
                List<Interval> res=new List<Interval>();
                int i = 0, j = 0;
                Interval interval;
                while (i<A.Length && j < B.Length)
                {
                    if (A[i].start > B[j].end) j++;
                    else if (B[j].start > A[i].end) i++;
                    else if (B[j].start >= A[i].start)
                    {
                        if (A[i].end > B[j].end)
                        {
                            interval = new Interval(B[j].start, B[j].end);
                            j++;
                        }
                        else
                        {
                            interval = new Interval(B[j].start, A[i].end);
                            i++;
                        }
                        res.Add(interval);
                    }else if (A[i].start >= B[j].start)
                    {
                        if (A[i].end > B[j].end)
                        {
                            interval = new Interval(A[i].start, B[j].end);
                            j++;
                        }
                        else
                        {
                            interval = new Interval(A[i].start, A[i].end);
                            i++;
                        }
                        res.Add(interval);
                    }
                }
                return res.ToArray();
            }

            public static Interval[] GetIntersection(List<Interval[]> intervalsList)
            {
                Interval[] res=intervalsList[0];
                for (int i = 1; i < intervalsList.Count; ++i)
                {
                    res = Intersection(res, intervalsList[i]);
                }
                return res;
            }
            
            public static List<Interval> Intersection(List<Interval> intervalList)
            {
                Sort(intervalList);
                List<Interval> res = new List<Interval>
                {
                    intervalList[0]
                };
                for (int i = 1; i < intervalList.Count; ++i)
                {
                    if (intervalList[i].start > intervalList[res.Count - 1].end) res.Add(intervalList[i]);
                    else res[res.Count - 1] = new Interval(res[res.Count - 1].start, Mathf.Max(res[res.Count - 1].end, intervalList[i].end));
                }
                return res;
            }
            
            /// <summary>
            /// Sort by start
            /// </summary>
            /// <param name="intervalList"></param>
            public static void Sort(List<Interval> intervalList)
            {
                for(int i = 0; i < intervalList.Count-1; ++i)
                {
                    for(int j = i+1; j < intervalList.Count; ++j)
                    {
                        if (intervalList[i].start > intervalList[j].start)
                        {
                            Interval tmp= intervalList[i];
                            intervalList[i] = intervalList[j];
                            intervalList[j] = tmp;
                        }
                    }
                }
            }

            /// <summary>
            /// Swap
            /// </summary>
            /// <param name="a"></param>
            /// <param name="b"></param>
            public static void Swap(ref Interval a,ref Interval b)
            {
                Interval tmp = a;
                a = b;
                b = tmp;
            }

            public static void Swap(Interval[] intervals,int a,int b)
            {
                Interval tmp = intervals[a];
                intervals[a] =intervals[ b];
                intervals[b] = tmp;
            }

            public static int IsValid(Interval[] intervals,ref float val)
            {
                for(int i = 0; i < intervals.Length; ++i)
                {
                    if (val >= intervals[i].start && val <= intervals[i].end) return i;
                }
                return -1;
            }

        }
    }
}
