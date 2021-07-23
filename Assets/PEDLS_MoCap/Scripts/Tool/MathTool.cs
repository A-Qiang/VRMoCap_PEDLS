using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
namespace Mocap
{
    namespace IK
    {
        public class MathTool
        {
            public static void Swap(ref float a, ref float b)
            {
                float tmp = a;
                a = b;
                b = tmp;
            }

            public static double[,] GetParasVe3Rotation(Vector3 n, Vector3 p)
            {
                double[,] paras = new double[3, 3];

                paras[0, 0] = -p.y * n.z + p.z * n.y;
                paras[0, 1] = p.x * (1 - n.x * n.x) - p.y * n.x*n.y - p.z * n.x * n.z;
                paras[0, 2] = p.x * n.x * n.x + p.y * n.x * n.y + p.z * n.x * n.z;

                paras[1, 0] = p.x * n.z - p.z * n.x;
                paras[1, 1] = -p.x * n.x * n.y + p.y * (1 - n.y * n.y) - p.z * n.y * n.z;
                paras[1, 2] = p.x * n.y * n.x + p.y * n.y * n.y + p.z * n.y * n.z;

                paras[2, 0] = -p.x * n.y + p.y * n.x;
                paras[2, 1] = -p.x * n.x * n.z - p.y * n.z * n.y + p.z * (1 - n.z * n.z);
                paras[2, 2] = p.x * n.x * n.z + p.y * n.y * n.z + p.z * n.z * n.z;

                return paras;
            }

            /// <summary>
            /// mat(T)*mat
            /// </summary>
            /// <param name="mat"></param>
            /// <returns></returns>
            public static double[,] Matrix3X3MultiplySelf(double[,] mat)
            {
                double[,] res = new double[3, 3];
                for(int i = 0; i < 3; ++i)
                {
                    for(int j = i; j < 3; ++j)
                    {
                        res[i, j] = mat[0, i] * mat[0, j] + mat[1, i] * mat[1, j] + mat[2, i] * mat[2, j];
                        if (i != j) res[j, i] = res[i, j];
                    }
                }
                return res;
            }

            public static int SolveQuartic(double a,double b,double c,double d,double e,ref double x1,ref double x2,ref double x3,ref double x4)
            {
                double A, B, C, D, E, F,delta,deltaMod;
                D = 3 * b * b - 8 * a * c;
                E = -b * b * b + 4 * a * b * c - 8 * a * a * d;
                F = 3 * b * b * b * b + 16 * a * a * c * c - 16 * a * b * b * c + 16 * a * a * b * d - 64 * a * a * a * e;
                A = D * D - 3 * F;
                B = D * F - 9 * E * E;
                C = F * F - 3 * D * E * E;
                delta = B * B - 4 * A * C;
                deltaMod = Math.Abs(delta);
                int sign = Math.Sign(E);

                //1
                if (D == 0 && E == 0 && F == 0)
                {
                    x1 = x2 = x3 = x4 = -4 * e / d;
                    return 1;
                }

                //2
                if (D!=0&&E!=0&&F!=0&&A == 0 && B == 0 && C == 0)
                {
                    x1 = (-b * D + 9 * E) / 4 * a * D;
                    x2 = x3 = x4 = (-b * D - 3 * E) / 4 * a * D;
                    return 2;
                }

                //3
                if (E == 0 && F == 0 && D != 0)
                {
                    if (D > 0)
                    {
                        double tmp = Math.Sqrt(D);
                        x1 = x2 = (-b + tmp) / (4 * a);
                        x3 = x4 = (-b - tmp) / (4 * a);
                        return 2;
                    }
                    else
                    {
                        return 0;
                    }
                }

                //4
                if (A != 0 && B != 0 && C != 0 && (delta == 0||deltaMod<Mathf.Epsilon))
                {
                    double tmp = 2 * B / A;
                    if (tmp > 0)
                    {
                        tmp = Math.Sqrt(tmp);
                        x1 = (-b + 2 * A * E / B + tmp) / (4 * a);
                        x2 = (-b + 2 * A * E / B - tmp) / (4 * a);
                    }
                    else
                    {
                        return 2;
                    }
                    x3 =x4= (-b - 2 * A * E / B) / (4 * a);
                    return 3;
                }

                //5
                if (delta > 0&&deltaMod>Mathf.Epsilon)
                {
                    double z,z1, z2;
                    z1 = A * D + 3 * (-B + Math.Sqrt(delta)) / 2.0;
                    z2 = A * D + 3 * (-B - Math.Sqrt(delta)) / 2.0;
                    z1 = Math.Sign(z1) * Math.Pow(Math.Sign(z1) * z1, 1.0 / 3);
                    z2 = Math.Sign(z2) * Math.Pow(Math.Sign(z2) * z2, 1.0 / 3);

                    z = Math.Sqrt(D * D - D * (z1 + z2) + (z1 + z2) * (z1 + z2) - 3 * A);
                    
                    double tmp = Math.Sqrt((2 * D - (z1 + z2) + 2 * z) / 3.0);
                    
                    x1 = (-b + sign * Math.Sqrt((D + z1 + z2) / 3) + tmp) / (4*a);
                    x2 = (-b + sign * Math.Sqrt((D + z1 + z2) / 3) - tmp) / (4*a);
                    return 2;
                }

                //6
                if (delta < 0&&deltaMod>Mathf.Epsilon)
                {
                    if ( D > 0 &&F > 0&& E == 0)
                    {
                        x1 = (-b + Math.Sqrt(D + 2 * Math.Sqrt(F))) / (4 * a);
                        x2 = (-b - Math.Sqrt(D + 2 * Math.Sqrt(F))) / (4 * a);
                        x3 = (-b + Math.Sqrt(D - 2 * Math.Sqrt(F))) / (4 * a);
                        x4 = (-b - Math.Sqrt(D - 2 * Math.Sqrt(F))) / (4 * a);
                        return 4;
                    }
                    if (D > 0 && F > 0&&E!=0)
                    {
                        double rootA = Math.Sqrt(A);
                        double theta = Math.Acos((3 * B - 2 * A * D) / (2 * A * rootA));
                        double sin = Math.Sin(theta / 3.0);
                        double cos = Math.Cos(theta / 3.0);
                        double y1 = Math.Sqrt((D - 2 * rootA * cos) / 3);
                        double y2 = Math.Sqrt((D + rootA * (cos + Math.Sqrt(3) * sin)) / 3);
                        double y3 = Math.Sqrt((D + rootA * (cos - Math.Sqrt(3) * sin)) / 3);

                        x1 = (-b + sign * y1 + (y2 + y3)) / 4 * a;
                        x2 = (-b + sign * y1 - (y2 + y3)) / 4 * a;
                        x3 = (-b + sign * y1 + (y2 - y3)) / 4 * a;
                        x4 = (-b + sign * y1 - (y2 - y3)) / 4 * a;
                        return 4;

                    }
                    else
                    {
                        return 0;
                    }
                }
                return 0;
            }

            private static double DistanceMetric(double[,] hs, double[,] hd, double angle, float smoothWeight, float heuristicWeight)
            {
                double[] ved = new double[3];
                double[] ves = new double[3];
                for (int i = 0; i < 3; ++i)
                {
                    ves[i] = hs[i, 0] * Math.Sin(angle) + hs[i, 1] * Math.Cos(angle) + hs[i, 2];
                    ved[i] = hd[i, 0] * Math.Sin(angle) + hd[i, 1] * Math.Cos(angle) + hd[i, 2];
                }
                return smoothWeight * (ves[0] * ves[0] + ves[1] * ves[1] + ves[2] * ves[2]) + heuristicWeight * (ved[0] * ved[0] + ved[1] * ved[1] + ved[2] * ved[2]);
            }

            private static double DistanceMetric(double[,] hs, double[,] hd, double[,] hi, double angle, float smoothWeight, float heuristicWeight,float initWeight)
            {
                double[] ved = new double[3];
                double[] ves = new double[3];
                double[] vei = new double[3];
                for (int i = 0; i < 3; ++i)
                {
                    ves[i] = hs[i, 0] * Math.Sin(angle) + hs[i, 1] * Math.Cos(angle) + hs[i, 2];
                    ved[i] = hd[i, 0] * Math.Sin(angle) + hd[i, 1] * Math.Cos(angle) + hd[i, 2];
                    vei[i] = hi[i, 0] * Math.Sin(angle) + hi[i, 1] * Math.Cos(angle) + hi[i, 2];
                }
                return smoothWeight * (ves[0] * ves[0] + ves[1] * ves[1] + ves[2] * ves[2]) + heuristicWeight * (ved[0] * ved[0] + ved[1] * ved[1] + ved[2] * ved[2]) + initWeight * (vei[0] * vei[0] + vei[1] * vei[1] + vei[2] * vei[2]);
            }

            public static float LMOptimize(Circle swivel, Vector3 zero, Vector3 p1, Vector3 p2, Vector3 pd, Vector3 initPos, float smoothWeight, float heuristicWeight, float initWeight)
            {
                Vector3 ps = 2 * p1 - p2;
                double[,] paras, parasSmooth, hs, parasDes, hd, parasInit, hi;
                double a, b, c, d, e, f;
                double A, B, C, D, E;
                double x1 = 0, x2 = 0, x3 = 0, x4 = 0;
                double[] angle = new double[4];
                parasSmooth = new double[3, 3];
                parasDes = new double[3, 3];
                parasInit = new double[3, 3];
                
                paras = MathTool.GetParasVe3Rotation(swivel.n, zero - swivel.c);
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 2; ++j)
                    {
                        parasSmooth[i, j] = paras[i, j];
                        parasDes[i, j] = paras[i, j];
                        parasInit[i, j] = paras[i, j];
                    }
                    parasSmooth[i, 2] = paras[i, 2] - ps[i] + swivel.c[i];
                    parasDes[i, 2] = paras[i, 2] - pd[i] + swivel.c[i];
                    parasInit[i, 2] = paras[i, 2] - initPos[i] + swivel.c[i];
                }
                
                hs = MathTool.Matrix3X3MultiplySelf(parasSmooth);
                hd = MathTool.Matrix3X3MultiplySelf(parasDes);
                hi = MathTool.Matrix3X3MultiplySelf(parasInit);

                a = -smoothWeight * hs[1, 0] - heuristicWeight * hd[1, 0] - initWeight * hi[1, 0];
                b = smoothWeight * hs[0, 1] + heuristicWeight * hd[0, 1] + initWeight * hi[0, 1];
                c = smoothWeight * (hs[0, 0] - hs[1, 1]) + heuristicWeight * (hd[0, 0] - hd[1, 1]) + initWeight * (hi[0, 0] - hi[1, 1]);
                d = -smoothWeight * hs[1, 2] - heuristicWeight * hd[1, 2] - initWeight * hi[1, 2];
                e = smoothWeight * hs[0, 2] + heuristicWeight * hd[0, 2] + initWeight * hi[0, 2];
                f = 0;
                
                A = b - e + f;
                B = -2 * c + 2 * d;
                C = 4 * a - 2 * b + 2 * f;
                D = 2 * c + 2 * d;
                E = b + e + f;
                
                MathTool.SolveQuartic(A, B, C, D, E, ref x1, ref x2, ref x3, ref x4);
                angle[0] = 2 * Math.Atan(x1);
                angle[1] = 2 * Math.Atan(x2);
                angle[2] = 2 * Math.Atan(x3);
                angle[3] = 2 * Math.Atan(x4);

                int index = 0;
                double min = DistanceMetric(parasSmooth, parasDes, parasInit, angle[0],smoothWeight,heuristicWeight,initWeight);
                for (int i = 1; i < 4; ++i)
                {
                    double res = DistanceMetric(parasSmooth, parasDes, parasInit, angle[i], smoothWeight, heuristicWeight, initWeight);
                    if (res < min)
                    {
                        min = res;
                        index = i;
                    }
                }
                return Mathf.Rad2Deg * (float)angle[index];
            }

            public static float LMOptimize(Circle swivel, Vector3 zero, Vector3 p1, Vector3 p2, Vector3 pd, float smoothWeight, float heuristicWeight)
            {
                Vector3 ps = 2 * p1 - p2;
                double[,] paras, parasSmooth, hs, parasDes, hd;
                double a, b, c, d, e, f;
                double A, B, C, D, E;
                double x1 = 0, x2 = 0, x3 = 0, x4 = 0;
                double[] angle = new double[4];
                parasSmooth = new double[3, 3];
                parasDes = new double[3, 3];
                
                paras = MathTool.GetParasVe3Rotation(swivel.n, zero - swivel.c);
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 2; ++j)
                    {
                        parasSmooth[i, j] = paras[i, j];
                        parasDes[i, j] = paras[i, j];
                    }
                    parasSmooth[i, 2] = paras[i, 2] - ps[i] + swivel.c[i];
                    parasDes[i, 2] = paras[i, 2] - pd[i] + swivel.c[i];
                }
                
                hs = MathTool.Matrix3X3MultiplySelf(parasSmooth);
                hd = MathTool.Matrix3X3MultiplySelf(parasDes);

                a = -smoothWeight * hs[1, 0] - heuristicWeight * hd[1, 0] ;
                b = smoothWeight * hs[0, 1] + heuristicWeight * hd[0, 1] ;
                c = smoothWeight * (hs[0, 0] - hs[1, 1]) + heuristicWeight * (hd[0, 0] - hd[1, 1]) ;
                d = -smoothWeight * hs[1, 2] - heuristicWeight * hd[1, 2] ;
                e = smoothWeight * hs[0, 2] + heuristicWeight * hd[0, 2] ;
                f = 0;
                
                A = b - e + f;
                B = -2 * c + 2 * d;
                C = 4 * a - 2 * b + 2 * f;
                D = 2 * c + 2 * d;
                E = b + e + f;
                
                MathTool.SolveQuartic(A, B, C, D, E, ref x1, ref x2, ref x3, ref x4);
                angle[0] = 2 * Math.Atan(x1);
                angle[1] = 2 * Math.Atan(x2);
                angle[2] = 2 * Math.Atan(x3);
                angle[3] = 2 * Math.Atan(x4);

                int index = 0;
                double min = DistanceMetric(parasSmooth, parasDes, angle[0], smoothWeight, heuristicWeight);
                for (int i = 1; i < 4; ++i)
                {
                    double res = DistanceMetric(parasSmooth, parasDes, angle[i], smoothWeight, heuristicWeight);
                    if (res < min)
                    {
                        min = res;
                        index = i;
                    }
                }
                return Mathf.Rad2Deg * (float)angle[index];
            }

        }
    }
}
