using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Plankton;
using PlanktonGh;
using Rhino;
using Rhino.Geometry;

namespace MeshMachine
{
    public class Util
    {
        public static List<int> CompactByVertex(PlanktonMesh P, List<int> L)
        {
            List<int> L2 = new List<int>();

            for (int i = 0; i < P.Vertices.Count; i++)
            {
                if (P.Vertices[i].IsUnused == false)
                {
                    L2.Add(L[i]);
                }
            }
            return L2;
        }

        public static List<int> CompactByEdge(PlanktonMesh P, List<int> L1)
        {
            List<int> L2 = new List<int>();

            int EdgeCount = P.Halfedges.Count / 2;

            for (int i = 0; i < EdgeCount; i++)
            {
                if (P.Halfedges[2 * i].IsUnused == false)
                {
                    L2.Add(L1[i]);
                }
            }
            return L2;
        }


        public static Vector3d Normal(PlanktonMesh P, int V)
        {
            Point3d Vertex = P.Vertices[V].ToPoint3d();
            Vector3d Norm = new Vector3d();

            int[] OutEdges = P.Vertices.GetHalfedges(V);
            int[] Neighbours = P.Vertices.GetVertexNeighbours(V);
            Vector3d[] OutVectors = new Vector3d[Neighbours.Length];
            int Valence = P.Vertices.GetValence(V);

            for (int j = 0; j < Valence; j++)
            {
                OutVectors[j] = P.Vertices[Neighbours[j]].ToPoint3d() - Vertex;
            }

            for (int j = 0; j < Valence; j++)
            {
                if (P.Halfedges[OutEdges[(j + 1) % Valence]].AdjacentFace != -1)
                {
                    Norm += (Vector3d.CrossProduct(OutVectors[(j + 1) % Valence], OutVectors[j]));
                }
            }

            Norm.Unitize();
            return Norm;
        }

        public static Point3d MidPt(PlanktonMesh P, int E)
        {
            Point3d Pos1 = P.Vertices[P.Halfedges[2 * E].StartVertex].ToPoint3d();
            Point3d Pos2 = P.Vertices[P.Halfedges[2 * E + 1].StartVertex].ToPoint3d();
            return (Pos1 + Pos2) * 0.5;
        }


        public static Vector3d[] LaplacianSmooth(PlanktonMesh P, int W, double Strength)
        {
            int VertCount = P.Vertices.Count;
            Vector3d[] Smooth = new Vector3d[VertCount];

            for (int i = 0; i < VertCount; i++)
            {
                if ((P.Vertices[i].IsUnused == false) && (P.Vertices.IsBoundary(i) == false))
                {
                    int[] Neighbours = P.Vertices.GetVertexNeighbours(i);
                    Point3d Vertex = P.Vertices[i].ToPoint3d();
                    Point3d Centroid = new Point3d();
                    if (W == 0)
                    {
                        for (int j = 0; j < Neighbours.Length; j++)
                        { Centroid = Centroid + P.Vertices[Neighbours[j]].ToPoint3d(); }
                        Smooth[i] = ((Centroid * (1.0 / P.Vertices.GetValence(i))) - Vertex) * Strength;
                    }
                    if (W == 1)
                    {
                        //get the radial vectors of the 1-ring
                        //get the vectors around the 1-ring
                        //get the cotangent weights for each edge

                        int valence = Neighbours.Length;

                        Point3d[] NeighbourPts = new Point3d[valence];
                        Vector3d[] Radial = new Vector3d[valence];
                        Vector3d[] Around = new Vector3d[valence];
                        double[] CotWeight = new double[valence];
                        double WeightSum = 0;

                        for (int j = 0; j < valence; j++)
                        {
                            NeighbourPts[j] = P.Vertices[Neighbours[j]].ToPoint3d();
                            Radial[j] = NeighbourPts[j] - Vertex;
                        }

                        for (int j = 0; j < valence; j++)
                        {
                            Around[j] = NeighbourPts[(j + 1) % valence] - NeighbourPts[j];
                        }

                        for (int j = 0; j < Neighbours.Length; j++)
                        {
                            //get the cotangent weights
                            int previous = (j + valence - 1) % valence;
                            Vector3d Cross1 = Vector3d.CrossProduct(Radial[previous], Around[previous]);
                            double Cross1Length = Cross1.Length;
                            double Dot1 = Radial[previous] * Around[previous];

                            int next = (j + 1) % valence;
                            Vector3d Cross2 = Vector3d.CrossProduct(Radial[next], Around[j]);
                            double Cross2Length = Cross2.Length;
                            double Dot2 = Radial[next] * Around[j];

                            CotWeight[j] = Math.Abs(Dot1 / Cross1Length) + Math.Abs(Dot2 / Cross2Length);
                            WeightSum += CotWeight[j];
                        }

                        double InvWeightSum = 1.0 / WeightSum;

                        Vector3d ThisSmooth = new Vector3d();

                        for (int j = 0; j < Neighbours.Length; j++)
                        {
                            ThisSmooth = ThisSmooth + Radial[j] * CotWeight[j];
                        }

                        Smooth[i] = ThisSmooth * InvWeightSum * Strength;
                    }

                }
            }
            return Smooth;
        }


    }
    
}
