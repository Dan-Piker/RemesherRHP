using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Commands;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using Plankton;
using PlanktonGh;

namespace MeshMachine
{
    [System.Runtime.InteropServices.Guid("f4ffab0c-3a53-4f76-8811-424793bf5424")]
    public class MeshMachineCommand : Command
    {
        public MeshMachineCommand()
        {
            // Rhino only creates one instance of each command class defined in a
            // plug-in, so it is safe to store a refence in a static property.
            Instance = this;
        }

        ///<summary>The only instance of this command.</summary>
        public static MeshMachineCommand Instance
        {
            get;
            private set;
        }

        ///<returns>The command name as it appears on the Rhino command line.</returns>
        public override string EnglishName
        {
            get { return "MMTriangles"; }
        }

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            // TODO: start here modifying the behaviour of your command.
            // ---
            RhinoApp.WriteLine("MeshMachine WIP test", EnglishName);

            Brep SB;
            using (GetObject getBrep = new GetObject())
            {
                getBrep.SetCommandPrompt("Please select the brep to remesh");
                getBrep.Get();
                SB = getBrep.Object(0).Brep();
            }
           
            //GetNumber TargetLength = new GetNumber();  
            //double L = TargetLength.Number();

            Rhino.Input.Custom.GetNumber gn = new Rhino.Input.Custom.GetNumber();
            gn.SetCommandPrompt("Specify a Target Edge Length");
            gn.SetLowerLimit(0.5, false);
            gn.Get();
            if (gn.CommandResult() != Rhino.Commands.Result.Success)
                return gn.CommandResult();
            double L = gn.Number();

            //Point3d pt0;
            //using (GetPoint getPointAction = new GetPoint())
            //{
            //    getPointAction.SetCommandPrompt("Please select the start point");
            //    if (getPointAction.Get() != GetResult.Point)
            //    {
            //        RhinoApp.WriteLine("No start point was selected.");
            //        return getPointAction.CommandResult();
            //    }
            //    pt0 = getPointAction.Point();
            //}

            //Point3d pt1;
            //using (GetPoint getPointAction = new GetPoint())
            //{
            //    getPointAction.SetCommandPrompt("Please select the end point");
            //    getPointAction.SetBasePoint(pt0, true);
            //    getPointAction.DynamicDraw +=
            //      (sender, e) => e.Display.DrawLine(pt0, e.CurrentPoint, System.Drawing.Color.DarkRed);
            //    if (getPointAction.Get() != GetResult.Point)
            //    {
            //        RhinoApp.WriteLine("No end point was selected.");
            //        return getPointAction.CommandResult();
            //    }
            //    pt1 = getPointAction.Point();
            //}

            //doc.Objects.AddLine(pt0, pt1);

            PlanktonMesh P = new PlanktonMesh();
            
            List<int> AnchorV = new List<int>();
            List<int> FeatureV = new List<int>();
            List<int> FeatureE = new List<int>();
            double FixT = 0.00001;
            double LengthTol = 0.15;     //a tolerance for when to split/collapse edges
            double SmoothStrength = 0.8;  //smoothing strength
            double PullStrength = 0.8;  //pull to target mesh strength         
            double CurvDep = 0;
            int Flip = 1;

            MeshingParameters MeshParams = new MeshingParameters();
            MeshParams.MaximumEdgeLength = 3 * L;
            MeshParams.MinimumEdgeLength = L;
            MeshParams.JaggedSeams = false;
            MeshParams.SimplePlanes = false;            
            
            Mesh[] BrepMeshes = Mesh.CreateFromBrep(SB, MeshParams);
            Mesh M = new Mesh();
            foreach (var mesh in BrepMeshes)
                M.Append(mesh);

            M.Faces.ConvertQuadsToTriangles();
            P = M.ToPlanktonMesh();

            var FC = new List<Curve>();
            foreach (BrepEdge E in SB.Edges)
            {
                if (!E.IsSmoothManifoldEdge(0.01))
                { FC.Add(E.ToNurbsCurve()); }
            }
            var Corners = SB.Vertices;
            List<Point3d> FV = new List<Point3d>();
            foreach(Point Pt in Corners)
            {
                FV.Add(Pt.Location);
            }

            //Mark any vertices or edges lying on features
            for (int i = 0; i < P.Vertices.Count; i++)
            {
                Point3d Pt = P.Vertices[i].ToPoint3d();
                AnchorV.Add(-1);
                for (int j = 0; j < FV.Count; j++)
                {
                    if (Pt.DistanceTo(FV[j]) < FixT)
                    { AnchorV[AnchorV.Count - 1] = j; }
                }

                FeatureV.Add(-1);
                for (int j = 0; j < FC.Count; j++)
                {
                    double param = new double();
                    FC[j].ClosestPoint(Pt, out param);
                    if (Pt.DistanceTo(FC[j].PointAt(param)) < FixT)
                    { FeatureV[FeatureV.Count - 1] = j; }
                }
            }

            int EdgeCount = P.Halfedges.Count / 2;
            for (int i = 0; i < EdgeCount; i++)
            {
                FeatureE.Add(-1);
                int vStart = P.Halfedges[2 * i].StartVertex;
                int vEnd = P.Halfedges[2 * i + 1].StartVertex;

                Point3d PStart = P.Vertices[vStart].ToPoint3d();
                Point3d PEnd = P.Vertices[vEnd].ToPoint3d();

                for (int j = 0; j < FC.Count; j++)
                {
                    double paramS = new double();
                    double paramE = new double();
                    Curve thisFC = FC[j];
                    thisFC.ClosestPoint(PStart, out paramS);
                    thisFC.ClosestPoint(PEnd, out paramE);
                    if ((PStart.DistanceTo(thisFC.PointAt(paramS)) < FixT) &&
                      (PEnd.DistanceTo(thisFC.PointAt(paramE)) < FixT))
                    {
                        FeatureE[FeatureE.Count - 1] = j;
                    }
                }
            }



            for (int iter = 0; iter < 30; iter++)
            {
                EdgeCount = P.Halfedges.Count / 2;
                double[] EdgeLength = P.Halfedges.GetLengths();
                List<bool> Visited = new List<bool>();
                Vector3d[] Normals = new Vector3d[P.Vertices.Count];

                for (int i = 0; i < P.Vertices.Count; i++)
                {
                    Visited.Add(false);
                    Normals[i] = Util.Normal(P, i);
                }

                double t = LengthTol;     //a tolerance for when to split/collapse edges
                double smooth = SmoothStrength;  //smoothing strength
                double pull = PullStrength;  //pull to target mesh strength               

                //    Split the edges that are too long
                for (int i = 0; i < EdgeCount; i++)
                {
                    if (P.Halfedges[2 * i].IsUnused == false)
                    {
                        int vStart = P.Halfedges[2 * i].StartVertex;
                        int vEnd = P.Halfedges[2 * i + 1].StartVertex;

                        if ((Visited[vStart] == false)
                          && (Visited[vEnd] == false))
                        {
                            double L2 = L;
                            Point3d Mid = Util.MidPt(P, i);

                            //if (CurvDep > 0)
                            //{
                            //    double NormDiff = Vector3d.VectorAngle(Normals[vStart], Normals[vEnd]);
                            //    L2 = Math.Min((1.0 / (3.0 * NormDiff) * L), 5 * L);

                            //    if (CurvDep != 1)
                            //    {
                            //        L2 = L2 * (CurvDep) + L * (1.0 - CurvDep);
                            //    }

                            //}
                            //if (BoundScale != 1.0)
                            //{
                            //    double MinDist = 99954;

                            //    for (int j = 0; j < FC.Count; j++)
                            //    {
                            //        double param = new double();

                            //        FC[j].ClosestPoint(Mid, out param);
                            //        double ThisDist = Mid.DistanceTo(FC[j].PointAt(param));
                            //        if (ThisDist < MinDist)
                            //        { MinDist = ThisDist; }
                            //    }

                            //    if (MinDist < BoundDist)
                            //    {
                            //        L2 = L2 * BoundScale + (MinDist / BoundDist) * (L2 * (1 - BoundScale));
                            //    }
                            //}

                            //if (SizP.Count > 0)
                            //{
                            //    L2 = WeightedCombo(Mid, SizP, SizV, WExp, L2, BGW);
                            //    //  L2 = (WL * (1.0 - BGW)) + (BGW * L2);
                            //}

                            if (EdgeLength[2 * i] > (1 + t) * (4f / 3f) * L2)
                            {

                                int SplitHEdge = P.Halfedges.TriangleSplitEdge(2 * i);
                                if (SplitHEdge != -1)
                                {
                                    int SplitCenter = P.Halfedges[SplitHEdge].StartVertex;
                                    P.Vertices.SetVertex(SplitCenter, Util.MidPt(P, i));

                                    //update the feature information
                                    FeatureE.Add(FeatureE[i]);
                                    FeatureV.Add(FeatureE[i]);
                                    AnchorV.Add(-1);

                                    //2 additional new edges have also been created (or 1 if split was on a boundary)
                                    //mark these as non-features
                                    int CEdgeCount = P.Halfedges.Count / 2;
                                    while (FeatureE.Count < CEdgeCount)
                                    { FeatureE.Add(-1); }

                                    Visited.Add(true);
                                    int[] Neighbours = P.Vertices.GetVertexNeighbours(SplitCenter);
                                    foreach (int n in Neighbours)
                                    { Visited[n] = true; }
                                }
                            }

                        }
                    }
                }

                //Collapse the edges that are too short
                for (int i = 0; i < EdgeCount; i++)
                {
                    if (P.Halfedges[2 * i].IsUnused == false)
                    {
                        int vStart = P.Halfedges[2 * i].StartVertex;
                        int vEnd = P.Halfedges[2 * i + 1].StartVertex;
                        if ((Visited[vStart] == false)
                          && (Visited[vEnd] == false))
                        {
                            if (!(AnchorV[vStart] != -1 && AnchorV[vEnd] != -1)) // if both ends are anchored, don't collapse
                            {
                                int Collapse_option = 0; //0 for none, 1 for collapse to midpt, 2 for towards start, 3 for towards end
                                //if neither are anchorV
                                if (AnchorV[vStart] == -1 && AnchorV[vEnd] == -1)
                                {
                                    // if both on same feature (or neither on a feature)
                                    if (FeatureV[vStart] == FeatureV[vEnd])
                                    { Collapse_option = 1; }
                                    // if start is on a feature and end isn't
                                    if ((FeatureV[vStart] != -1) && (FeatureV[vEnd] == -1))
                                    { Collapse_option = 2; }
                                    // if end is on a feature and start isn't
                                    if ((FeatureV[vStart] == -1) && (FeatureV[vEnd] != -1))
                                    { Collapse_option = 3; }
                                }
                                else // so one end must be an anchor
                                {
                                    // if start is an anchor
                                    if (AnchorV[vStart] != -1)
                                    {
                                        // if both are on same feature, or if the end is not a feature
                                        if ((FeatureE[i] != -1) || (FeatureV[vEnd] == -1))
                                        { Collapse_option = 2; }
                                    }
                                    // if end is an anchor
                                    if (AnchorV[vEnd] != -1)
                                    {
                                        // if both are on same feature, or if the start is not a feature
                                        if ((FeatureE[i] != -1) || (FeatureV[vStart] == -1))
                                        { Collapse_option = 3; }
                                    }
                                }

                                double L2 = L;
                                Point3d Mid = Util.MidPt(P, i);

                                if (CurvDep > 0)
                                {
                                    double NormDiff = Vector3d.VectorAngle(Normals[vStart], Normals[vEnd]);
                                    L2 = Math.Min((1.0 / (3.0 * NormDiff) * L), 5 * L);

                                    if (CurvDep != 1)
                                    {
                                        L2 = L2 * (CurvDep) + L * (1.0 - CurvDep);
                                    }
                                }

                                //if (BoundScale != 1.0)
                                //{
                                //    double MinDist = 99954;

                                //    for (int j = 0; j < FC.Count; j++)
                                //    {
                                //        double param = new double();

                                //        FC[j].ClosestPoint(Mid, out param);
                                //        double ThisDist = Mid.DistanceTo(FC[j].PointAt(param));
                                //        if (ThisDist < MinDist)
                                //        { MinDist = ThisDist; }
                                //    }

                                //    if (MinDist < BoundDist)
                                //    {
                                //        L2 = L2 * BoundScale + (MinDist / BoundDist) * (L2 * (1 - BoundScale));
                                //    }
                                //}

                                //if (SizP.Count > 0)
                                //{
                                //    L2 = WeightedCombo(Mid, SizP, SizV, WExp, L2, BGW);
                                //    //double WL = WeightedCombo(Mid, SizP, SizV, WExp);
                                //    //L2 = (WL * (1.0 - BGW)) + (BGW * L2);
                                //}

                                if ((Collapse_option != 0) && (EdgeLength[2 * i] < (1 - t) * 4f / 5f * L2))
                                {
                                    int Collapsed = -1;
                                    int CollapseRtn = -1;
                                    if (Collapse_option == 1)
                                    {
                                        Collapsed = P.Halfedges[2 * i].StartVertex;
                                        P.Vertices.SetVertex(Collapsed, Util.MidPt(P, i));
                                        CollapseRtn = P.Halfedges.CollapseEdge(2 * i);
                                    }
                                    if (Collapse_option == 2)
                                    {
                                        Collapsed = P.Halfedges[2 * i].StartVertex;
                                        CollapseRtn = P.Halfedges.CollapseEdge(2 * i);
                                    }
                                    if (Collapse_option == 3)
                                    {
                                        Collapsed = P.Halfedges[2 * i + 1].StartVertex;
                                        CollapseRtn = P.Halfedges.CollapseEdge(2 * i + 1);
                                    }
                                    if (CollapseRtn != -1)
                                    {
                                        int[] Neighbours = P.Vertices.GetVertexNeighbours(Collapsed);
                                        foreach (int n in Neighbours)
                                        { Visited[n] = true; }
                                    }
                                }
                            }
                        }
                    }
                }

                EdgeCount = P.Halfedges.Count / 2;

                if ((Flip == 0) && (PullStrength > 0))
                {
                    //Flip edges to reduce valence error
                    for (int i = 0; i < EdgeCount; i++)
                    {
                        if (!P.Halfedges[2 * i].IsUnused
                          && (P.Halfedges[2 * i].AdjacentFace != -1)
                          && (P.Halfedges[2 * i + 1].AdjacentFace != -1)
                          && (FeatureE[i] == -1)  // don't flip feature edges
                          )
                        {
                            int Vert1 = P.Halfedges[2 * i].StartVertex;
                            int Vert2 = P.Halfedges[2 * i + 1].StartVertex;
                            int Vert3 = P.Halfedges[P.Halfedges[P.Halfedges[2 * i].NextHalfedge].NextHalfedge].StartVertex;
                            int Vert4 = P.Halfedges[P.Halfedges[P.Halfedges[2 * i + 1].NextHalfedge].NextHalfedge].StartVertex;

                            int Valence1 = P.Vertices.GetValence(Vert1);
                            int Valence2 = P.Vertices.GetValence(Vert2);
                            int Valence3 = P.Vertices.GetValence(Vert3);
                            int Valence4 = P.Vertices.GetValence(Vert4);

                            if (P.Vertices.NakedEdgeCount(Vert1) > 0) { Valence1 += 2; }
                            if (P.Vertices.NakedEdgeCount(Vert2) > 0) { Valence2 += 2; }
                            if (P.Vertices.NakedEdgeCount(Vert3) > 0) { Valence3 += 2; }
                            if (P.Vertices.NakedEdgeCount(Vert4) > 0) { Valence4 += 2; }

                            int CurrentError =
                              Math.Abs(Valence1 - 6) +
                              Math.Abs(Valence2 - 6) +
                              Math.Abs(Valence3 - 6) +
                              Math.Abs(Valence4 - 6);
                            int FlippedError =
                              Math.Abs(Valence1 - 7) +
                              Math.Abs(Valence2 - 7) +
                              Math.Abs(Valence3 - 5) +
                              Math.Abs(Valence4 - 5);
                            if (CurrentError > FlippedError)
                            {
                                P.Halfedges.FlipEdge(2 * i);
                            }
                        }
                    }
                }
                else
                {
                    //Flip edges based on angle
                    for (int i = 0; i < EdgeCount; i++)
                    {
                        if (!P.Halfedges[2 * i].IsUnused
                          && (P.Halfedges[2 * i].AdjacentFace != -1)
                          && (P.Halfedges[2 * i + 1].AdjacentFace != -1)
                          && (FeatureE[i] == -1) // don't flip feature edges
                          )
                        {
                            int Vert1 = P.Halfedges[2 * i].StartVertex;
                            int Vert2 = P.Halfedges[2 * i + 1].StartVertex;
                            int Vert3 = P.Halfedges[P.Halfedges[P.Halfedges[2 * i].NextHalfedge].NextHalfedge].StartVertex;
                            int Vert4 = P.Halfedges[P.Halfedges[P.Halfedges[2 * i + 1].NextHalfedge].NextHalfedge].StartVertex;

                            Point3d P1 = P.Vertices[Vert1].ToPoint3d();
                            Point3d P2 = P.Vertices[Vert2].ToPoint3d();
                            Point3d P3 = P.Vertices[Vert3].ToPoint3d();
                            Point3d P4 = P.Vertices[Vert4].ToPoint3d();

                            double A1 = Vector3d.VectorAngle(new Vector3d(P3 - P1), new Vector3d(P4 - P1))
                              + Vector3d.VectorAngle(new Vector3d(P4 - P2), new Vector3d(P3 - P2));

                            double A2 = Vector3d.VectorAngle(new Vector3d(P1 - P4), new Vector3d(P2 - P4))
                              + Vector3d.VectorAngle(new Vector3d(P2 - P3), new Vector3d(P1 - P3));

                            if (A2 > A1)
                            {
                                P.Halfedges.FlipEdge(2 * i);
                            }
                        }
                    }
                }

                //if (Minim)
                //{
                //    Vector3d[] SmoothC = LaplacianSmooth(P, 1, smooth);

                //    for (int i = 0; i < P.Vertices.Count; i++)
                //    {
                //        if (AnchorV[i] == -1) // don't smooth feature vertices
                //        {
                //            P.Vertices.MoveVertex(i, 0.5 * SmoothC[i]);
                //        }
                //    }
                //}

                Vector3d[] Smooth = Util.LaplacianSmooth(P, 0, smooth);

                for (int i = 0; i < P.Vertices.Count; i++)
                {
                    if (AnchorV[i] == -1) // don't smooth feature vertices
                    {
                        // make it tangential only
                        Vector3d VNormal = Util.Normal(P, i);
                        double ProjLength = Smooth[i] * VNormal;
                        Smooth[i] = Smooth[i] - (VNormal * ProjLength);

                        P.Vertices.MoveVertex(i, Smooth[i]);

                        if (P.Vertices.NakedEdgeCount(i) != 0)//special smoothing for feature edges
                        {
                            int[] Neighbours = P.Vertices.GetVertexNeighbours(i);
                            int ncount = 0;
                            Point3d Avg = new Point3d();

                            for (int j = 0; j < Neighbours.Length; j++)
                            {
                                if (P.Vertices.NakedEdgeCount(Neighbours[j]) != 0)
                                {
                                    ncount++;
                                    Avg = Avg + P.Vertices[Neighbours[j]].ToPoint3d();
                                }
                            }
                            Avg = Avg * (1.0 / ncount);
                            Vector3d move = Avg - P.Vertices[i].ToPoint3d();
                            move = move * smooth;
                            P.Vertices.MoveVertex(i, move);
                        }

                        if (FeatureV[i] != -1)//special smoothing for feature edges
                        {
                            int[] Neighbours = P.Vertices.GetVertexNeighbours(i);
                            int ncount = 0;
                            Point3d Avg = new Point3d();

                            for (int j = 0; j < Neighbours.Length; j++)
                            {
                                if ((FeatureV[Neighbours[j]] == FeatureV[i]) || (AnchorV[Neighbours[j]] != -1))
                                {
                                    ncount++;
                                    Avg = Avg + P.Vertices[Neighbours[j]].ToPoint3d();
                                }
                            }
                            Avg = Avg * (1.0 / ncount);
                            Vector3d move = Avg - P.Vertices[i].ToPoint3d();
                            move = move * smooth;
                            P.Vertices.MoveVertex(i, move);
                        }

                        //projecting points onto the target along their normals

                        if (pull > 0)
                        {
                            Point3d Point = P.Vertices[i].ToPoint3d();
                            Vector3d normal = Util.Normal(P, i);
                            Ray3d Ray1 = new Ray3d(Point, normal);
                            Ray3d Ray2 = new Ray3d(Point, -normal);
                            double RayPt1 = Rhino.Geometry.Intersect.Intersection.MeshRay(M, Ray1);
                            double RayPt2 = Rhino.Geometry.Intersect.Intersection.MeshRay(M, Ray2);
                            Point3d ProjectedPt;

                            if ((RayPt1 < RayPt2) && (RayPt1 > 0) && (RayPt1 < 1.0))
                            {
                                ProjectedPt = Point * (1 - pull) + pull * Ray1.PointAt(RayPt1);
                            }
                            else if ((RayPt2 < RayPt1) && (RayPt2 > 0) && (RayPt2 < 1.0))
                            {
                                ProjectedPt = Point * (1 - pull) + pull * Ray2.PointAt(RayPt2);
                            }
                            else
                            {
                                ProjectedPt = Point * (1 - pull) + pull * M.ClosestPoint(Point);
                            }

                            P.Vertices.SetVertex(i, ProjectedPt);
                        }


                        if (FeatureV[i] != -1) //pull feature vertices onto feature curves
                        {
                            Point3d Point = P.Vertices[i].ToPoint3d();
                            Curve CF = FC[FeatureV[i]];
                            double param1 = 0.0;
                            Point3d onFeature = new Point3d();
                            CF.ClosestPoint(Point, out param1);
                            onFeature = CF.PointAt(param1);
                            P.Vertices.SetVertex(i, onFeature);
                        }
                    }
                    else
                    {
                        P.Vertices.SetVertex(i, FV[AnchorV[i]]); //pull anchor vertices onto their points
                    }
                }




            



                AnchorV = Util.CompactByVertex(P, AnchorV); //compact the fixed points along with the vertices
                FeatureV = Util.CompactByVertex(P, FeatureV);
                FeatureE = Util.CompactByEdge(P, FeatureE);

                P.Compact(); //this cleans the mesh data structure of unused elements
            }


            Mesh MR = P.ToRhinoMesh();
            MR.Unweld(0.4,true);
          
            doc.Objects.AddMesh(MR);

            doc.Views.Redraw();
            RhinoApp.WriteLine("The {0} command added one mesh to the document.", EnglishName);

            // ---

            return Result.Success;
        }
    }

   
}

       
     