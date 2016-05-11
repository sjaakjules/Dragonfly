using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;

using Threaded.Support;
using CustomExtensions;

using Grasshopper.Kernel;
using Rhino.Geometry;

using System.Diagnostics;
using System.IO;

namespace Threaded
{
    class AsyncCopter : GenericWorker
    {
        public static bool hasLoaded = false;
        bool shouldStop = false;

        IGH_DataAccess DA;

        List<Point3d> points;

        double compleationDistance;

        // Quadcopter Variables
        double mass = 5.5;                          // Kg
        double maxSpeed = 48.0/3.6;                 // m/s
        double maxTilt = (50.0 * Math.PI / 180);    // radians 

        // Copter Physics variables
        Vector3d pos;
        Vector3d lastPos;
        Vector3d lastVel;
        Vector3d veclocityVec;
        Vector3d totalAcceleration;
        Vector3d thrust;
        Vector3d gravity = new Vector3d(0, 0, -9.8);
        double desiredSpeed = 0.1;                  // m/s
        double lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;
        readonly int solverSpeed = 10;                        // ms
        double timeStep = .01;

        Point3d currentPoint;

        Polyline posHistory = new Polyline();
        readonly double segmentLength = 0.2;                   // m

        Polyline rope = new Polyline();
        double ropeDesity = 0.3;                    // kg/m
        double totalRopeLength = 0;
        double springConstant = 3000;               // rope is 1300 N/m
        // wire rope 5.345*10^7
        
        AsyncRope ropeSim;
        List<pointMass> ropePointMass = new List<pointMass>();
        List<double> ropeRestLengths = new List<double>();
        double ropeSolverSpeed = 0.001;
        Curve[] obstacles;


        public AsyncCopter(List<Point3d> _points, double _compleationDistance, double _speed,double _ropeDensity, double _springConstant,double _ropeSolverSpeed,Curve[] _obstacles, ref IGH_DataAccess _DA)
            : base()
        {
            DA = _DA;
            compleationDistance = _compleationDistance;
            desiredSpeed = _speed;
            ropeDesity = _ropeDensity;
            springConstant = _springConstant;
            ropeSolverSpeed = _ropeSolverSpeed;
            obstacles = _obstacles;

            points = _points;
            currentPoint = _points[0];
            pos = (Vector3d)_points[0];
            lastPos = (Vector3d)_points[0];

            posHistory.Add(_points[0]);
            posHistory.Add(_points[1]);

            ropeSim = new AsyncRope(_ropeDensity, _springConstant, ropeSolverSpeed,obstacles, ref _DA);
            ropePointMass.Add(new pointMass(new Vector3d(_points[0]), segmentLength * _ropeDensity, Vector3d.Zero));
            ropePointMass.Add(new pointMass(new Vector3d(_points[0]), segmentLength * _ropeDensity, Vector3d.Zero));
            ropeRestLengths.Add(segmentLength);
            ropeRestLengths.Add(segmentLength);
        }

        public void StopNow()
        {
            shouldStop = true;
            this.Abort();
            base.Abort();
        }

        public override void run(BackgroundWorker worker)
        {
            Stopwatch ropeSimTimer = new Stopwatch();
            ropeSimTimer.Start();

            StringBuilder error = new StringBuilder();

            DA.SetData(0, "Simulating...");

            timeStep = (DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp) / 1000;
            lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;

            System.Threading.Thread.Sleep(solverSpeed);
            try
            {

                // for each via point
                for (int i = 1; i < points.Count; i++)
                {

                    if (shouldStop)
                    {
                        break;
                    }

                    DA.SetData(2, new Point3d(points[i]));

                    while (Math.Abs(points[i].DistanceTo(new Point3d(pos))) > compleationDistance && !shouldStop)
                    {
                        timeStep = (DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp) / 1000;

                        lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;

                        // stop if triggered from GH
                        if (shouldStop)
                        {
                            break;
                        }

                        // update the Pos and popHistory
                        pos = updatePosition((Vector3d)points[i]);

                        posHistory.Last = new Point3d(pos);
                        // Add pos to posHistory if it is far enough away.
                        if (posHistory[posHistory.Count - 2].DistanceTo((Point3d)pos) > segmentLength)
                        {
                            posHistory.Add(posHistory.Last);
                            posHistory[posHistory.Count - 2] = new Point3d(pos);
                        }


                        ropePointMass[ropePointMass.Count - 1].Pos = pos;
                        // Add pos to rope if it is far enough away.
                        if (((Point3d)pos).DistanceTo(points[0]) > totalRopeLength + segmentLength)
                        {
                            ropeRestLengths.Add(segmentLength);
                            // set last and first as anchor points
                            ropePointMass.Add(new pointMass(pos, segmentLength * ropeDesity, Vector3d.Zero));
                            ropePointMass[ropePointMass.Count - 2] = new pointMass(pos + 0.5 * (ropePointMass[ropePointMass.Count - 3].Pos - pos), segmentLength * ropeDesity, new Vector3d(1, 1, 1));
                            totalRopeLength += segmentLength;
                        }


                        ropeSimTimer.Restart();
                        //   pointMass[] simArray = ropePointMass.ToArray();
                        double[] segLengthArray = ropeRestLengths.ToArray();

                        if (!ropePointMass[0].isFixed)
                        {
                            ropePointMass[0].Anchor = Vector3d.Zero;
                        }
                        if (!ropePointMass[ropePointMass.Count - 1].isFixed)
                        {
                            ropePointMass[ropePointMass.Count - 1].Anchor = Vector3d.Zero;
                        }


                        string errorMsg = "";
                        // Relax rope with pos location
                        if (ropePointMass.Count > 5)
                        {
                            while (ropeSimTimer.Elapsed.TotalMilliseconds < solverSpeed)
                            {
                                errorMsg = ropeSim.Simulate(ropePointMass, segLengthArray, ropeSolverSpeed);
                            }
                        }
                        else
                        {
                            System.Threading.Thread.Sleep(solverSpeed);
                        }


                        rope = new Polyline(ropePointMass.Count);
                        for (int j = 0; j < ropePointMass.Count; j++)
                        {
                            rope.Add(ropePointMass[j].Point);
                        }

                        // Publish to GH
                        DA.SetData(1, new Point3d(pos));
                        DA.SetData(3, posHistory);
                        DA.SetData(4, rope);

                        DA.SetData(0, "Simulating...\n" + errorMsg);

                        // lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;
                    }

                    worker.ReportProgress(100 * i / points.Count);
                }

                if (ropePointMass.Count > 5)
                {
                    while (!shouldStop)
                    {
                        pointMass[] simArray = ropePointMass.ToArray();
                        double[] segLengthArray = ropeRestLengths.ToArray();
                        string errorMsg = "";

                        errorMsg = ropeSim.Simulate(ropePointMass, ropeRestLengths.ToArray(), ropeSolverSpeed);

                        rope = new Polyline(simArray.Length);
                        for (int j = 0; j < simArray.Length; j++)
                        {
                            rope.Add(ropePointMass[j].Point);
                        }

                        DA.SetData(4, rope);
                        DA.SetData(0, "Simulating...\n" + errorMsg);
                    }
                }
            }
            catch (Exception e)
            {

                DA.SetData(0, "Error: " + e.Message + "\n" + e.StackTrace + "\n" + e.Source);
            }

        }


        public Vector3d updatePosition(Vector3d desiredLocation)
        {
           // timeStep = (DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp) / 1000;
            lastVel = (pos - lastPos);

            totalAcceleration = desiredLocation - pos;
            totalAcceleration.Unitize();
            totalAcceleration = totalAcceleration * desiredSpeed / timeStep;

            lastPos = pos;
            Vector3d nextPos = pos + lastVel / 5 + totalAcceleration * timeStep * timeStep;

            veclocityVec = nextPos - pos;
            veclocityVec.Unitize();
            nextPos = veclocityVec * desiredSpeed + pos;

            return nextPos;
        }


        
    }
}
