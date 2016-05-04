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
    class AsyncRope : GenericWorker
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

        // Physics variables
        Vector3d pos;
        Vector3d lastPos;
        Vector3d lastVel;
        Vector3d veclocityVec;
        Vector3d totalAcceleration;
        Vector3d thrust;
        Vector3d gravity = new Vector3d(0, 0, -9.8);
        double desiredSpeed = 0.1;                  // m/s
        double lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;
        int solverSpeed = 1;                        // ms
        double timeStep = 1;

        Point3d currentPoint;

        Polyline posHistory = new Polyline();
        double segmentLength = 1;                   // m

        public AsyncRope(List<Point3d> _points, double _compleationDistance, double _speed, ref IGH_DataAccess _DA)
            : base()
        {
            points = _points;
            currentPoint = _points[0];
            pos = (Vector3d)_points[0];
            lastPos = (Vector3d)_points[0];
            posHistory.Add(_points[0]);
            posHistory.Add(_points[1]);
            DA = _DA;
            compleationDistance = _compleationDistance;
            desiredSpeed = _speed;
        }

        public void StopNow()
        {
            shouldStop = true;
        }

        public override void run(BackgroundWorker worker)
        {
            Stopwatch timer = new Stopwatch();
            timer.Start();
            bool overshotPoint = false;
            DA.SetData( 0, "Simulating...");
            timeStep = DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp;
            lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;
            System.Threading.Thread.Sleep(solverSpeed);

            for (int i = 1; i < points.Count; i++)
            {
                if (shouldStop)
                {
                    break;
                }
                double startDistance = Math.Abs(points[i].DistanceTo(new Point3d(pos)));
                double maxTime = 2 * startDistance / desiredSpeed;

                DA.SetData(2, new Point3d(points[i]));

                timer.Restart();
                while (Math.Abs(points[i].DistanceTo(new Point3d(pos))) > compleationDistance)
                {
                    // stop if triggered from GH
                    if (shouldStop)
                    {
                        break;
                    }

                    // update the Pos and popHistory
                    pos = updatePosition((Vector3d)points[i]);
                    posHistory.Last = new Point3d(pos);

                    // Add pos to posHistory if it is far enough away.
                    if (posHistory[posHistory.Count- 2].DistanceTo((Point3d)pos) > segmentLength)
                    {
                        posHistory.Add(posHistory.Last);
                        posHistory[posHistory.Count - 2] = new Point3d(pos);
                        //posHistory.Insert(posHistory.Count - 1, new Point3d(pos));
                        //posHistory.Add(new Point3d(pos));
                    }

                    // Relax PopHistory with pos location


                    // Publish to GH
                    DA.SetData(1, new Point3d(pos));
                    DA.SetData(3, posHistory);

                    if (timer.Elapsed.TotalSeconds > maxTime)
                    {
                        overshotPoint = true;
                        break;
                    }

                    System.Threading.Thread.Sleep(solverSpeed);
                }
                if (overshotPoint)
                {
                    DA.SetData( 0, "Oh shit, you overshot");
                    this.StopNow();
                    this.Abort();
                    break;
                }
                worker.ReportProgress(100 * i / points.Count);
            }
        }

        public Vector3d updatePosition(Vector3d desiredLocation)
        {
            timeStep = (DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp) / 1000;
            lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;

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

        void relaxPoly(Polyline rope)
        {

        }
        
    }
}
