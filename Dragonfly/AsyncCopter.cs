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
        int solverSpeed = 1;                        // ms
        double timeStep = 1;

        Point3d currentPoint;

        Polyline posHistory = new Polyline();
        double segmentLength = 1;                   // m

        Polyline rope = new Polyline();
        double ropeDesity = 0.05;                    // kg/m
        double lastRope = 0;
        double springConstant = 130;               // rope is 1300 N/m
                                                    // wire rope 5.345*10^7
        Polyline ropeLastPos = new Polyline(); 


        public AsyncCopter(List<Point3d> _points, double _compleationDistance, double _speed,double _ropeDensity, double _springConstant, ref IGH_DataAccess _DA)
            : base()
        {
            points = _points;
            currentPoint = _points[0];
            pos = (Vector3d)_points[0];
            lastPos = (Vector3d)_points[0];
            posHistory.Add(_points[0]);
            posHistory.Add(_points[1]);
            rope.Add(_points[0]);
            rope.Add(_points[1]);
            ropeLastPos.Add(_points[0]);
            ropeLastPos.Add(_points[1]);
            DA = _DA;
            compleationDistance = _compleationDistance;
            desiredSpeed = _speed;
            ropeDesity = _ropeDensity;
            springConstant = _springConstant;
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
                    timeStep = (DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp) / 1000;
                    

                    // stop if triggered from GH
                    if (shouldStop)
                    {
                        break;
                    }

                    // update the Pos and popHistory
                    pos = updatePosition((Vector3d)points[i]);
                    posHistory.Last = new Point3d(pos);
                    rope.Last = new Point3d(pos);

                    // Add pos to posHistory if it is far enough away.
                    if (posHistory[posHistory.Count- 2].DistanceTo((Point3d)pos) > segmentLength)
                    {
                        posHistory.Add(posHistory.Last);
                        posHistory[posHistory.Count - 2] = new Point3d(pos);
                        //posHistory.Insert(posHistory.Count - 1, new Point3d(pos));
                        //posHistory.Add(new Point3d(pos));
                    }

                    // Add pos to rope if it is far enough away.
                    if (((Point3d)pos).DistanceTo(points[0]) > lastRope + segmentLength)
                    {
                        rope.Add(rope.Last);
                        rope[rope.Count - 2] = new Point3d(pos);

                        ropeLastPos.Add(ropeLastPos.Last);
                        ropeLastPos[ropeLastPos.Count - 2] = new Point3d(pos);

                        lastRope += segmentLength;
                    }

                    // Relax PopHistory with pos location
                    relaxPoly(rope);

                    // Publish to GH
                    DA.SetData(1, new Point3d(pos));
                    DA.SetData(3, posHistory);
                    DA.SetData(4, rope);

                    if (timer.Elapsed.TotalSeconds > maxTime)
                    {
                        overshotPoint = true;
                        break;
                    }
                    lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;
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

        void relaxPoly(Polyline rope)
        {
           // timeStep = (DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp) / 1000;
            double distance;
            Vector3d springForce;
            Vector3d totalForce;
            Vector3d velocity;
            for (int i = 1; i < rope.Length; i++)
            {
                distance = rope[i - 1].DistanceTo(rope[i]);
                springForce = (rope[i - 1] - rope[i]) * (distance - segmentLength) * springConstant;
                totalForce = springForce + gravity * ropeDesity * segmentLength;

                velocity = (rope[i] - ropeLastPos[i]) / timeStep;
                ropeLastPos[i] = rope[i];
                rope[i] = rope[i] + velocity * timeStep + totalForce * timeStep * timeStep;
            }

        }
        
    }
}
