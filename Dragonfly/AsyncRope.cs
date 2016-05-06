using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Threaded.Support;
using CustomExtensions;

using Grasshopper.Kernel;
using Rhino.Geometry;

using System.Diagnostics;
using System.IO;
using System.ComponentModel;

namespace Threaded
{
    class AsyncRope : GenericWorker
    {
        IGH_DataAccess DA;

        public static bool hasLoaded = false;
        bool shouldStop = false;

        // Copter Physics variables

        Polyline rope = new Polyline();
        double ropeDesity = 0.05;                    // kg/m
        double springConstant = 130;               // rope is 1300 N/m
                                                    // wire rope 5.345*10^7
        Polyline ropeLastPos = new Polyline();
        double lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;
        int solverSpeed = 1;                        // ms
        double timeStep = 1;
        Vector3d gravity = new Vector3d(0, 0, -9.8);

        double[] segmentLengths;

        public AsyncRope(Polyline _rope, double _density, double _springConstant, ref IGH_DataAccess _DA)
        {
            rope = new Polyline(_rope);
            segmentLengths = new double[rope.SegmentCount];
            for (int i = 0; i < rope.SegmentCount; i++)
            {
                segmentLengths[i] = rope.SegmentAt(i).Length;
            }
            ropeLastPos = new Polyline(_rope);
            DA = _DA;
            ropeDesity = _density;
            springConstant = _springConstant;
        }

        public void StopNow()
        {
            shouldStop = true;
        }

        public override void run(BackgroundWorker worker)
        {
            DA.SetData(0, "Simulating...");
            timeStep = DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp;
            lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;
            System.Threading.Thread.Sleep(solverSpeed);


            // Relax PopHistory with pos location
            relaxPoly(rope);

        }

        void relaxPoly(Polyline rope)
        {
            // timeStep = (DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp) / 1000;
            Stopwatch timer = new Stopwatch();
            timer.Start();

            double distance;
            Vector3d totalForce;
            Vector3d velocity;

            Vector3d[] springForce = new Vector3d[rope.SegmentCount+1];

            Vector3d debug = Vector3d.Zero;

            while (!shouldStop)
            {
                timeStep = (DateTime.Now.TimeOfDay.TotalMilliseconds - lastTimeStamp) / 1000;
                springForce[0] = Vector3d.Zero;

                debug.X = rope.SegmentCount;

                // stop if triggered from GH
                if (shouldStop)
                {
                    break;
                }
                for (int i = 0; i < rope.SegmentCount; i++)
                {

                    distance = rope.SegmentAt(i).Length;
                    // force at i+1
                    springForce[i] = springForce[i] + (rope[i + 1] - rope[i]) * (distance - segmentLengths[i]) * springConstant;
                    springForce[i + 1] = (rope[i] - rope[i + 1]) * (distance - segmentLengths[i]) * springConstant;

                    if (i != 0)
                    {
                        totalForce = springForce[i] + gravity * ropeDesity * segmentLengths[i];
                        velocity = (rope[i] - ropeLastPos[i]) / timeStep;
                        ropeLastPos[i] = rope[i];
                        rope[i] = rope[i] + velocity * timeStep + totalForce * timeStep * timeStep / (ropeDesity * segmentLengths[i]);
                    }
                }

                DA.SetData(1, new Polyline(rope));
                DA.SetData(3, debug);
                /*
                for (int i = 1; i < rope.SegmentCount - 1; i++)
                {

                    distance = rope[i - 1].DistanceTo(rope[i]);
                    springForce = (rope[i - 1] - rope[i]) * (distance - segmentLengths[i - 1]) * springConstant;
                    totalForce = springForce + gravity * ropeDesity * segmentLengths[i - 1];

                    velocity = (rope[i] - ropeLastPos[i]) / timeStep;
                    ropeLastPos[i] = rope[i];
                    rope[i] = rope[i] + velocity * timeStep + totalForce * timeStep * timeStep / (ropeDesity * segmentLengths[i - 1]);

                    DA.SetData(1, new Polyline(rope));

                }
                 * 
                 */

                

                lastTimeStamp = DateTime.Now.TimeOfDay.TotalMilliseconds;
                System.Threading.Thread.Sleep(solverSpeed);

            }

        }
    }
}
