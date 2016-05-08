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

        double ropeDesity = 0.4;                    // kg/m
        double springConstant = 3000;               // rope is 1300 N/m
        // wire rope 5.345*10^7
        readonly double internalFrictionConstant = 0.005;      // 0.2 N/(m/s) is good from trial and error
        readonly double dragCoeficient = 0.45;              // Shape is a circle
        double solverSpeed = 0.0007;                        // ms

        double[] segmentLengths;
        pointMass[] ropePointMass;


        public AsyncRope(double _density, double _springConstant, double _solverSpeed, ref IGH_DataAccess _DA)
        {
            DA = _DA;
            ropeDesity = _density;
            springConstant = _springConstant;
            solverSpeed = _solverSpeed;
        }

        public AsyncRope(Polyline _rope, double _density, double _springConstant, double _solverSpeed, ref IGH_DataAccess _DA)
        {
            DA = _DA;
            ropeDesity = _density;
            springConstant = _springConstant;
            solverSpeed = _solverSpeed;

            segmentLengths = new double[_rope.SegmentCount];
            ropePointMass = new pointMass[_rope.SegmentCount + 1];

            for (int i = 0; i < _rope.SegmentCount; i++)
            {
                segmentLengths[i] = _rope.SegmentAt(i).Length;
            }

            // Set first point as anchor
            ropePointMass[0] = new pointMass(new Vector3d(_rope.First()), _density * .1, Vector3d.Zero);
            // middle points are anchorless
            for (int i = 1; i < ropePointMass.Length - 1; i++)
            {
                ropePointMass[i] = new pointMass(new Vector3d(_rope[i]), _density * segmentLengths[i - 1], new Vector3d(1, 1, 1));
            }
            // set last point as anchor
            ropePointMass[ropePointMass.Length - 1] = new pointMass(new Vector3d(_rope.Last), _density * segmentLengths[segmentLengths.Length - 1], Vector3d.Zero);

        }

        public void StopNow()
        {
            shouldStop = true;
        }

        public override void run(BackgroundWorker worker)
        {
            DA.SetData(0, "Simulating...");

            // Relax PopHistory with pos location


            Vector3d debug = Vector3d.Zero;
            Stopwatch timere = new Stopwatch();

            while (!shouldStop)
            {
                timere.Restart();
                string error = Simulate(ropePointMass, segmentLengths, solverSpeed);
                error = timere.Elapsed.TotalMilliseconds.ToString() + "\n" + error;
                DA.SetData(0, error.ToString());
                DA.SetData(1, ropePointMass.ToPoly());
            }
        }

        public string Simulate(pointMass[] SimRope, double[] restLengths, double timeStep)
        {
            double distance = 0;
            Vector3d segmentVector = Vector3d.Zero;
            Vector3d segmentVelocity = Vector3d.Zero;
            StringBuilder error = new StringBuilder();

            // for each segment
            for (int i = 0; i < (SimRope.Length - 1); i++)
            {
                // add the force due to spring constant to [i,i+1]
                segmentVector = SimRope[i + 1].Pos - SimRope[i].Pos;
                distance = segmentVector.Length;
                segmentVector.Unitize();
                // force
                SimRope[i].addForce(segmentVector * (distance - restLengths[i]) * springConstant);
                SimRope[i + 1].addForce(-segmentVector * (distance - restLengths[i]) * springConstant);

                // add the force due to internal friction to [i,i+1]
                segmentVelocity = SimRope[i + 1].Vel - SimRope[i].Vel; // relative to pos i

                // Forces
                SimRope[i].addForce(-segmentVelocity * internalFrictionConstant);
                SimRope[i + 1].addForce(segmentVelocity * internalFrictionConstant);

                // add the force due to wind resistance to [i]
                SimRope[i].addForce(-0.5 * 1.225 * SimRope[i].Vel.SquareLength * dragCoeficient * .1 * distance * SimRope[i].unitVel); // 1.225 is desity of air // 0.1*distance is projected area


                // simulate [i]
                SimRope[i].solve(timeStep);
            }
            error.AppendLine("RopeSim all good.");
            return error.ToString();
        }

    }




    class pointMass
    {
        Vector3d position, lastPosition, velocity, forces, anchor;
        double mass;

        public Vector3d Vel { get { return velocity; } }
        public Vector3d Pos { get { return position; } set { position = value; } }
        public Point3d Point { get { return new Point3d(position); } }
        public Vector3d unitVel { get { Vector3d outVec = new Vector3d(velocity); outVec.Unitize(); return outVec; } }
        public bool isFixed { get { return anchor.Equals(Vector3d.Zero); } }
        public Vector3d Anchor { set { anchor = value; } }

        public pointMass(Vector3d startPosition, double weight, Vector3d _anchor)
        {
            position = startPosition;
            lastPosition = startPosition;
            velocity = Vector3d.Zero;
            mass = weight;
            forces = new Vector3d(0, 0, -9.8 * mass);
            anchor = _anchor;
        }

        public void addForce(Vector3d newForce)
        {
            forces += newForce;
        }

        public void solve(double timeStep)
        {
            velocity = (position - lastPosition) / timeStep;
                lastPosition = position;
                position = position + (velocity * timeStep + forces * timeStep * timeStep / (mass)).scalerMultiply(anchor);
            forces = new Vector3d(0, 0, -9.8 * mass);
        }

    }

}
