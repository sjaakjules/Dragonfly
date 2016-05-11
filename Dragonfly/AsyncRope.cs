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
        double groundFrictionConstant = 0.2;
        double groundRepelConstant = 100;
        double groundAbsoptionConstant = 2;
        double solverSpeed = 0.001;                        // ms
        double collisionDistance = .2;

        List<double> segmentLengths = new List<double>();
        List<pointMass> ropePointMass = new List<pointMass>();

        List<Plane> collisionPlanes = new List<Plane>();

        Curve[] obstacles;


        public AsyncRope(double _density, double _springConstant, double _solverSpeed, Curve[] _obstacles, ref IGH_DataAccess _DA)
        {
            DA = _DA;
            ropeDesity = _density;
            springConstant = _springConstant;
            solverSpeed = _solverSpeed;
            obstacles =  _obstacles;
        }

        public AsyncRope(Polyline _rope, double _density, double _springConstant, double _solverSpeed, double _groundFrictionConstant, double _groundRepelConstant, double _groundAbsoptionConstant, Curve[] _obstacles, ref IGH_DataAccess _DA)
        {
            DA = _DA;
            ropeDesity = _density;
            springConstant = _springConstant;
            solverSpeed = _solverSpeed;
            groundFrictionConstant = _groundFrictionConstant;
            groundRepelConstant = _groundRepelConstant;
            groundAbsoptionConstant = _groundAbsoptionConstant;
            obstacles =  _obstacles;
            
            for (int i = 0; i < _rope.SegmentCount; i++)
            {
                segmentLengths.Add(_rope.SegmentAt(i).Length);
            }

            // Set first point as anchor
            ropePointMass.Add( new pointMass(new Vector3d(_rope.First()), _density * .1, Vector3d.Zero));
            // middle points are anchorless
            for (int i = 1; i < segmentLengths.Count; i++)
            {
                ropePointMass.Add( new pointMass(new Vector3d(_rope[i]), _density * segmentLengths[i - 1], new Vector3d(1, 1, 1)));
            }
            // set last point as anchor
            ropePointMass.Add(new pointMass(new Vector3d(_rope.Last), _density * segmentLengths[segmentLengths.Count - 1], Vector3d.Zero));

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
                string error = Simulate(ropePointMass, segmentLengths.ToArray(), solverSpeed);
                error = timere.Elapsed.TotalMilliseconds.ToString() + "\n" + error;
                DA.SetData(0, error.ToString());
                DA.SetData(1, ropePointMass.ToPoly());
            }
        }

        public string Simulate(List<pointMass> SimRope, double[] restLengths, double timeStep)
        {
            collisionDistance = restLengths[0] * 3;
            double distance = 0;
            Vector3d segmentVector = Vector3d.Zero;
            Vector3d segmentVelocity = Vector3d.Zero;
            StringBuilder error = new StringBuilder();

            // for each segment update the forces and move the rope one step.
            for (int i = 0; i < (SimRope.Count - 1); i++)
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


                // If the point is below is below the ground
                if (SimRope[i].Pos.Z < 0)
                {
                    Vector3d tempVelocity = SimRope[i].Vel;
                    tempVelocity.Z = 0;

                    // apply surface friction
                    SimRope[i].addForce(-tempVelocity * groundFrictionConstant);

                    tempVelocity = SimRope[i].Vel;
                    tempVelocity.X = 0;
                    tempVelocity.Y = 0;

                    // if collision apply absorption force
                    if (tempVelocity.Z < 0)
                    {
                        SimRope[i].addForce(-tempVelocity * groundAbsoptionConstant);
                    }

                    // apply repulsion force
                    SimRope[i].addForce(new Vector3d(0, 0, groundRepelConstant * -SimRope[i].Pos.Z));

                }

                // simulate [i]
                SimRope[i].solve(timeStep);
            }

            // if rope is coliding with other ropes. 

            Polyline rope = new Polyline(SimRope.Count);
            for (int j = 0; j < SimRope.Count; j++)
            {
                rope.Add(SimRope[j].Point);
            }

            NurbsCurve curve = rope.ToNurbsCurve();

            try
            {
                if (obstacles != null && obstacles.Length > 0)
                {
                    for (int i = 0; i < obstacles.Length; i++)
                    {
                        double max = 0;
                        int maxIndicie = 0;
                        double secondMax = 0;
                        int secondMaxIndicie = 0;
                        Point3d ropeInt, obsticalInt;

                        curve.ClosestPoints(obstacles[i], out ropeInt, out obsticalInt);

                        // If the distance is within the collision zone
                        if (ropeInt.DistanceTo(obsticalInt) < collisionDistance)
                        {
                            // Check each node of the rope and add collision to planes if a plane of a close plane
                            for (int j = 0; j < SimRope.Count; j++)
                            {
                                // if a massnode is close to the collision point and not in the collision planes
                                if (SimRope[j].Point.DistanceTo(ropeInt) < segmentLengths.Average())
                                {
                                    // check if it is within the planes.
                                    // If it isnt within collision zone size area of the collision list then add to collection 
                                    Point3d collisionNormalVector = ropeInt.minus(obsticalInt);
                                    Plane newcollisionPlane = new Plane(obsticalInt, new Vector3d(collisionNormalVector));

                                    bool isWithin = false;
                                    Plane[] tempcollisionPlanes = new Plane[collisionPlanes.Count + 1];
                                    foreach (var plane in collisionPlanes)
                                    {
                                        // if the point on the surface of the obstical  is within collison distance of each plane in collisionplanes
                                        if (plane.Origin.DistanceTo(newcollisionPlane.Origin) < collisionDistance)
                                        {
                                            // And is within the same normal direction
                                            if (plane.Normal.getAngleBetween(newcollisionPlane.Normal) < Math.PI)
                                            {
                                                // modify the plane which is similar to be the new vector 
                                            }
                                            // And oposite direction to stored value
                                            if (plane.Normal.getAngleBetween(newcollisionPlane.Normal) > Math.PI)
                                            {

                                            }
                                            isWithin = true;
                                            break;
                                        }
                                        // if not
                                        else
                                        {
                                            applyCollisionForce(newcollisionPlane, SimRope[j]);
                                        }
                                    }
                                    if (!isWithin)
                                    {
                                        collisionPlanes.Add(newcollisionPlane);
                                    }
                                }
                            }

                        }
                        //Curve.GetDistancesBetweenCurves(obstacles[i], rope.ToNurbsCurve(), 0.1, out max, out  Amax, out  Bmax, out  min, out  Amin, out Bmin);

                    }
                }

            }
            catch (Exception e)
            {


                error.AppendLine("Shit error: \n" + e.Message + "\n" + e.ToString() + "\n" + e.StackTrace + "\n" + e.Source+"\n" + e.InnerException.Message);
                return error.ToString();
            }


            error.AppendLine("RopeSim all good.");
            return error.ToString();
        }

        /// <summary>
        /// Sets the collision force relative to the normal of a plane on the surface of the impacted object 
        /// </summary>
        /// <param name="collisionPlane"></param>
        /// <param name="collisionNode"></param>
        /// <param name="collisionPoint"></param>
        /// <returns></returns>
        void applyCollisionForce(Plane collisionPlane, pointMass collisionNode)
        {
            Point3d remappedPoint;
            collisionPlane.RemapToPlaneSpace(collisionNode.Point, out remappedPoint);
            Vector3d collisionNormal = collisionPlane.Normal;

            if (collisionPlane.DistanceTo(collisionNode.Point) < 0)
            {
                Point3d tempVelocity = new Point3d( collisionNode.Vel);
                Point3d planarPoint;
                // temp velocity relative to the plane with relative.z =0
                collisionPlane.RemapToPlaneSpace(tempVelocity, out planarPoint);

                Point3d planarVelocity = collisionPlane.PointAt(planarPoint.X, planarPoint.Y, 0);

                // apply surface friction to this relative velocity in global coordinates
                collisionNode.addForce(-new Vector3d(tempVelocity) * groundFrictionConstant);

                // velocity in the normal to the plane direction only. (project velocity vector with normal vector, vel dot normal vector unitised times unitised normal vector
                tempVelocity = new Point3d(collisionNode.Vel.DotProduct(collisionPlane.Normal.Unit()) *collisionPlane.Normal.Unit());

                // if collision apply absorption force
                if (tempVelocity.Z < 0)
                {
                    collisionNode.addForce(-new Vector3d(tempVelocity) * groundAbsoptionConstant);
                }

                // apply repulsion force relative to the normal vector. (Project again so use unitised tempVelocity)
                collisionNode.addForce(-new Vector3d(tempVelocity) * groundRepelConstant);
            }

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
