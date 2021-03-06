﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Grasshopper.Kernel;
using Rhino.Geometry;

using Threaded;
using Grasshopper.Kernel.Types.Transforms;

namespace CustomExtensions
{
    static class CustomExtensions
    {
        public static Mesh Orient(this Mesh obj, Orientation transform)
        {
            Mesh outMesh = obj.DuplicateMesh();
            outMesh.Transform(transform.ToMatrix());
            return outMesh;
        }

        public static Vector3d Unit(this Vector3d v)
        {
            Vector3d v_hat = v;
            v_hat.Unitize();
            return v_hat;
        }

        public static Polyline ToPoly(this pointMass[] simRope)
        {
            List<Point3d> outPoly = new List<Point3d>();
            for (int i = 0; i < simRope.Length; i++)
            {
                outPoly.Add(simRope[i].Point);
            }
            return new Polyline(outPoly);
        }

        public static Polyline ToPoly(this List<pointMass> simRope)
        {
            List<Point3d> outPoly = new List<Point3d>();
            for (int i = 0; i < simRope.Count; i++)
            {
                outPoly.Add(simRope[i].Point);
            }
            return new Polyline(outPoly);
        }

        public static void doubleArray(this double[] array, double value)
        {
            for (int i = 0; i < array.Length; i++)
            {
                array[i] = value;
            }
        }

        public static Point3d minus(this Point3d a, Point3d b)
        {
            Vector3d v_a = new Vector3d(a);

            Vector3d v_b = new Vector3d(b);
            return new Point3d(v_a - v_b);
        }

        public static double DotProduct(this Vector3d a, Vector3d b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vector3d scalerMultiply(this Vector3d a, Vector3d b)
        {
            return new Vector3d(a.X * b.X , a.Y * b.Y , a.Z * b.Z);
        }

        public static double CrossProductLength(this Vector3d a, Vector3d b)
        {
            return Vector3d.CrossProduct(a, b).Length;
        }

        /// <summary>
        /// Gets the angle between two vectors in radians. 
        /// This will unitise them to ensure the angle is correct.
        /// Error: will return a NaN if unitise isn't possible such as a zero length input.
        /// </summary>
        /// <param name="a"></param> The first vector, as it is an angle between them order isnt important
        /// <param name="b"></param> The second vector.
        /// <returns></returns>
        public static double getAngleBetween(this Vector3d a, Vector3d b)
        {
            if (a.Unitize() || b.Unitize())
            {
                return Math.Atan2(a.CrossProductLength(b), a.DotProduct(b));
            }
            return double.NaN;
        }
    }
}
