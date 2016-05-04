﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace CustomExtensions
{
    static class CustomExtensions
    {
        public static double DotProduct(this Vector3d a, Vector3d b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
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