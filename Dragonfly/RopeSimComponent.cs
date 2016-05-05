using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using System.IO;
using Threaded;

namespace Dragonfly
{
    public class RopeSimComponent : GH_Component
    {
        static List<AsyncRope> ropeSims = new List<AsyncRope>();
        bool hasloaded = false;

        /// <summary>
        /// Initializes a new instance of the RopeSimComponent class.
        /// </summary>
        public RopeSimComponent()
            : base("RopeSimComponent", "Rope",
                "Rope Simulator, This will relax a rope",
                "Dragonfly", "copterSim")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Rope", "Rope", "Rope from the copter", GH_ParamAccess.list);
            pManager.AddNumberParameter("Rope Constant (N/m)", "Stress", "The spring constant in hooks law, F=kx, 0.1 std", GH_ParamAccess.item);
            pManager.AddNumberParameter("rope Density", "weight", "weight per linear m, rope 1cm diameter is 0.05", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Start Sim", "Start", "toggle to start", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Progress Text", "Err", "Text from the background worker. will usually be a percentage of task.", GH_ParamAccess.item);
            pManager.AddCurveParameter("Rope", "Rope", "Rope from the copter", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Declare a variable for the input String
            double ropeweight = 0.05;
            double springConstant = 10;
            List<Point3d> ropeVectors = new List<Point3d>();
            Polyline rope = new Polyline();
            bool StartSim = false;

            StringWriter ErrorMsg = new StringWriter();

            // Use the DA object to retrieve the data inside the first input parameter.
            // If the retieval fails (for example if there is no data) we need to abort.
            if (!DA.GetDataList(0, ropeVectors)) { return; }
            if (!DA.GetData(1, ref springConstant)) { return; }
            if (!DA.GetData(2, ref ropeweight)) { return; }
            if (!DA.GetData(3, ref StartSim)) { return; }

            rope = new Polyline(ropeVectors);
            // If the retrieved data is Nothing, we need to abort.
            // We're also going to abort on a zero-length String.
            if (ropeweight < 0 || ropeweight > 1)
            {
                ErrorMsg.WriteLine("weight is crazy, try above 0 and 1 kg/m");
                DA.SetData(0, ErrorMsg.ToString());

                return;
            }
            if (springConstant < 0)
            {
                ErrorMsg.WriteLine("springconstant is crazy, try above 0, say 100?");
                DA.SetData(0, ErrorMsg.ToString());

                return;
            }


            if (StartSim)
            {
                if (!hasloaded)
                {
                    AsyncRope ropeSim = new AsyncRope(rope,ropeweight,springConstant, ref DA);
                    AsyncRope.hasLoaded = true;
                    ropeSims.Add(ropeSim);
                    ErrorMsg.WriteLine("starting");
                    DA.SetData(0, ErrorMsg.ToString());

                    ropeSim.Start();
                    ErrorMsg.WriteLine("started");
                    hasloaded = true;
                }
                else
                {
                    ErrorMsg.WriteLine("Simulating....");
                    DA.SetData(0, ErrorMsg.ToString());
                }
            }
            else
            {
                if (hasloaded)
                {
                    ErrorMsg.WriteLine("Stopping Simulation");
                    DA.SetData(0, ErrorMsg.ToString());

                    foreach (AsyncRope Sim in ropeSims)
                    {
                        Sim.StopNow();
                        Sim.Abort();
                    }
                    AsyncRope.hasLoaded = false;
                    hasloaded = false;
                    ropeSims.Clear();
                }
                else
                {
                    DA.SetData(0, ErrorMsg.ToString());
                    ErrorMsg.WriteLine("Simulation is stopped");
                    DA.SetData(0, ErrorMsg.ToString());
                }
            }
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{da1b8951-2804-4633-b3b5-1d63bb80b837}"); }
        }
    }
}