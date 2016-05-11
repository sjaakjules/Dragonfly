using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

using Threaded;
using Threaded.Support;
using CustomExtensions;
using System.IO;

namespace Dragonfly
{
    public class DragonflyComponent : GH_Component
    {
        static List<AsyncCopter> copterSims = new List<AsyncCopter>();
        bool hasloaded = false;

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public DragonflyComponent()
            : base("CopterSim", "Sim",
                "This will simulate a copter given a flight path",
                "Dragonfly", "copterSim")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Via Points", "Via Points", "A flatterned List of via points for the copter to fly between.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Compleation Distance", "Dist", "The maximum distance to the desired point before it moves down the list", GH_ParamAccess.item);
            pManager.AddNumberParameter("Speed (m/s)", "Speed", "The average speed for the copter to fly, 0.1 std", GH_ParamAccess.item);
            pManager.AddNumberParameter("Rope Constant (N/m)", "Stress", "The spring constant in hooks law, F=kx, 0.1 std", GH_ParamAccess.item);
            pManager.AddNumberParameter("rope Density", "Density", "weight per linear m, rope 1cm diameter is 0.05", GH_ParamAccess.item);
            pManager.AddNumberParameter("SolverSpeed", "SolverSpeed", "Time in ms for each loop. must be less than 0.001 for stability", GH_ParamAccess.item);
            pManager.AddCurveParameter("obstacles List", "List of obstacle ropes for rope simulation", "obstacles", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Start Sim", "Start", "toggle to start", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Progress Text", "Err", "Text from the background worker. will usually be a percentage of task.", GH_ParamAccess.item);
            pManager.AddPointParameter("Current Position", "Copter", "The current position which loops from start to finish.", GH_ParamAccess.item);
            pManager.AddPointParameter("Next Position", "Next Pos", "The current desited position.", GH_ParamAccess.item);
            pManager.AddCurveParameter("Copter Trail", "Trail", "copter trail which is the flight path it has taken", GH_ParamAccess.item);
            pManager.AddCurveParameter("Rope", "Rope", "Rope from the copter", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Declare a variable for the input String
            List<Point3d> viaPoints = new List<Point3d>();
            bool StartSim = false;
            StringWriter ErrorMsg = new StringWriter();
            double compleationDist = 0;
            double speeed = 0.1;
            double ropeweight = 0.5;
            double springConstant = 3000;
            double ropeSolverSpeed = 0.001;
            List<Curve> _obstacles = new List<Curve>();

            // Use the DA object to retrieve the data inside the first input parameter.
            // If the retieval fails (for example if there is no data) we need to abort.
            if (!DA.GetDataList(0, viaPoints)) { return; }
            if (!DA.GetData(1, ref compleationDist)) { return; }
            if (!DA.GetData(2, ref speeed)) { return; }
            if (!DA.GetData(3, ref springConstant)) { return; }
            if (!DA.GetData(4, ref ropeweight)) { return; }
            if (!DA.GetData(5, ref ropeSolverSpeed)) { return; }
            if (!DA.GetDataList(6, _obstacles)) { return; }
            if (!DA.GetData(7, ref StartSim)) { return; }

            // If the retrieved data is Nothing, we need to abort.
            // We're also going to abort on a zero-length String.
            if (viaPoints == null) { return; }
            if (viaPoints.Count <= 1) { return; }
            if (compleationDist <= 0)
            {
                ErrorMsg.WriteLine("Compleation Distance too small.");
                DA.SetData( 0, ErrorMsg.ToString());

                return;
            }
            if (speeed < 0 || speeed > 1)
            {
                ErrorMsg.WriteLine("Speed is crazy, try between 0 and 1m/s");
                DA.SetData( 0, ErrorMsg.ToString());

                return;
            }
            if (ropeSolverSpeed > 0.01)
            {
                ErrorMsg.WriteLine("solver speed is crazy! try below 0.001");
                DA.SetData(0, ErrorMsg.ToString());

                return;
            }

            DA.SetData( 0, ErrorMsg.ToString());

            if (StartSim)
            {
                if (!hasloaded)
                {
                    AsyncCopter ropeSim = new AsyncCopter(viaPoints, compleationDist, speeed, ropeweight, springConstant, ropeSolverSpeed, _obstacles.ToArray(), ref DA);
                    AsyncCopter.hasLoaded = true;
                    copterSims.Add(ropeSim);
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

                    foreach (AsyncCopter Sim in copterSims)
                    {
                        Sim.StopNow();
                        Sim.Abort();
                    }
                    AsyncCopter.hasLoaded = false;
                    hasloaded = false;
                    //ropeSims.Clear();
                }
                else
                {
                    DA.SetData(0, ErrorMsg.ToString());
                    ErrorMsg.WriteLine("Simulation is stopped");
                    DA.SetData(0, ErrorMsg.ToString());
                }

                for (int i = 0; i < copterSims.Count; i++)
                {
                    if (copterSims[i].running)
                    {
                        copterSims[i].StopNow();
                        copterSims[i].Abort();
                    }
                    else
                    {
                        copterSims.RemoveAt(i);
                    }
                }
            }
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return Properties.Resources.Dragonfly;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{34b4d52f-741f-4924-b6fc-19797baa350d}"); }
        }
    }
}
