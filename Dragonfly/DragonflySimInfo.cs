using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Dragonfly
{
    public class DragonflyInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "Copter Sim";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return Properties.Resources.Dragonfly2;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
                
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("4467efca-0ce5-474a-8220-db75ed901973");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
