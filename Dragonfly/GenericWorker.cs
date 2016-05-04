using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;

namespace Threaded.Support
{

    // This is the async thread parent
    public class GenericWorker
    {

        public bool running = false;
        BackgroundWorker bw = new BackgroundWorker();
        public String progressText = "";


        public GenericWorker()
        {
            bw.WorkerReportsProgress = true;
            bw.WorkerSupportsCancellation = true;
            bw.DoWork += new DoWorkEventHandler(bw_DoWork);
            bw.ProgressChanged += new ProgressChangedEventHandler(bw_ProgressChanged);
            bw.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
        }

        public void Start()
        {
            if (bw.IsBusy != true)
            {
                running = true;
                bw.RunWorkerAsync();
            }
        }

        public void Abort()
        {
            if (bw.WorkerSupportsCancellation == true)
            {
                bw.CancelAsync();
            }
            bw.Dispose();
        }

        private void bw_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;

            run(worker);

        }

        private void bw_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            if ((e.Cancelled == true))
            {
                progressText = "Canceled!";
                running = false;
                bw.DoWork -= new DoWorkEventHandler(bw_DoWork);
                bw.ProgressChanged -= new ProgressChangedEventHandler(bw_ProgressChanged);
                bw.RunWorkerCompleted -= new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
            }

            else if (!(e.Error == null))
            {
                progressText = ("Error: " + e.Error.Message);
                running = false;
                bw.DoWork -= new DoWorkEventHandler(bw_DoWork);
                bw.ProgressChanged -= new ProgressChangedEventHandler(bw_ProgressChanged);
                bw.RunWorkerCompleted -= new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
            }

            else
            {
                running = false;
                progressText = "Done!";
                bw.DoWork -= new DoWorkEventHandler(bw_DoWork);
                bw.ProgressChanged -= new ProgressChangedEventHandler(bw_ProgressChanged);
                bw.RunWorkerCompleted -= new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
            }
        }

        private void bw_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            progressText += (e.ProgressPercentage.ToString() + "%");
        }


        public virtual void run(BackgroundWorker worker)
        {

            // worker.ReportProgress(0);
        }
    }
}