using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;

namespace Threaded.Support
{
    // This is the async thread parent
    public class KeyedGenericWorker
    {
        static List<Guid> keys = new List<Guid>();
        static Dictionary<Guid, BackgroundWorker> bw = new Dictionary<Guid,BackgroundWorker>();
        public String progressText = "";
        public bool running = false;
        public bool hasCalled = false;
        Guid key;
        StringWriter errorMsg;
        object errorLock;


        public KeyedGenericWorker(Guid threadKey, StringWriter _errorMsg, object _errorLock)
        {
            errorMsg = _errorMsg;
            errorLock = _errorLock;
            key = threadKey;
            if (!keys.Contains(threadKey))
            {
                keys.Add(threadKey);
                bw.Add(threadKey, new BackgroundWorker());
                bw[key].WorkerReportsProgress = true;
                bw[key].WorkerSupportsCancellation = true;
                bw[key].DoWork += new DoWorkEventHandler(bw_DoWork);
                bw[key].ProgressChanged += new ProgressChangedEventHandler(bw_ProgressChanged);
                bw[key].RunWorkerCompleted += new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
            }
            else
            {

                hasCalled = true;
            }
        }

        public void Start()
        {
            lock (errorLock)
            {
                errorMsg.WriteLine("Starting background");
            }
            if (keys.Contains(key) && bw[key].IsBusy != true)
            {
                lock (errorLock)
                {
                    errorMsg.WriteLine("Key found...");
                }
                running = true;
                bw[key].RunWorkerAsync();
            }
            else
            {

                lock (errorLock)
                {
                    errorMsg.WriteLine("Key not found");
                }
            }

        }

        public void Abort()
        {

            lock (errorLock)
            {
                errorMsg.WriteLine("Runaway!");
            }
            if (keys.Contains(key) && bw[key].WorkerSupportsCancellation == true)
            {
                bw[key].CancelAsync();
                keys.Remove(key);
                bw.Remove(key);
            }
            else if (keys.Contains(key))
            {
                bw[key].Dispose();
            }
        }

        public void Start(Guid key)
        {
            if (keys.Contains(key) && bw[key].IsBusy != true)
            {
                running = true;
                bw[key].RunWorkerAsync();
            }

        }
        public void Abort(Guid key)
        {
            if (keys.Contains(key) && bw[key].WorkerSupportsCancellation == true)
            {
                bw[key].CancelAsync();
            }
        }

        public static bool IsRunning(Guid key)
        {
            if (keys.Contains(key))
            {
                return bw[key].IsBusy;
            }
            return false;
        }

        public static bool HasLoaded(Guid key)
        {
            if (keys.Contains(key))
            {
                return true;
            }
            return false;
        }

        public static void Remove(Guid key)
        {
            if (!bw[key].CancellationPending)
            {
                bw[key].CancelAsync();
            }
            keys.Remove(key);
            bw.Remove(key);
        }

        private void bw_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;

            lock (errorLock)
            {
                errorMsg.WriteLine("starting the run method");
            }
            run(worker);
        }

        private void bw_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            if ((e.Cancelled == true))
            {

                lock (errorLock)
                {
                    errorMsg.WriteLine("Canceled!");
                }
                progressText = "Canceled!";
                if (keys.Contains(key))
                {
                    bw[key].DoWork -= new DoWorkEventHandler(bw_DoWork);
                    bw[key].ProgressChanged -= new ProgressChangedEventHandler(bw_ProgressChanged);
                    bw[key].RunWorkerCompleted -= new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
                    keys.Remove(key);
                    bw.Remove(key);
                }
            }

            else if (!(e.Error == null))
            {
                lock (errorLock)
                {
                    errorMsg.WriteLine("Error: " + e.Error.Message);
                }
                progressText = ("Error: " + e.Error.Message);
                if (keys.Contains(key))
                {
                    bw[key].DoWork -= new DoWorkEventHandler(bw_DoWork);
                    bw[key].ProgressChanged -= new ProgressChangedEventHandler(bw_ProgressChanged);
                    bw[key].RunWorkerCompleted -= new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
                    keys.Remove(key);
                    bw.Remove(key);
                }
            }

            else
            {
                running = false;

                if (keys.Contains(key))
                {
                    bw[key].DoWork -= new DoWorkEventHandler(bw_DoWork);
                    bw[key].ProgressChanged -= new ProgressChangedEventHandler(bw_ProgressChanged);
                    bw[key].RunWorkerCompleted -= new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
                    keys.Remove(key);
                    bw.Remove(key);
                }
                lock (errorLock)
                {
                    errorMsg.WriteLine("done!");
                }
                progressText = "Done!";
            }
        }

        private void bw_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            progressText = (e.ProgressPercentage.ToString() + "%");
        }


        public virtual void run(BackgroundWorker worker)
        {
            // worker.ReportProgress(0);
        }
    }
}
