using System;
using System.ComponentModel;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Drawing.Imaging;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Threading.Tasks;
using System.Threading;

namespace KinectRecord
{
    class RGBRecord
    {
        //<summary>Conversts the RGB Stream of Kinect to Bitmap Color and writes the bitmap buffer to the file system in TIF format.</summary>
        private static String folderName = "C:/temp";
        public static float qual=1;
        //public ImageSource ToColorBitmap(int h, int w, int b, ColorImageFormat f, byte[] pd)
        //{
        //    int width = w;
        //    int height = h;
        //    int bpp = b;
        //    var format = f;
        //    byte[] pixelData = pd;
        //    int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8; ;
        //    WriteableBitmap colorBitmap = new WriteableBitmap(width, height, 96.0, 96.0, PixelFormats.Bgr32, null);
        //    colorBitmap.WritePixels(new Int32Rect(0, 0, width, height), pixelData, stride, 0);
        //    //Testing Block
        //    var temp = BitmapSource.Create(width, height, 96.0, 96.0, PixelFormats.Bgr32, null, pixelData, stride);
        //    long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
        //    FileStream stream = new FileStream("C:/temp/rgb/" + milliseconds + "temp.tif", FileMode.Create);
        //    TiffBitmapEncoder encoder = new TiffBitmapEncoder();
        //    encoder.Frames.Add(BitmapFrame.Create(temp));
        //    encoder.Save(stream);
        //    stream.Close();
        //    //End of Testing Block
        //    return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixelData, stride);
        //}   
        private static void saveTimeStamp(String loc, long stamp)
        {
            string filePath = folderName + ".txt";
            using (StreamWriter timesw = File.AppendText(filePath))
            {
                timesw.WriteLine(stamp);
                timesw.Close();
            }
        }

        public void ToColorBitmapQ(int h, int w, int b, ColorImageFormat f, byte[] pd)
        {
            int width = w;
            int height = h;
            int bpp = b;
            var format = f;
            byte[] pixelData = pd;
            int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;   
            var temp = BitmapSource.Create(width, height, 96.0, 96.0, PixelFormats.Bgr32, null, pixelData, stride);            
            temp.Freeze();
            long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;           
            MemoryStream mems = new MemoryStream();
            WmpBitmapEncoder encoder = new WmpBitmapEncoder();
            BitmapFrame bframe = BitmapFrame.Create(temp);
            encoder.ImageQualityLevel = qual;
            encoder.Frames.Add(bframe);
            encoder.Save(mems);
            qual -= 0.0001F;
            qual = Math.Max(qual, 0);
            try
                 {
                   MainWindow.RGBConcurrentQueue.Enqueue(mems);
                 }
                catch (Exception e)
                 {
                 qual = 0;
                }
            }

        private int GetObjectSize(object TestObject)
        {
            System.Runtime.Serialization.Formatters.Binary.BinaryFormatter bf = new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();
            MemoryStream ms = new MemoryStream();
            byte[] Array;
            bf.Serialize(ms, TestObject);
            Array = ms.ToArray();
            return Array.Length;
        }

        private BitmapImage GetImage(BitmapSource bmpSrc,int bpp)
        {
            //<summary>
            // Stores RGB frames from memoryStrean to Disk.
            //</summary>
            using (MemoryStream memoryStream = new MemoryStream())
            {
                PngBitmapEncoder encoder = new PngBitmapEncoder();
                encoder.Interlace = PngInterlaceOption.On;
                encoder.Frames.Add(BitmapFrame.Create(bmpSrc));
                encoder.Save(memoryStream);
         
                BitmapImage imageSource = new BitmapImage();
                imageSource.BeginInit();
                imageSource.StreamSource = memoryStream;
                imageSource.EndInit();
                return imageSource;
            }
        }

        public static void ToColorBitmapDQ()
        {
            //<summary>
            // Stores RGB stream frames from memoryStrean to Disk.
            //</summary>
            ConcurrentQueue<MemoryStream> cq = MainWindow.RGBConcurrentQueue;
            Console.WriteLine("Initialized");
            while (true)
            {
                if (!cq.IsEmpty)
                {
                    try
                    {
                        Console.WriteLine("Total number of items in the RGB concurrent queue: " + cq.Count);
                        MemoryStream bmps;
                        bool check = cq.TryDequeue(out bmps);

                        // MainWindow.bitmapRGB=new WriteableBitmap(bmps);
                        if (check) { 
                        long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
                        FileStream stream = new FileStream("C:/temp/rgb/" + milliseconds + "temp.wmp", FileMode.Create);
                            bmps.WriteTo(stream);
                            stream.Close();
                            bmps.Close();
                            qual += 0.00003F;
                            saveTimeStamp("time_rgb", milliseconds);
                        }
                    }
                    catch (Exception e) { Thread.Sleep(100);
                        Console.WriteLine("RGB Queue exception. Total number of items in the concurrent queue: " + cq.Count);
                    }                    
                }
                else {
                    qual = 1;
                    if (!MainWindow.running) { break; }
                    Thread.Sleep(500);
                }
            }           
        }
    }
}
