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
    class DepthRecord
    {
        private static String folderName = "C:/temp";

        public ImageSource ToDepthBitmap(DepthFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            ushort minDepth = frame.DepthMinReliableDistance;
            ushort maxDepth = frame.DepthMaxReliableDistance;
            ushort[] depthData = new ushort[width * height];
            byte[] pixelData = new byte[width * height * (PixelFormats.Bgr32.BitsPerPixel + 7) / 8];
            frame.CopyFrameDataToArray(depthData);
            int colorIndex = 0;
            for (int depthIndex = 0; depthIndex < depthData.Length; ++depthIndex)
            {
                ushort depth = depthData[depthIndex];
                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);
                pixelData[colorIndex++] = intensity; // Blue
                pixelData[colorIndex++] = intensity; // Green
                pixelData[colorIndex++] = intensity; // Red
                ++colorIndex;
            }

            var format = PixelFormats.Bgr32;
            int stride = width * format.BitsPerPixel / 8;
            //Testing Block
            var temp = BitmapSource.Create(width, height, 96, 96, format, null, pixelData, stride);
            long milliseconds = DateTime.Now.Second;
            FileStream stream = new FileStream("C:/temp/depth/" + milliseconds + "temp.tif", FileMode.Create);
            TiffBitmapEncoder encoder = new TiffBitmapEncoder();
            //TextBlock myTextBlock = new TextBlock();
            // myTextBlock.Text = "Codec Author is: " + encoder.CodecInfo.Author.ToString();
            encoder.Frames.Add(BitmapFrame.Create(temp));
            //MessageBox.Show(myPalette.Colors.Count.ToString());
            encoder.Save(stream);
            stream.Close();
            //End of Testing Block
            return BitmapSource.Create(width, height, 96, 96, format, null, pixelData, stride);
        }
        
        private static void saveTimeStamp(String loc, long stamp)
        {
            string filePath = folderName + ".txt";
            using (StreamWriter timesw = File.AppendText(filePath))
            {
                timesw.WriteLine(stamp);
                timesw.Close();
            }
        }

        public void ToDepthBitmapQ(int width,int height,ushort minDepth,ushort maxDepth,ushort[] depthData)
        {          
            byte[] pixelData = new byte[width * height * (PixelFormats.Bgr32.BitsPerPixel + 7) / 8];
            
            int colorIndex = 0;
            for (int depthIndex = 0; depthIndex < depthData.Length; ++depthIndex)
            {
                ushort depth = depthData[depthIndex];
                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);
                pixelData[colorIndex++] = intensity; // Blue
                pixelData[colorIndex++] = intensity; // Green
                pixelData[colorIndex++] = intensity; // Red
                ++colorIndex;
            }

            var format = PixelFormats.Bgr32;
            int stride = width * format.BitsPerPixel / 8;

            BitmapSource temp = BitmapSource.Create(width, height, 96, 96, format, null, pixelData, stride);
            temp.Freeze();
            MainWindow.DepthConcurrentQueue.Enqueue(temp);
            
           
        }

        public static void ToDepthBitmapDQ()
        {


            ConcurrentQueue<BitmapSource> cq = MainWindow.DepthConcurrentQueue;

            //Console.WriteLine("Initialized");
            while (true)
            {

                if (!cq.IsEmpty)
                {
                    try
                    {
                        Console.WriteLine("Total number of items in the Depth concurrent queue: " + cq.Count);
                        BitmapSource frame;
                        cq.TryDequeue(out frame);
                        //start

                        long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
                        FileStream stream = new FileStream("C:/temp/depth/" + milliseconds + "temp.tif", FileMode.Create);
                        TiffBitmapEncoder encoder = new TiffBitmapEncoder();
                       
                        encoder.Frames.Add(BitmapFrame.Create(frame));
                       
                        encoder.Save(stream);
                        stream.Close();
                        //end


                        saveTimeStamp("time_depth", milliseconds);
                    }
                    catch (Exception e) { Thread.Sleep(100);
                        Console.WriteLine("Depth Queue exception. Total number of items in the concurrent queue: " + cq.Count); }


                }
                else
                {
                    if (!MainWindow.running) { break; }
                    Thread.Sleep(500);
                }



            }


        }
    }
}
