//------------------------------------------------------------------------------
// <copyright>
// Made by Priyank Trived for Texas State University
//     The Kinect for Windows APIs used here are preliminary and subject to change
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace KinectRecord
{
    using System;
    using System.ComponentModel;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Drawing.Imaging;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Collections.Concurrent;
    using System.Threading.Tasks;
    using System.Threading;  

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

      public static ConcurrentQueue<MemoryStream> RGBConcurrentQueue = new ConcurrentQueue<MemoryStream>();
        public static ConcurrentQueue<BitmapSource> DepthConcurrentQueue = new ConcurrentQueue<BitmapSource>();
        public static Boolean running = true;
        private static Color RED = Color.FromRgb(255, 0, 0);
        private static Color GREEN = Color.FromRgb(0, 255, 0);
        private static Color BLUE = Color.FromRgb(0, 0, 255);
        private static Color YELLOW = Color.FromRgb(255, 255, 0);
        private static Color ORANGE = Color.FromRgb(255, 100, 0);
        private int cnt = 0;

        /// <summary>
        /// Recording options. Related to button use
        /// </summary>
        private Boolean recButton, skelRecO, rgbRecO, depthRecO, handLassO, stopTimer = false;

        /// <summary>
        /// Get the skeleton directory. Define out directory locations
        /// </summary>
        private String skelFolder, depthFolder, imageFolder = "";

        /// <summary>
        /// Main folder to hold all files - If the user does not provide a name we call it the deafult
        /// </summary>
        private static String folderName = "C:/temp";

        /// <summary>
        /// Cheap way of downsampling
        /// </summary>
        private int frameCountRGB = 1;

        /// <summary>
        /// The default downsampling rate. Default is 5.
        /// </summary>
        private int downSample = 5;

        /// <summary>
        /// Size of the RGB pixel in the bitmap  - RGB ooutput
        /// </summary>
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Hand size
        /// </summary>
        private const double HandSize = 50;

        /// <summary>
        /// Thickness of joint lines
        /// </summary>
        private const double JointThinkness = 20;

        /// <summary>
        /// Thickness of clip edge recetangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Vrush used for drawing hands that are currently tracked as open
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso - so a pointer
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently trackeed
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used to draw joints that are infered
        /// </summary>
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used to draw tracked joints
        /// </summary>
        private Pen trackedBonePen = new Pen(Brushes.Green, 10);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>
        private readonly Pen inferredBonePen = new Pen(Brushes.Green, 1);

        /// <summary>
        /// The draw group for rending the entire body output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Draw image that will be fed back to the user - overlay
        /// </summary>
        private DrawingImage imageSourceSkeleton;

        /// <summary>
        /// coordinate mapper to map from one point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Array for tracked bodies
        /// </summary>
        private Body[] bodies = null;
        ///  <summary>
        /// Depth width
        /// </summary>
        private int displayWidth;
        ///  <summary
        /// Depth height
        /// </summary>
        private int displayHeight;
        /// <summary>
        /// Reader for body frames - skeleton
        /// </summary>
        private BodyFrameReader readerSkel = null;
        /// <summary>
        /// Size of the RGB pixel in the bitmap - depth
        /// </summary>
        private readonly int cbytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        /// <summary>
        /// Intermediate storage for receiving frame data from the sensor - depth
        /// </summary>
        private ushort[] frameData = null;
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;
        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader readerRGB = null;
        /// <summary>
        /// Reader for color frames
        /// </summary>
        private DepthFrameReader readerDepth = null;
        /// <summary>
        /// Bitmap to display for each stream
        /// </summary>
        public static WriteableBitmap bitmapDepth = null;

        public static WriteableBitmap bitmapRGB = null;
        /// <summary>
        /// Intermediate storage for receiving frame data from the sensor for each stream
        /// </summary>
        private byte[] pixels, pixelsDepth = null;

        /// <summary>
        /// The time of the first frame received
        /// </summary>
        private long startTime = 0;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Next time to update FPS/frame time status
        /// </summary>
        private DateTime nextStatusUpdate = DateTime.MinValue;

        /// <summary>
        /// Number of frames since last FPS/frame time status
        /// </summary>
        private uint framesSinceUpdate = 0;

        /// <summary>
        /// Timer for FPS calculation
        /// </summary>
        private Stopwatch stopwatch = null;

        /// <summary>
        /// The timer counter
        /// </summary>
        public int count = 0;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {

            
            // create a stopwatch for FPS calculation
            this.stopwatch = new Stopwatch();

            // for Alpha, one sensor is supported
            this.kinectSensor = KinectSensor.GetDefault();

            if (this.kinectSensor != null)
            {
                this.coordinateMapper = this.kinectSensor.CoordinateMapper;

                // open the sensor
                this.kinectSensor.Open();

                
                //obtain descriptions for each stream
                FrameDescription frameDescriptionRGB = this.kinectSensor.ColorFrameSource.FrameDescription; //RGB
                FrameDescription frameDescriptionDepth = this.kinectSensor.DepthFrameSource.FrameDescription; //depth
                this.bodies = new Body[this.kinectSensor.BodyFrameSource.BodyCount];


                //Obtain dimensions and skeleton
                this.displayWidth = frameDescriptionRGB.Width;
                this.displayHeight = frameDescriptionRGB.Height;

                // open the reader for the color frames
                this.readerRGB = this.kinectSensor.ColorFrameSource.OpenReader(); //RGB
                this.readerDepth = this.kinectSensor.DepthFrameSource.OpenReader(); //Depth
                this.readerSkel = this.kinectSensor.BodyFrameSource.OpenReader(); // Skeleton

                // allocate space to put the pixels being received
                this.pixels = new byte[frameDescriptionRGB.Width * frameDescriptionRGB.Height * this.bytesPerPixel]; //RGB

                this.frameData = new ushort[frameDescriptionDepth.Width * frameDescriptionDepth.Height]; //Depth
                this.pixelsDepth = new byte[frameDescriptionDepth.Width * frameDescriptionDepth.Height * this.cbytesPerPixel]; //Depth

                // create the bitmap to display RGB and depth
                bitmapRGB = new WriteableBitmap(frameDescriptionRGB.Width, frameDescriptionRGB.Height, 96.0, 96.0, PixelFormats.Bgr32, null); //RGB
                bitmapDepth = new WriteableBitmap(frameDescriptionDepth.Width, frameDescriptionDepth.Height, 96.0, 96.0, PixelFormats.Bgr32, null); //Depth

            }
            else
            {

            }
            //Create the drawing group to use for drawing - needs tidying up
            this.drawingGroup = new DrawingGroup();
            //create an image source of the skelton
            this.imageSourceSkeleton = new DrawingImage(this.drawingGroup);
            // use the window object as the view model in this simple example
            this.DataContext = this;
            // initialize the components (controls) of the window
            this.InitializeComponent();
            //Event Handles
            startRec.Click += Button_Clicks_EventHandler;
        }
        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;
        /// <summary>
        /// Gets the bitmap to display for RGB image
        /// </summary>
        public ImageSource ImageRGB
        {
            get
            {
                return bitmapRGB;
            }
        }
        /// <summary>
        /// Gets the bitmap to display for skeleton image
        /// </summary>
        public ImageSource imageSkel
        {
            get
            {
                return this.imageSourceSkeleton;
            }
        }
        // ------------ Pt Code || Depth to Bitmap image --------------------//
        
        
        /// <summary>
        /// Execute start up tasks - intialise the reader search e events
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {

            Thread RGBR = new Thread(new ThreadStart(RGBRecord.ToColorBitmapDQ));
            RGBR.Start();
            Thread DepthR = new Thread(new ThreadStart(DepthRecord.ToDepthBitmapDQ));
            DepthR.Start();
            if (this.readerRGB != null)
            {
                this.readerRGB.FrameArrived += this.Reader_FrameArrivedColour; //RGB
            }
            if (this.readerDepth != null)
            {
                this.readerDepth.FrameArrived += this.Reader_FrameArrivedDepth; //Depth
            }
            if (this.readerSkel != null)
            {
                this.readerSkel.FrameArrived += this.Reader_FrameArrivedSkel; //skeleton
            }
        }
        /// <summary>
        /// Execute shutdown (dispose) tasks for each stream
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            running = false;
            if (this.readerRGB != null)
            {
                this.readerRGB.Dispose();
                this.readerRGB = null;
            }
            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
            if (this.readerDepth != null)
            {
                this.readerDepth.Dispose();
                this.readerDepth = null;
            }
            if (this.readerSkel != null)
            {
                this.readerSkel.Dispose();
                this.readerSkel = null;
            }
        }
        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrivedColour(object sender, ColorFrameArrivedEventArgs e)
        {
            ColorFrameReference frameReference = e.FrameReference;
            if (this.startTime == 0)
            {
                this.startTime = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            }
            try
            {
                ColorFrame frame = frameReference.AcquireFrame();
                if (frame != null)
                {
                    // ColorFrame is IDisposable
                    using (frame)
                    {
                        this.framesSinceUpdate++;
                        FrameDescription frameDescription = frame.FrameDescription;
                        // update status unless last message is sticky for a while
                        if (DateTime.Now >= this.nextStatusUpdate)
                        {
                            // calcuate fps based on last frame received
                            int fps = 0;
                            if (this.stopwatch.IsRunning)
                            {
                                this.stopwatch.Stop();
                                fps = (int)(this.framesSinceUpdate / this.stopwatch.Elapsed.TotalSeconds) / 2;
                                label2.Content = "FPS: " + fps;
                                this.stopwatch.Reset();
                            }
                            long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
                            this.nextStatusUpdate = DateTime.Now + TimeSpan.FromSeconds(1);
                        }
                        if (!this.stopwatch.IsRunning)
                        {
                            this.framesSinceUpdate = 0;
                            this.stopwatch.Start();
                        }
                        // verify data and write the new color frame data to the display bitmap
                        if ((frameDescription.Width == bitmapRGB.PixelWidth) && (frameDescription.Height == bitmapRGB.PixelHeight))
                        {
                            if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
                            {
                                frame.CopyRawFrameDataToArray(this.pixels);
                            }
                            else
                            {
                                frame.CopyConvertedFrameDataToArray(this.pixels, ColorImageFormat.Bgra);
                            }

                            int width = frame.FrameDescription.Width;
                            int height = frame.FrameDescription.Height;
                            int bpp = (int)frame.FrameDescription.BytesPerPixel;
                            var format = ColorImageFormat.Rgba;
                            byte[] pixelData = new byte[height*width*bpp];
                            pixelData=this.pixels;
                            int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;
                            int writebpp = this.bytesPerPixel;
                            bitmapRGB.WritePixels(
                                  new Int32Rect(0, 0, frameDescription.Width, frameDescription.Height),
                                  //new Int32Rect(0, 0, 2500, frameDescription.Height),
                                  this.pixels,
                                  frameDescription.Width * this.bytesPerPixel,
                                  0);


                            //save rgb if it meets the criteria
                            if (rgbRecO == true)// && frameCountRGB == downSample)
                            {                             
                                Task.Factory.StartNew(() =>
                               {                                   
                                    new RGBRecord().ToColorBitmapQ(height,width,bpp,format,pixelData);                                  
                                });                                                     
                                frameCountRGB = 0;                                
                            }

                            if (frameCountRGB == downSample)
                            {
                                frameCountRGB = 0;
                            }
                            frameCountRGB = frameCountRGB + 1;
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                // ignore if the frame is no longer available
            }
        }

        public static void showRGB(BitmapSource bmps)
        {
            bitmapRGB = new WriteableBitmap(bmps);

        }

            private void performance_Checked(object sender, RoutedEventArgs e)
        {
            if (performance.IsChecked.HasValue)
            {
                rgb_view.Visibility = Visibility.Hidden;
                skel_view.Visibility = Visibility.Hidden;
                depth_view.Visibility = Visibility.Hidden;

            }
            else if (!performance.IsChecked.HasValue)
            {
                rgb_view.Visibility = Visibility.Visible;
                skel_view.Visibility = Visibility.Visible;
                depth_view.Visibility = Visibility.Visible;
            }
        }

        private void performance_Unchecked(object sender, RoutedEventArgs e)
        {
            rgb_view.Visibility = Visibility.Visible;
            skel_view.Visibility = Visibility.Visible;
            depth_view.Visibility = Visibility.Visible;
        }
        public ImageSource ImageDepth
        {
            get
            {
                return bitmapDepth;
            }
        }
        /// <summary>
        /// Gets or sets the current status text to displayin the footer
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrivedDepth(object sender, DepthFrameArrivedEventArgs e)
        {
            DepthFrameReference frameReference = e.FrameReference;
            try
            {
                DepthFrame frame = frameReference.AcquireFrame();
                if (frame != null)
                {
                    // DepthFrame is IDisposable
                    using (frame)
                    {
                        FrameDescription frameDescription = frame.FrameDescription;
                        // verify dazta and write the new depth frame data to the display bitmap
                        if (((frameDescription.Width * frameDescription.Height) == this.frameData.Length) &&
                            (frameDescription.Width == bitmapDepth.PixelWidth) && (frameDescription.Height == bitmapDepth.PixelHeight))
                        {
                            // Copy the pixel data from the image to a temporary array
                            frame.CopyFrameDataToArray(this.frameData);
                            // Get the min and max reliable depth for the current frame
                            ushort minDepth = frame.DepthMinReliableDistance;
                            ushort maxDepth = frame.DepthMaxReliableDistance;
                            // Convert the depth to RGB
                            int colorPixelIndex = 0;
                            for (int i = 0; i < this.frameData.Length; ++i)
                            {
                                // Get the depth for this pixel
                                ushort depth = this.frameData[i];
                                
                                // To convert to a byte, we're discarding the most-significant
                                // rather than least-significant bits.
                                // We're preserving detail, although the intensity will "wrap."
                                // Values outside the reliable depth range are mapped to 0 (black).
                                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);
                                // Write out blue byte
                                this.pixelsDepth[colorPixelIndex++] = intensity;
                                // Write out green byte
                                this.pixelsDepth[colorPixelIndex++] = intensity;
                                // Write out red byte
                                this.pixelsDepth[colorPixelIndex++] = intensity;
                                // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                                // If we were outputting BGRA, we would write alpha here.
                                ++colorPixelIndex;
                                // Save depth to file
                            }
                            bitmapDepth.WritePixels(
                                new Int32Rect(0, 0, frameDescription.Width, frameDescription.Height),
                                this.pixelsDepth,
                                frameDescription.Width * this.cbytesPerPixel,
                                0);
                            //Save the depth data using a binary stream if the option is selected
                            if (depthRecO == true)
                            {
                                long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
                                String photolocation = milliseconds + ".tif";
                                string filePath = depthFolder + '/' + "depth" + photolocation;
                                int width = frame.FrameDescription.Width;
                                int height = frame.FrameDescription.Height;                                                             
                                ushort[] depthData = new ushort[width * height];
                                frame.CopyFrameDataToArray(depthData);
                                new DepthRecord().ToDepthBitmapQ( width,  height,  minDepth, maxDepth,depthData);                                
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                // ignore if the frame is no longer available
            }
        }

        private void record_length_check_Checked(object sender, RoutedEventArgs e)
        {
            recording_length_panel.Visibility = Visibility.Visible;
            recLength.Text = "10";
        }

        private void record_length_check_Unchecked(object sender, RoutedEventArgs e)
        {
            recording_length_panel.Visibility = Visibility.Hidden;
            recLength.Text = String.Empty;
        }

        private void skelCheck_Checked(object sender, RoutedEventArgs e)
        {
            handLass.Visibility = Visibility.Visible;
        }

        private void skelCheck_Unchecked(object sender, RoutedEventArgs e)
        {
            handLass.Visibility = Visibility.Hidden;
        }

        /// <summary>
        /// Handles the arrival of each skeleton frame
        /// </summary?
        private void Reader_FrameArrivedSkel(object sender, BodyFrameArrivedEventArgs e)
        {
            BodyFrameReference frameReference = e.FrameReference;
            try
            {
                BodyFrame frame = frameReference.AcquireFrame();
                if (frame != null)
                {
                    // BodyFrame is IDisposable
                    using (frame)
                    {
                        this.framesSinceUpdate++;
                        using (DrawingContext dc = this.drawingGroup.Open())
                        {
                            // Draw a transparent background to set the render size
                            dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                            // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                            // As long as those body objects are not disposed and not set to null in the array,
                            // those body objects will be re-used.
                            frame.GetAndRefreshBodyData(this.bodies);
                            foreach (Body body in this.bodies)
                            {
                                if (body.IsTracked)
                                {
                                    //Joint shoulder = body.Joints[JointType.ShoulderLeft];
                                    //Joint elbow = body.Joints[JointType.ElbowLeft];
                                    //Joint wrist = body.Joints[JointType.WristLeft];
                                    //double angle = elbow.Angle(shoulder, wrist);
                                    //double angle = elbow.Angle(shoulder, wrist);
                                    //angle_lbl.Content = "FPS: " + angle;

                                    IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                                    // convert the joint points to depth (display) space
                                    Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                                    foreach (JointType jointType in joints.Keys)
                                    {
                                        ColorSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(joints[jointType].Position);
                                        jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                    }
                                    this.DrawBody(joints, jointPoints, dc);
                                    this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                                    this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                                    //save the skel coords if the option is selected
                                    if (skelRecO == true)
                                    {
                                        long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
                                        Task.Factory.StartNew(() =>
                                        {
                                            saveSkelInformation(milliseconds, body);
                                            saveTimeStamp("time_skel", milliseconds);
                                        });
                                        //Save current frame time stamp
                                    }
                                }
                            }
                            // prevent drawing outside of our render area
                            this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                // ignore if the frame is no longer available
            }
        }


        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext,Nullable<Color> color)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];
            if (color == null) color = Color.FromRgb(0, 255, 0); ;
            SolidColorBrush drawBrush = new SolidColorBrush((Color)color);
            

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == TrackingState.Inferred &&
                joint1.TrackingState == TrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen.Clone();
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {

                drawPen = this.trackedBonePen.Clone();
                
            }
            drawPen.Brush = drawBrush;
            
            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }


        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext)
        {
            // Draw the bones

            // Torso
            this.DrawBone(joints, jointPoints, JointType.Head, JointType.Neck, drawingContext,null);
            this.DrawBone(joints, jointPoints, JointType.Neck, JointType.SpineShoulder, drawingContext,null);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.SpineMid, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.SpineMid, JointType.SpineBase, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.ShoulderRight, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.ShoulderLeft, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.SpineBase, JointType.HipRight, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.SpineBase, JointType.HipLeft, drawingContext, null);

            // Right Arm
            this.DrawBone(joints, jointPoints, JointType.ShoulderRight, JointType.ElbowRight, drawingContext, BLUE);
            this.DrawBone(joints, jointPoints, JointType.ElbowRight, JointType.WristRight, drawingContext, BLUE);
            this.DrawBone(joints, jointPoints, JointType.WristRight, JointType.HandRight, drawingContext, BLUE);
            this.DrawBone(joints, jointPoints, JointType.HandRight, JointType.HandTipRight, drawingContext, BLUE);
            this.DrawBone(joints, jointPoints, JointType.WristRight, JointType.ThumbRight, drawingContext, BLUE);

            // Left Arm
            this.DrawBone(joints, jointPoints, JointType.ShoulderLeft, JointType.ElbowLeft, drawingContext, RED);
            this.DrawBone(joints, jointPoints, JointType.ElbowLeft, JointType.WristLeft, drawingContext, RED);
            this.DrawBone(joints, jointPoints, JointType.WristLeft, JointType.HandLeft, drawingContext, RED);
            this.DrawBone(joints, jointPoints, JointType.HandLeft, JointType.HandTipLeft, drawingContext, RED);
            this.DrawBone(joints, jointPoints, JointType.WristLeft, JointType.ThumbLeft, drawingContext, RED);

            // Right Leg
            this.DrawBone(joints, jointPoints, JointType.HipRight, JointType.KneeRight, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.KneeRight, JointType.AnkleRight, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.AnkleRight, JointType.FootRight, drawingContext, null);

            // Left Leg
            this.DrawBone(joints, jointPoints, JointType.HipLeft, JointType.KneeLeft, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.KneeLeft, JointType.AnkleLeft, drawingContext, null);
            this.DrawBone(joints, jointPoints, JointType.AnkleLeft, JointType.FootLeft, drawingContext, null);

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                SolidColorBrush drawBrush = null;                
                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = (SolidColorBrush)this.trackedJointBrush.Clone();
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = (SolidColorBrush)this.inferredJointBrush.Clone();
                }
                if (drawBrush != null)
                {

                    if(jointType.ToString().Contains("Right"))drawBrush.Color = BLUE;
                    if (jointType.ToString().Contains("Left")) drawBrush.Color = RED;
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThinkness, JointThinkness);
                }
            }
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        ///  Main recording oporator. Handles the button event click
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Button_Clicks_EventHandler(object sender, EventArgs e)
        {
            if (sender == startRec)
            {
                //Shall we allow the recording?
                if (recButton == true)
                {
                    //Stop all recoriding
                    startRec.Content = "Record";
                    if (!String.IsNullOrEmpty(recLength.Text))
                    {
                        labelTimer.Visibility = Visibility.Hidden;
                        countdownBox.Visibility = Visibility.Hidden;
                    }
                    
                    recButton = false;
                    skelRecO = false;
                    rgbRecO = false;
                    depthRecO = false;
                    handLassO = false;
                    stopTimer = true;
                    count = 1;
                    this.countdownBox.Content = count;

                }
                else // Let's start recording
                {
                    if (!(bool)rgbCheck.IsChecked && !(bool)depthCheck.IsChecked && !(bool)skelCheck.IsChecked) 
                    {
                        MessageBox.Show("Please select at-least one Kinect stream to begin recording.");
                        return;
                    }
                    startRec.Content = "Stop";
                    if (!String.IsNullOrEmpty(recLength.Text))
                    {
                        labelTimer.Visibility = Visibility.Visible;
                        countdownBox.Visibility = Visibility.Visible;
                    }
                    recButton = true;
                    stopTimer = false;
                    //GENERATE FOLDERS
                    //Get the folder name. if blank lets set a default folder.                                                            
                    //Generate folders - might as well create them all
                    //folderName = actionName.Text;
                        Directory.CreateDirectory(folderName);
                        Directory.CreateDirectory(folderName + "/skel");
                        Directory.CreateDirectory(folderName + "/depth");
                        Directory.CreateDirectory(folderName + "/rgb");

                    
                    if (recLength.Text.Trim().Length > 0) // if a time length is provided
                    {
                        int counter = Convert.ToInt32(this.recLength.Text);
                        startTimer(counter, TimeSpan.FromSeconds(1), cur => counter = cur);
                    }

                    skelFolder = folderName + "/skel";
                    depthFolder = folderName + "/depth";
                    imageFolder = folderName + "/rgb";

                    //START DIFFERENT RECORDINGS
                    //Record the skeleton
                    if (skelCheck.IsChecked == true)
                    {
                        //Start skeleton recording
                        skelRecO = true;
                        //Lets see if we want to record specific skeleton stuff
                        if (handLass.IsChecked == true)
                        {
                            handLassO = true;
                        }

                        //GENERATE FILE
                        System.IO.File.Create(folderName + "/skel.txt").Dispose();
                    }

                    if (depthCheck.IsChecked == true)
                    {
                        //Start depth recording
                        depthRecO = true;

                        //GENERATE TIME STAMP FILE
                        System.IO.File.Create(folderName + "/time_depth.txt").Dispose();

                    }

                    if (rgbCheck.IsChecked == true)
                    {
                        rgbRecO = true;

                        //GENERATE TIMESTAMP FILE
                        System.IO.File.Create(folderName + "/time_rgb.txt").Dispose();

                    }

                }
            }

        }

        /// <summary>
        /// Start the event handler for recording.
        /// </summary>
        /// <param name="count"></param>
        /// <param name="interval"></param>
        /// <param name="ts"></param>
        private void startTimer(int count, TimeSpan interval, Action<int> ts)
        {
            var dt = new System.Windows.Threading.DispatcherTimer();
            dt.Interval = interval;
            dt.Tick += (_, a) =>
            {
                if (count-- == 0)
                {
                    //when count hits 0, stop the recording
                    startRec.Content = "Record";
                    recButton = false;
                    skelRecO = false;
                    rgbRecO = false;
                    depthRecO = false;
                    handLassO = false;
                    stopTimer = true;
                    dt.Stop();
                }
                else if (stopTimer == true)
                {
                    dt.Stop();

                }
                else
                {
                    this.countdownBox.Content = count;
                    ts(count);
                }

            };
            ts(count);
            dt.Start();
        }

        /// <summary>
        /// Save frame for each data stream
        /// </summary>
        /// <param name="loc"></param>
        /// <param name="stamp"></param>
        private void saveTimeStamp(String loc, long stamp)
        {
            string filePath = folderName + ".txt";
            using (StreamWriter timesw = File.AppendText(filePath))
            {
                timesw.WriteLine(stamp);
                timesw.Close();
            }
        }

        /// <summary>
        /// Save the coordinates in depth space of the skeleton
        /// </summary>
        /// <param name="timeStamp"></param>
        /// <param name="body"></param>

        private void saveSkelInformation(long timeStamp, Body body)
        {

            //string filePath = skelFolder + '/' + "skel" + ".txt";
            string filePath = skelFolder + '/' + "skel" + ".txt";

            StreamWriter cooStream = new StreamWriter(filePath, true);
            //if (cnt == 0)
            //    Console.Write("nope");
            //{
            //    cooStream.Write("," + "SpineBase.Stat" + "," + "SpineBase.X" + "," + "SpineBase.Y" + "," + "SpineBase.Z" + "," + "SpineBase.Col.X" + "," + "SpineBase.Col.Y"
            //                 + "," + "SpineMid.State" + "," + "SpineMid.X" + "," + "SpineMid.Y" + "," + "SpineMid.Z" + "," + "SpineMid.Col.X" + "," + "SpineMid.Col.Y"
            //                 + "," + "Neck.Stat" + "," + "Neck.X" + "," + "Neck.Y" + "," + "Neck.Z" + "," + "Neck.Col.X" + "," + "Neck.Col.Y"
            //                 + "," + "Head.Stat" + "," + "Head.X" + "," + "Head.Y" + "," + "Head.Z" + "," + "Head.Col.X" + "," + "Head.Col.Y"
            //                 + "," + "ShoulderLeft.Stat" + "," + "ShoulderLeft.X" + "," + "ShoulderLeft.Y" + "," + "ShoulderLeft.Z" + "," + "ShoulderLeft.Col.X" + "," + "ShoulderLeft.Col.Y"
            //                 + "," + "ElbowLeft.Stat" + "," + "ElbowLeft.X" + "," + "ElbowLeft.Y" + "," + "ElbowLeft.Z" + "," + "ElbowLeft.Col.X" + "," + "ElbowLeft.Col.Y"
            //                 + "," + "WristLeft.Stat" + "," + "ElbowLeft.X" + "," + "ElbowLeft.Y" + "," + "ElbowLeft.Z" + "," + "ElbowLeft.Col.X" + "," + "ElbowLeft.Col.Y"
            //                 + "," + "HandLeft.Stat" + "," + "HandLeft.X" + "," + "HandLeft.Y" + "," + "HandLeft.Z" + "," + "HandLeft.Col.X" + "," + "HandLeft.Col.Y"
            //                 + "," + "ShoulderRight.Stat" + "," + "ShoulderRight.X" + "," + "ShoulderRight.Y" + "," + "ShoulderRight.Z" + "," + "ShoulderRight.Col.X" + "," + "ShoulderRight.Col.Y"
            //                 + "," + "ElbowRight.Stat" + "," + "ElbowRight.X" + "," + "ElbowRight.Y" + "," + "ElbowRight.Z" + "," + "ElbowRight.Col.X" + "," + "ElbowRight.Col.Y"
            //                 + "," + "WristRight.Stat" + "," + "ElbowRight.X" + "," + "ElbowRight.Y" + "," + "ElbowRight.Z" + "," + "ElbowRight.Col.X" + "," + "ElbowRight.Col.Y"
            //                 + "," + "HandRight.Stat" + "," + "HandRight.X" + "," + "HandRight.Y" + "," + "HandRight.Z" + "," + "HandRight.Col.X" + "," + "HandRight.Col.Y"
            //                 + "," + "hipLeft.Stat" + "," + "hipLeft.X" + "," + "hipLeft.Y" + "," + "hipLeft.Z" + "," + "hipLeft.Col.X" + "," + "hipLeft.Col.Y"
            //                 + "," + "KneeLeft.Stat" + "," + "KneeLeft.X" + "," + "KneeLeft.Y" + "," + "KneeLeft.Z" + "," + "KneeLeft.Col.X" + "," + "KneeLeft.Col.Y"
            //                 + "," + "AnkleLeft.Stat" + "," + "AnkleLeft.X" + "," + "AnkleLeft.Y" + "," + "AnkleLeft.Z" + "," + "AnkleLeft.Col.X" + "," + "AnkleLeft.Col.Y"
            //                 + "," + "FootLeft.Stat" + "," + "FootLeft.X" + "," + "FootLeft.Y" + "," + "FootLeft.Z" + "," + "FootLeft.Col.X" + "," + "FootLeft.Col.Y"
            //                 + "," + "hipRight.Stat" + "," + "hipRight.X" + "," + "hipRight.Y" + "," + "hipRight.Z" + "," + "hipRight.Col.X" + "," + "hipRight.Col.Y"
            //                 + "," + "KneeRight.Stat" + "," + "KneeRight.X" + "," + "KneeRight.Y" + "," + "KneeRight.Z" + "," + "KneeRight.Col.X" + "," + "KneeRight.Col.Y"
            //                 + "," + "AnkleRight.Stat" + "," + "AnkleRight.X" + "," + "AnkleRight.Y" + "," + "AnkleRight.Z" + "," + "AnkleRight.Col.X" + "," + "AnkleRight.Col.Y"
            //                 + "," + "FootRight.Stat" + "," + "FootRight.X" + "," + "FootRight.Y" + "," + "FootRight.Z" + "," + "FootRight.Col.X" + "," + "FootRight.Col.Y"
            //                 + "," + "SpineShoulder.Stat" + "," + "SpineShoulder.X" + "," + "SpineShoulder.Y" + "," + "SpineShoulder.Z" + "," + "SpineShoulder.Col.X" + "," + "SpineShoulder.Col.Y"
            //                 + "," + "handTipLeft.Stat" + "," + "handTipLeft.X" + "," + "handTipLeft.Y" + "," + "handTipLeft.Z" + "," + "handTipLeft.Col.X" + "," + "handTipLeft.Col.Y"
            //                 + "," + "ThumbLeft.Stat" + "," + "ThumbLeft.X" + "," + "ThumbLeft.Y" + "," + "ThumbLeft.Z" + "," + "ThumbLeft.Col.X" + "," + "ThumbLeft.Col.Y"
            //                 + "," + "handTipRight.Stat" + "," + "handTipRight.X" + "," + "handTipRight.Y" + "," + "handTipRight.Z" + "," + "handTipRight.Col.X" + "," + "handTipRight.Col.Y"
            //                 + "," + "ThumbRight.Stat" + "," + "ThumbRight.X" + "," + "ThumbRight.Y" + "," + "ThumbRight.Z" + "," + "ThumbRight.Col.X" + "," + "ThumbRight.Col.Y"
            //                 );
            //}

            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
           
            foreach (JointType jointType in joints.Keys)
            {
                //Camera space points
                ColorSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(joints[jointType].Position);

                //RGB space
                //camera spac coord sx 3
                cooStream.Write("," + joints[jointType].TrackingState + "," + joints[jointType].Position.X + "," + joints[jointType].Position.Y + "," + joints[jointType].Position.Z + "," + depthSpacePoint.X + "," + depthSpacePoint.Y);

            }
            //End of Frame
            cooStream.WriteLine("\n");
            //If we want to record both hand loss and open hand
            if (handLassO == true)
            {
                string wrtLineData = "LeftHand " + body.HandLeftState + " RightHand " + body.HandRightState;
                cooStream.WriteLine(wrtLineData);

            }
            cnt++;
            cooStream.Close();
        }
        
    }
    
}