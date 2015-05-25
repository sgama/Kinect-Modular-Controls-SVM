﻿//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
namespace Microsoft.Samples.Kinect.ColorBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.Threading;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Drawing;
    using Microsoft.Kinect;
    using OpenCvSharp;
    using OpenCvSharp.CPlusPlus;
    using OpenCvSharp.Utilities;
    using OpenCvSharp.Extensions;
    using System.Collections.Generic;


    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : System.Windows.Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private CvSVM svm = null;

        private List<Shape> controls = new List<Shape>();

        private State state = State.DETECT_CONTROLS;

        private enum State
        {
            DETECT_CONTROLS,
            MAIN
        }

        public class Shape
        {
            public enum ShapeType
            {
                SQUARE,
                CIRCLE,
                SLIDER
            }
            public OpenCvSharp.CPlusPlus.Rect bounds;
            public ShapeType type;

            public Shape(ShapeType type, OpenCvSharp.CPlusPlus.Rect bounds)
            {
                this.type = type;
                this.bounds = bounds;
            }
        }

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            //load SVM
            this.svm = loadSVM();

            this.kinectSensor = KinectSensor.GetDefault(); // get the kinectSensor object

            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader(); // open the reader for the color frames
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;  // wire handler for frame arrival
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);// create the colorFrameDescription from the ColorFrameSource using Bgra format
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null); // create the bitmap to display

            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged; // set IsAvailableChanged event notifier

            this.kinectSensor.Open(); // open the sensor

            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText : Properties.Resources.NoSensorStatusText; // set the status text
            this.DataContext = this; // use the window object as the view model in this simple example
            this.InitializeComponent(); // initialize the components (controls) of the window

            if (!this.kinectSensor.IsAvailable)
            {
                loadFakeKinect();
            }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorFrameReader.IsPaused = true;

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            byte[] pixels = new byte[colorFrameDescription.Height * colorFrameDescription.Width * 4];
                            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
                            int stride = colorFrameDescription.Width * 4;
                            BitmapSource source = BitmapSource.Create(colorFrameDescription.Width, colorFrameDescription.Height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);

                            Bitmap bitmap;
                            using (MemoryStream outStream = new MemoryStream())
                            {
                                BitmapEncoder enc = new BmpBitmapEncoder();
                                enc.Frames.Add(BitmapFrame.Create(source));
                                enc.Save(outStream);
                                bitmap = new Bitmap(outStream);
                            }

                            switch (state)
                            {
                                case State.DETECT_CONTROLS:
                                    detectShapeCandidates(ref bitmap, false);
                                    break;
                                case State.MAIN:
                                    break;
                            }
                            
                            writeToBackBuffer(ConvertBitmap(bitmap));
                            bitmap.Dispose();
                           
                            
                        }

                        this.colorFrameReader.IsPaused = false;
                    }
                }
            }
        }

        private void writeToBackBuffer(BitmapSource source)
        {
            this.colorBitmap.Lock();
            int stride = source.PixelWidth * (source.Format.BitsPerPixel / 8); // Calculate stride of source
            byte[] data = new byte[stride * source.PixelHeight]; // Create data array to hold source pixel data
            source.CopyPixels(data, stride, 0); // Copy source image pixels to the data array

            // Write the pixel data to the WriteableBitmap.
            this.colorBitmap.WritePixels(new Int32Rect(0, 0, source.PixelWidth, source.PixelHeight), data, stride, 0);
            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, source.PixelWidth, source.PixelHeight));
            this.colorBitmap.Unlock();
        }

        private void detectShapeCandidates(ref Bitmap bitmap, Boolean saveShapes)
        {
            Debug.WriteLine("Running OpenCV");
            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
            Mat colorMat = BitmapConverter.ToMat(bitmap);
            MatOfDouble mu = new MatOfDouble();
            MatOfDouble sigma = new MatOfDouble();
            Cv2.MeanStdDev(colorMat, mu, sigma);
            double mean = mu.GetArray(0, 0)[0];
            mu.Dispose();
            sigma.Dispose();

            Mat greyMat = new Mat();
            Cv2.CvtColor(colorMat, greyMat, ColorConversion.BgraToGray, 0);
            greyMat = greyMat.GaussianBlur(new OpenCvSharp.CPlusPlus.Size(1, 1), 5, 5, BorderType.Default);
            greyMat = greyMat.Canny(0.5 * mean, 1.2 * mean, 3, true);

            Mat contourMat = new Mat(greyMat.Size(), colorMat.Type());
            greyMat.CopyTo(contourMat);
            var contours = contourMat.FindContoursAsArray(ContourRetrieval.List, ContourChain.ApproxSimple);

            this.controls.Clear();
            for (int j =0; j< contours.Length; j++)
            {
                var poly = Cv2.ApproxPolyDP(contours[j], 0.01 * Cv2.ArcLength(contours[j], true), true);
                int num = poly.Length;
     
                if (num >= 4 && num < 20)
                {
                    var color = Scalar.Blue;
                    var rect = Cv2.BoundingRect(poly);
                    
                    if (rect.Height < 20 || rect.Width < 20) continue;
                    if (saveShapes)
                    {
                        string path = Path.Combine(myPhotos, "shape_samples");
                        path = Path.Combine(path, "shape_sample_" + Path.GetRandomFileName() + ".png");
                        Mat shapeMat = preprocessShape(rect, greyMat);
                        Bitmap shape = shapeMat.ToBitmap();
                        shape.Save(path);
                        shape.Dispose();
                        shapeMat.Dispose();
                        continue;
                    }
                    if (svm != null)
                    {
                        Mat shapeMat = preprocessShape(rect, greyMat);
                        float shapeClass = classifyShape(shapeMat, svm);
                        if (shapeClass >= 0)
                        {
                            Shape shape = null;
                            switch ((int)shapeClass)
                            {
                                case 0:
                                    color = Scalar.Red;
                                    shape = new Shape(Shape.ShapeType.SQUARE, rect);
                                    break;
                                case 1:
                                    color = Scalar.Yellow;
                                    shape = new Shape(Shape.ShapeType.CIRCLE, rect);
                                    break;
                                case 2:
                                    color = Scalar.Green;
                                    shape = new Shape(Shape.ShapeType.SLIDER, rect);
                                    break;
                            }
                            Cv2.Rectangle(colorMat, rect, color, 2);
                            this.controls.Add(shape);
                        }
                    }
                    else {
                        Cv2.Rectangle(colorMat, rect, color, 2);
                    }
                }
            }
            

            bitmap = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(colorMat);
            colorMat.Dispose();
            greyMat.Dispose();
            contourMat.Dispose();
        }

        private Mat preprocessShape(OpenCvSharp.CPlusPlus.Rect shapeRect, Mat sceneMat)
        {
            var matRect = new OpenCvSharp.CPlusPlus.Rect(0, 0, sceneMat.Width, sceneMat.Height);
            shapeRect.Inflate((int)(shapeRect.Width * 0.1), (int)(shapeRect.Height * 0.1));
            shapeRect = shapeRect.Intersect(matRect);
            Mat shapeMat = sceneMat.SubMat(shapeRect);
            var size = new OpenCvSharp.CPlusPlus.Size(128, 128);
            shapeMat = shapeMat.Resize(size);
            return shapeMat;
        }

        private float classifyShape(Mat shape, CvSVM svm)
        {
            HOGDescriptor hog = new HOGDescriptor();
            hog.WinSize = new OpenCvSharp.CPlusPlus.Size(32, 32); // Set hog features here: winSize; blockSize; cellSize
            hog.BlockSize = new OpenCvSharp.CPlusPlus.Size(4, 4);
            hog.CellSize = new OpenCvSharp.CPlusPlus.Size(4, 4);
            hog.BlockStride = new OpenCvSharp.CPlusPlus.Size(2, 2);
            float[] features = hog.Compute(shape, new OpenCvSharp.CPlusPlus.Size(16, 16), new OpenCvSharp.CPlusPlus.Size(0, 0), null);
            Mat featureMat = new Mat(1, features.Length, MatType.CV_32FC1, features);
            float prediction = svm.Predict(featureMat.ToCvMat());
            return prediction;
        }

        private CvSVM loadSVM()
        {
            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
            string folderPath = Path.Combine(myPhotos, "shape_samples");

            CvSVM svm = new CvSVM();
            svm.Load(Path.Combine(folderPath, "trained_svm"));

            return svm;
        }

           

        public static BitmapSource ConvertBitmap(Bitmap source)
        {
            return System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(source.GetHbitmap(), IntPtr.Zero,
                          Int32Rect.Empty,
                          BitmapSizeOptions.FromEmptyOptions());
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;


        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
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
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.colorBitmap != null)
            {
                BitmapEncoder encoder = new PngBitmapEncoder(); // create a png bitmap encoder which knows how to save a .png file
                encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap)); // create frame from the writable bitmap and add to encoder

                string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);
                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
                string path = Path.Combine(myPhotos, "KinectScreenshot-Color-" + time + ".png");

                try
                { // write the new file to disk
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    this.StatusText = string.Format(Properties.Resources.SavedScreenshotStatusTextFormat, path);
                }
                catch (IOException)
                {
                    this.StatusText = string.Format(Properties.Resources.FailedScreenshotStatusTextFormat, path);
                }
            }
        }

        private void Calibrate_Click(object sender, RoutedEventArgs e)
        {
            if (state == State.DETECT_CONTROLS)
            {
                state = State.MAIN;

                //shapes are now stored in this.controls
                int numSquares = 0, numCircles = 0, numSliders = 0;
                foreach (Shape shape in this.controls)
                {
                    if (shape.type == Shape.ShapeType.SQUARE) numSquares++;
                    if (shape.type == Shape.ShapeType.CIRCLE) numCircles++;
                    if (shape.type == Shape.ShapeType.SLIDER) numSliders++;
                }
                MessageBox.Show("Shapes calibrated. Squares: " + numSquares + ", Circles: " + numCircles + ", Sliders: " + numSliders);
                
            }
        }

        private void loadFakeKinect()
        {
            try
            {
                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
                string path = Path.Combine(myPhotos, "KinectScreenshot-Color-test.png");
                Bitmap bitmap = (Bitmap)Image.FromFile(path, true);
                detectShapeCandidates(ref bitmap, false);
                writeToBackBuffer(ConvertBitmap(bitmap));
                bitmap.Dispose();
            }
            catch (System.IO.FileNotFoundException)
            {
                MessageBox.Show("There was an error opening the bitmap." +
                    "Please check the path.");
            }
        }
    }
}
