//------------------------------------------------------------------------------
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

        private CoordinateMapper coordinateMapper = null;
        private ColorSpacePoint[] depthMappedToColorPoints = null;
        private WriteableBitmap colorBitmap = null;
        private FrameDescription colorFrameDescription;
        private FrameDescription depthFrameDescription;
        private MultiSourceFrameReader multiSourceFrameReader;

        private byte[] colorFrameData;

        //Depth Frame
        private byte[] depthPixels = null;
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// The size in bytes of the bitmap back buffer
        /// </summary>
        private uint bitmapBackBufferSize = 0;

        private KeyPoint[] keyCirclePoints = null;

        
        //private Rectangle baseRect;
        //private Rectangle featureRect;
        private int featureSize;
        private double featureDepth;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        private string fpsText = null;

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

            public Double depth = -1;

            public Shape(ShapeType type, OpenCvSharp.CPlusPlus.Rect bounds)
            {
                this.type = type;
                this.bounds = bounds;
            }
        }

        //check performance in ticks
        private const long TICKS_PER_SECOND = 10000000; //according to msdn
        private double fps = 0;
        private long prevTick = 0, ticksNow = 0;
        private double sumFps = 0;
        private int counter = 0;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            //load SVM
            this.svm = loadSVM();

            this.kinectSensor = KinectSensor.GetDefault(); // get the kinectSensor object

            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.BodyIndex);
            this.multiSourceFrameReader.MultiSourceFrameArrived += MultiSourceFrameReader_MultiSourceFrameArrived;

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;

            this.depthMappedToColorPoints = new ColorSpacePoint[this.depthFrameDescription.LengthInPixels];

            // Could copy only the pixels we need with this.colorBitmap.Pixels(...)
            this.colorFrameData = new byte[this.colorFrameDescription.Width * this.colorFrameDescription.Height * 4];

            // ??????
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            this.colorBitmap = new WriteableBitmap(this.colorFrameDescription.Width, this.colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null); // create the bitmap to display  
            
            int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
            // Calculate the WriteableBitmap back buffer size
            this.bitmapBackBufferSize = (uint)((this.colorBitmap.BackBufferStride * (this.colorBitmap.PixelHeight - 1)) + (this.colorBitmap.PixelWidth * bytesPerPixel));

            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged; // set IsAvailableChanged event notifier
            this.kinectSensor.Open(); // open the sensor

            this.fpsText = "FPS = 0";
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
        private unsafe void MultiSourceFrameReader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            // We use a try/finally to ensure that we clean up before we exit the function.  
            // This includes calling Dispose on any Frame objects that we may have and unlocking the bitmap back buffer.
            try
            {
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();

                // If any frame has expired by the time we process this event, return. The "finally" statement will Dispose any that are not null.
                if ((depthFrame == null) || (colorFrame == null))
                {
                    return;
                }
                this.multiSourceFrameReader.IsPaused = true;

                // Process Depth
                FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                // Access the depth frame data directly via LockImageBuffer to avoid making a copy
                using (KinectBuffer depthFrameData = depthFrame.LockImageBuffer())
                {
                    this.coordinateMapper.MapDepthFrameToColorSpaceUsingIntPtr(depthFrameData.UnderlyingBuffer, depthFrameData.Size, this.depthMappedToColorPoints);
                    ushort maxDepth = ushort.MaxValue;
                    this.ProcessDepthFrameData(depthFrameData.UnderlyingBuffer, depthFrameData.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                }

                // We're done with the DepthFrame 
                depthFrame.Dispose();
                depthFrame = null;

                BitmapSource colorSource = getColorImage(colorFrameDescription, colorFrame);
                Bitmap colorBitmap = getBitmap(colorSource);

                switch (state)
                {
                    case State.DETECT_CONTROLS:
                        detectShapeCandidates(ref colorBitmap, false);
                        break;
                    case State.MAIN:
                        OpenCV(ref colorBitmap);
                        break;
                }

                writeToBackBuffer(ConvertBitmap(colorBitmap), this.colorBitmap);
                colorBitmap.Dispose();
                colorFrame.Dispose();
                calculateFps();
            }
            finally
            {
                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                }

                if (colorFrame != null)
                {
                    colorFrame.Dispose();
                }
                this.multiSourceFrameReader.IsPaused = false;
            }
        }

        private unsafe void OpenCV(ref Bitmap bitmap)
        {
            Mat testMat = BitmapConverter.ToMat(bitmap);
            MatOfDouble mu = new MatOfDouble();
            MatOfDouble sigma = new MatOfDouble();
            Cv2.MeanStdDev(testMat, mu, sigma);
            double mean = mu.GetArray(0, 0)[0];
            mu.Dispose();
            sigma.Dispose();

            SimpleBlobDetector.Params circleParameters = new SimpleBlobDetector.Params();
            circleParameters.FilterByCircularity = true;
            circleParameters.MinCircularity = (float)0.85;
            circleParameters.MaxCircularity = (float)1;
            circleParameters.MinArea = 30; // Modify the value on the fly (TODO use bigger circle)
            //circleParameters.FilterByArea = true;
            //circleParameters.MaxArea = 500;

            SimpleBlobDetector detectCircleBlobs = new SimpleBlobDetector(circleParameters);
            keyCirclePoints = detectCircleBlobs.Detect(testMat);
            detectCircleBlobs.Dispose();

            // If Finger found basically
            if (keyCirclePoints != null)
            {
                this.featureSize = 0;
                int fingerIndex = -1;
                for (int i = 0; i < keyCirclePoints.Length; i++)
                {
                    if (keyCirclePoints[i].Size >= this.featureSize)
                    {
                        this.featureSize = (int)keyCirclePoints[i].Size;
                        fingerIndex = i;
                    }
                }

                if (fingerIndex != -1)
                {
                    OpenCvSharp.CPlusPlus.Point coordinate = keyCirclePoints[fingerIndex].Pt;
                    this.featureSize = (int)((keyCirclePoints[fingerIndex].Size) * Math.Sqrt(2));
                    testMat.Set<Vec3b>(coordinate.Y, coordinate.X, new Vec3b(0, 255, 0));
                    RotatedRect rRect = new RotatedRect(new Point2f(coordinate.X, coordinate.Y), new Size2f(this.featureSize, this.featureSize), 0);
                    Point2f[] circleVerticies = rRect.Points();
                    int height = (int)(circleVerticies[0].Y - circleVerticies[1].Y);
                    int width = (int)(circleVerticies[2].X - circleVerticies[1].X);
                    int startX = (int)(circleVerticies[0].X);
                    int startY = (int)(circleVerticies[1].Y);
                    MapColortoDepth(startX, startY, this.featureSize, this.featureSize, "feature");
                    OpenCvSharp.CPlusPlus.Rect featureRect = new OpenCvSharp.CPlusPlus.Rect(startX, startY, this.featureSize, this.featureSize);

                    for (int j = 0; j < 4; j++)
                    {
                        Cv2.Line(testMat, circleVerticies[j], circleVerticies[(j + 1) % 4], new Scalar(0, 255, 0));
                    }

                    for (int i = 0; i < this.controls.Count; i++)
                    {
                        if(this.controls[i].bounds.IntersectsWith(featureRect))
                        {
                            MapColortoDepth(this.controls[i].bounds.TopLeft.X, this.controls[i].bounds.TopLeft.Y, this.controls[i].bounds.Width, this.controls[i].bounds.Height, "base", i);
                            double diff = featureDepth - this.controls[i].depth;
                            if (Math.Abs(diff) < 0.5)
                            {
                                Console.Out.WriteLine("Intersection with " + i.ToString() + " Difference: " + diff.ToString() + " CONTACT");
                            }
                        }
                    }
                }
            }

            bitmap = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(testMat);
            testMat.Dispose();
        }

        //TODO: Request certain pixels
        private void MapColortoDepth(int startX, int startY, int widthX, int heightY, string opt = null, int shapeIndex = -1)
        {
            int colorWidth = 1920;
            int colorHeight = 1080;
            double depthCount = 0;

            for (int colorIndex = 0; colorIndex < this.depthMappedToColorPoints.Length - 4; colorIndex++)
            {
                ushort depth = this.depthPixels[colorIndex];
                ColorSpacePoint point = this.depthMappedToColorPoints[colorIndex];

                // round down to the nearest pixel
                int colorX = (int)Math.Floor(point.X + 0.5);
                int colorY = (int)Math.Floor(point.Y + 0.5);

                // make sure the pixel is part of the image
                if ((colorX >= 0 && (colorX < colorWidth) && (colorY >= 0) && (colorY < colorHeight)))
                {
                    int colorImageIndex = ((colorWidth * colorY) + colorX) * 4;

                    // Check if pixels are within the range of the bounding box
                    if (colorX >= startX && colorX <= startX + widthX && colorY >= startY && colorY <= startY + heightY)
                    {
                        depthCount += Convert.ToDouble(depth) / (widthX * heightY);
                        this.colorFrameData[colorImageIndex] = (byte)depth;
                        this.colorFrameData[colorImageIndex + 1] = (byte)depth;
                        this.colorFrameData[colorImageIndex + 2] = (byte)depth;
                        //colorImageIndex++; //Skip Alpha for BGR32 
                    }
                }
            }
            if (depthCount != 0) //TODO why not make this a return type?
            {
                if (opt == "base" && shapeIndex != -1)
                {
                    this.controls[shapeIndex].depth = depthCount;
                }
                if (opt == "feature")
                {
                    this.featureDepth = depthCount;
                }
            }

        }

        private void detectShapeCandidates(ref Bitmap bitmap, Boolean saveShapes)
        {
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

        private unsafe void writeToBackBuffer(BitmapSource source, WriteableBitmap bitmap)
        {
            bitmap.Lock();
            int stride = source.PixelWidth * (source.Format.BitsPerPixel / 8); // Calculate stride of source
            byte[] data = new byte[stride * source.PixelHeight]; // Create data array to hold source pixel data
            source.CopyPixels(data, stride, 0); // Copy source image pixels to the data array

            // Write the pixel data to the WriteableBitmap.
            bitmap.WritePixels(new Int32Rect(0, 0, source.PixelWidth, source.PixelHeight), data, stride, 0);
            bitmap.AddDirtyRect(new Int32Rect(0, 0, source.PixelWidth, source.PixelHeight));
            bitmap.Unlock();
        }

        private unsafe BitmapSource getColorImage(FrameDescription colorFrameDescription, ColorFrame colorFrame)
        {
            byte[] pixels = new byte[colorFrameDescription.Height * colorFrameDescription.Width * 4];
            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
            int stride = colorFrameDescription.Width * 4;
            return BitmapSource.Create(colorFrameDescription.Width, colorFrameDescription.Height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
        }

        private unsafe Bitmap getBitmap(BitmapSource source)
        {
            Bitmap bitmap;
            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder enc = new BmpBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create(source));
                enc.Save(outStream);
                bitmap = new Bitmap(outStream);
            }

            return bitmap;
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];
                // To convert to a byte, we're mapping the depth value to the byte range. Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        public unsafe BitmapSource getDepthImage(DepthFrame depthFrame)
        {
            int width = depthFrame.FrameDescription.Width;
            int height = depthFrame.FrameDescription.Height;

            ushort minDepth = depthFrame.DepthMinReliableDistance;
            ushort maxDepth = depthFrame.DepthMaxReliableDistance;

            ushort[] depthData = new ushort[width * height];
            byte[] pixelData = new byte[width * height * (PixelFormats.Bgr32.BitsPerPixel + 7) / 8];

            int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;

            depthFrame.CopyFrameDataToArray(depthData);

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

            return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixelData, stride);
        }

        public static BitmapSource ConvertBitmap(Bitmap source)
        {
            return System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(source.GetHbitmap(), IntPtr.Zero,
                          Int32Rect.Empty,
                          BitmapSizeOptions.FromEmptyOptions());
        }

        private void calculateFps()
        {
            ticksNow = DateTime.Now.Ticks;
            fps = ((float)TICKS_PER_SECOND) / (DateTime.Now.Ticks - prevTick); // 1 / ((ticksNow - prevTick) / TICKS_PER_SECOND);
            prevTick = ticksNow;

            //Calculate Mean
            sumFps += fps;
            counter++;
            this.FpsText = "FPS = " + fps.ToString();
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
        /// Gets or sets the current status text to display
        /// </summary>
        public string FpsText
        {
            get
            {
                return this.fpsText;
            }

            set
            {
                if (this.fpsText != value)
                {
                    this.fpsText = value;
                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("FpsText"));
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
            if (this.multiSourceFrameReader != null)
            {
                this.multiSourceFrameReader.Dispose();
                this.multiSourceFrameReader = null;
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
                writeToBackBuffer(ConvertBitmap(bitmap), this.colorBitmap);
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
