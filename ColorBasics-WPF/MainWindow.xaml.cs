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
    using System.Runtime.InteropServices;
    using System.Drawing;
    using Microsoft.Kinect;
    using OpenCvSharp;
    using OpenCvSharp.CPlusPlus;
    using OpenCvSharp.Utilities;
    using OpenCvSharp.Extensions;

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

        //BodyMask Frames
        private DepthSpacePoint[] colorMappedToDepthPoints = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        private string fpsText = null;

        private ColorImageFormat colorImageFormat = ColorImageFormat.Bgra;
        private FrameDescription colorFrameDescription;
        private FrameDescription depthFrameDescription;
        private MultiSourceFrameReader multiSourceFrameReader;

        //Depth Frame
        private ushort[] depthFrameData = null;
        private byte[] depthPixels = null;
        private const int MapDepthToByte = 8000 / 256;

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
            this.kinectSensor = KinectSensor.GetDefault(); // get the kinectSensor object

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(this.colorImageFormat);
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            this.depthFrameData = new ushort[depthFrameDescription.Width * depthFrameDescription.Height];
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.colorBitmap = new WriteableBitmap(this.colorFrameDescription.Width, this.colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null); // create the bitmap to display  
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
           
            this.colorMappedToDepthPoints = new DepthSpacePoint[this.colorFrameDescription.Width * this.colorFrameDescription.Height];


            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
            this.multiSourceFrameReader.MultiSourceFrameArrived += MultiSourceFrameReader_MultiSourceFrameArrived;

            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged; // set IsAvailableChanged event notifier
            this.kinectSensor.Open(); // open the sensor

            this.fpsText = "FPS = 0";
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText : Properties.Resources.NoSensorStatusText; // set the status text
            this.DataContext = this; // use the window object as the view model in this simple example
            this.InitializeComponent(); // initialize the components (controls) of the window

            if (!this.kinectSensor.IsAvailable)
            {
                try
                {
                    string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
                    string path = Path.Combine(myPhotos, "KinectScreenshot-Color-test.png");
                    Bitmap bitmap = (Bitmap)Image.FromFile(path, true);
                    OpenCV(ref bitmap);
                    writeToBackBuffer(ConvertBitmap(bitmap), this.colorBitmap);
                    bitmap.Dispose();
                }
                catch (Exception e)
                {
                    //MessageBox.Show("There was an error opening the test bitmap." +"Please check the path.");
                }
            }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private unsafe void MultiSourceFrameReader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame reference = e.FrameReference.AcquireFrame();
            calculateFps();

            if (reference == null)
            {
                Console.Out.WriteLine("MultiSourceFrameReader is null");
                return;
            }

            using (ColorFrame colorFrame = reference.ColorFrameReference.AcquireFrame())
            using (DepthFrame depthFrame = reference.DepthFrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    this.multiSourceFrameReader.IsPaused = true;
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        ushort maxDepth = ushort.MaxValue;
                        this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                       
                    }
                    RenderDepthPixels();

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
						depthFrame.CopyFrameDataToArray(depthFrameData);
                        this.coordinateMapper.MapColorFrameToDepthSpace(depthFrameData, this.colorMappedToDepthPoints);

                        BitmapSource colorSource = getColorImage(colorFrameDescription, colorFrame);
                        Bitmap colorBitmap = getBitmap(colorSource);
                        
                        fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = this.colorMappedToDepthPoints)
                        {
                            int depthWidth = depthFrame.FrameDescription.Width;
                            int depthHeight = depthFrame.FrameDescription.Height;

                            // Loop over each row and column of the color imageZero out any pixels that don't correspond to a body index
                            for (int colorIndex = 0; colorIndex < this.colorMappedToDepthPoints.Length; ++colorIndex)
                            {
                                float colorMappedToDepthX = colorMappedToDepthPointsPointer[colorIndex].X;
                                float colorMappedToDepthY = colorMappedToDepthPointsPointer[colorIndex].Y;

                                //Console.Out.WriteLine("TESTX: " + colorMappedToDepthX.ToString());

                                // The sentinel value is -inf, -inf, meaning that no depth pixel corresponds tothis color pixel.
                                if (!float.IsNegativeInfinity(colorMappedToDepthX) && !float.IsNegativeInfinity(colorMappedToDepthY))
                                {
                                    // Make sure the depth pixel maps to a valid point in color space
                                    int depthX = (int)(colorMappedToDepthX + 0.5f);
                                    int depthY = (int)(colorMappedToDepthY + 0.5f);

                                    byte test = (byte) (depthX & depthY);

                                    // If the point is not valid, there is no body index there.
                                    if ((depthX >= 0) && (depthX < depthWidth) && (depthY >= 0) && (depthY < depthHeight))
                                    {
                                        int depthIndex = (depthY * depthWidth) + depthX;
                                        //Console.Out.WriteLine("DEPTHINDEX: " + depthIndex);
                                        colorBitmap.SetPixel(colorIndex % 1920, colorIndex / 1920, System.Drawing.Color.FromArgb(test));
                                        // If we are tracking a body for the current pixel,do not zero out the pixel
                                        
                                        continue;
                                    }
                                }
                                // this pixel does not correspond to a body so make it black and transparent
                                //bitmapPixelsPointer[colorIndex] = 0;
                                int testX = colorIndex % 1920;
                                int testY = colorIndex / 1920;
                                //colorBitmap.SetPixel(testX, testY, System.Drawing.Color.Yellow);
                            }
                        } 
                        
                        OpenCV(ref colorBitmap);

                        writeToBackBuffer(ConvertBitmap(colorBitmap), this.colorBitmap);

                        colorBitmap.Dispose();
                    }
                }
                this.multiSourceFrameReader.IsPaused = false;
            }
        }

        private void RenderDepthPixels()
        {
            this.depthBitmap.Lock();
            this.depthBitmap.WritePixels(new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),this.depthPixels,this.depthBitmap.PixelWidth,0);
            this.depthBitmap.Unlock();
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

        private unsafe void OpenCV(ref Bitmap bitmap)
        {
            Mat testMat = BitmapConverter.ToMat(bitmap);
            MatOfDouble mu = new MatOfDouble();
            MatOfDouble sigma = new MatOfDouble();
            Cv2.MeanStdDev(testMat, mu, sigma);
            double mean = mu.GetArray(0, 0)[0];
            mu.Dispose();
            sigma.Dispose();

            checkRectangle(ref testMat);

            Cv2.CvtColor(testMat, testMat, ColorConversion.BgraToGray, 0);
            testMat = testMat.GaussianBlur(new OpenCvSharp.CPlusPlus.Size(1, 1), 5, 5, BorderType.Default);
            testMat = testMat.Canny(0.5 * mean, 1.2 * mean, 3, true);

            bitmap = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(testMat);
            testMat.Dispose();
        }

        private void checkRectangle(ref Mat testMat)
        {
            int xCoord = 960;
            int yCoord = 820;
            int xSize = 100;
            int ySize = 100;
            RotatedRect rRect = new RotatedRect(new Point2f(xCoord, yCoord), new Size2f(xSize, ySize), 0);
            Point2f[] vertices = rRect.Points();
            for (int i = 0; i < 4; i++)
            {
                Cv2.Line(testMat, vertices[i], vertices[(i + 1) % 4], new Scalar(255, 255, 255));
            }
            Vec3b color;
            double blue = 0;
            double green = 0;
            double red = 0;
            //Console.Out.WriteLine("R: " +color.Item2.ToString() + " G: " + color.Item1.ToString() + " B: " + color.Item0.ToString());
            for (int i = 0; i < xSize; i++)
            {
                for (int j = 0; j < ySize; j++)
                {
                    color = testMat.At<Vec3b>(xCoord + i - xSize / 2, yCoord + j - ySize / 2);
                    testMat.Set<Vec3b>(yCoord + j - ySize / 2, xCoord + i - xSize / 2, new Vec3b(255, 255, 255));
                    blue += Double.Parse(color.Item0.ToString()) / (xSize * ySize);
                    green += Double.Parse(color.Item1.ToString()) / (xSize * ySize);
                    red += Double.Parse(color.Item2.ToString()) / (xSize * ySize);
                }
            }
            //Console.Out.WriteLine("R: " + red.ToString().Substring(0, 4) + " G: " + green.ToString().Substring(0, 4) + " B: " + blue.ToString().Substring(0, 4));
            if (red > 180 && green < 80 && blue < 80)
            {
                Console.Out.WriteLine("RED");
            }
            if (red < 90 && green > 180 && blue < 90)
            {
                Console.Out.WriteLine("GREEN");
            }
            if (red < 90 && green < 90 && blue > 180)
            {
                Console.Out.WriteLine("BLUE");
            }
        }

        private void calculateFps()
        {
            ticksNow = DateTime.Now.Ticks;
            fps = ((float)TICKS_PER_SECOND) / (DateTime.Now.Ticks - prevTick); // 1 / ((ticksNow - prevTick) / TICKS_PER_SECOND);
            //Console.Out.WriteLine("fps: " + (int)(fps + 0.5));
            prevTick = ticksNow;

            //calc mean
            sumFps += fps;
            counter++;
            this.FpsText = "FPS = " + fps.ToString();
        }

        private BitmapSource ConvertBitmap(Bitmap source)
        {
            return System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(source.GetHbitmap(), IntPtr.Zero,Int32Rect.Empty,BitmapSizeOptions.FromEmptyOptions());
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText : Properties.Resources.SensorNotAvailableStatusText;
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ColorImageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }

        public ImageSource DepthImageSource
        {
            get
            {
                return this.depthBitmap;
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
    }
}
