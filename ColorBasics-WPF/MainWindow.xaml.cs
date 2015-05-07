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

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
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
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            byte[] pixels = new byte[colorFrameDescription.Height * colorFrameDescription.Width * 4];
                            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
                            int stride = colorFrameDescription.Width * 4;
                            BitmapSource source = BitmapSource.Create(colorFrameDescription.Width, colorFrameDescription.Height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);

                            byte[] data;
                            Bitmap bitmap;
                            using (MemoryStream outStream = new MemoryStream())
                            {
                                BitmapEncoder enc = new BmpBitmapEncoder();
                                enc.Frames.Add(BitmapFrame.Create(source));
                                enc.Save(outStream);
                                bitmap = new Bitmap(outStream);
                            }

                            Mat testMat = BitmapConverter.ToMat(bitmap);
                            MatOfDouble mu = new MatOfDouble();
                            MatOfDouble sigma = new MatOfDouble();
                            Cv2.MeanStdDev(testMat, mu, sigma);
                            double mean = mu.GetArray(0, 0)[0];
                            mu.Dispose();
                            sigma.Dispose();

                            Cv2.CvtColor(testMat, testMat, ColorConversion.BgraToGray, 0);
                            testMat = testMat.GaussianBlur(new OpenCvSharp.CPlusPlus.Size(1, 1), 5, 5, BorderType.Default);
                            testMat = testMat.Canny(0.5 * mean, 1.2 * mean, 3, true);
                            int width = source.PixelWidth;
                            int height = source.PixelHeight;
                            bitmap = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(testMat);
                            testMat.Dispose();
                            source = ConvertBitmap(bitmap);
                            bitmap.Dispose();
                            stride = width * (source.Format.BitsPerPixel / 8); // Calculate stride of source
                            data = new byte[stride * height]; // Create data array to hold source pixel data
                            source.CopyPixels(data, stride, 0); // Copy source image pixels to the data array

                            // Write the pixel data to the WriteableBitmap.
                            this.colorBitmap.WritePixels(new Int32Rect(0, 0, width, height), data, stride, 0);
                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, width, height));
                        }

                        this.colorBitmap.Unlock();
                        this.colorFrameReader.IsPaused = false;
                    }
                }
            }
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
    }
}
