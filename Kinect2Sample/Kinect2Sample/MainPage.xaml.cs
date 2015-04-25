﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.ComponentModel;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.UI.Xaml.Media.Imaging;
using WindowsPreview.Kinect;

namespace Kinect2Sample {

    /// An empty page that can be used on its own or navigated to within a Frame.
    /// To enable the xaml to bind to the status values, if they change, you must implement the INotifyPropertyChanged interface
    public sealed partial class MainPage : Page, INotifyPropertyChanged {
        private KinectSensor kinectSensor = null;
        private string statusText = null;
        private WriteableBitmap bitmap = null; // The WritableBitmap which will replace the contents of an image in xaml

        public event PropertyChangedEventHandler PropertyChanged;

        private const DisplayFrameType DEFAULT_DISPLAYFRAMETYPE = DisplayFrameType.Infrared;
        private FrameDescription currentFrameDescription;
        private DisplayFrameType currentDisplayFrameType;

        //Infrared Frame
        private ushort[] infraredFrameData = null; // Intermediate storage for receiving frame data from the sensor
        private byte[] infraredPixels = null; // Intermediate storage for frame data converted to color pixels for display

        //Depth Frame
        private ushort[] depthFrameData = null;
        private byte[] depthPixels = null;

        private const int BytesPerPixel = 4; // Size of the RGB pixel in the bitmap


        private MultiSourceFrameReader multiSourceFrameReader = null;

        /// <summary>
        /// The highest value that can be returned in the InfraredFrame.
        /// It is cast to a float for readability in the visualization code.
        /// </summary>
        private const float InfraredSourceValueMaximum =
            (float)ushort.MaxValue;

        /// </summary>
        /// Used to set the lower limit, post processing, of the
        /// infrared data that we will render.
        /// Increasing or decreasing this value sets a brightness
        /// "wall" either closer or further away.
        /// </summary>
        private const float InfraredOutputValueMinimum = 0.01f;

        /// <summary>
        /// The upper limit, post processing, of the
        /// infrared data that will render.
        /// </summary>
        private const float InfraredOutputValueMaximum = 1.0f;

        /// <summary>
        /// The InfraredSceneValueAverage value specifies the 
        /// average infrared value of the scene. 
        /// This value was selected by analyzing the average
        /// pixel intensity for a given scene.
        /// This could be calculated at runtime to handle different
        /// IR conditions of a scene (outside vs inside).
        /// </summary>
        private const float InfraredSceneValueAverage = 0.08f;

        /// <summary>
        /// The InfraredSceneStandardDeviations value specifies 
        /// the number of standard deviations to apply to
        /// InfraredSceneValueAverage.
        /// This value was selected by analyzing data from a given scene.
        /// This could be calculated at runtime to handle different
        /// IR conditions of a scene (outside vs inside).
        /// </summary>
        private const float InfraredSceneStandardDeviations = 3.0f;

        public enum DisplayFrameType {
            Infrared,
            Color,
            Depth
        }

        public MainPage() {
            this.kinectSensor = KinectSensor.GetDefault(); // one sensor is currently supported

            // currently selected state to initialize the bitmap with the right size
            SetupCurrentDisplay(DEFAULT_DISPLAYFRAMETYPE);

            //The MultiSourceFrameReader is initialized with the FrameSourceTypes which will be used,
            // so it is important to remember that to add new frame types
            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Infrared | FrameSourceTypes.Color | FrameSourceTypes.Depth);
            // set MultiSourceFrameArrived event notifier
            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged +=  this.Sensor_IsAvailableChanged;

            // use the window object as the view model in this example
            this.DataContext = this;

            this.kinectSensor.Open(); // open the sensor
            this.InitializeComponent();
        }

        // code will fire the property changed event when the status changes, and when the frame description changes. 
        private void Sensor_IsAvailableChanged(KinectSensor sender, IsAvailableChangedEventArgs args) {
            this.StatusText = this.kinectSensor.IsAvailable ? "Running" : "Not Available";
        }

        private void Reader_MultiSourceFrameArrived( MultiSourceFrameReader sender, MultiSourceFrameArrivedEventArgs e) {
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null) {
                return;
            }

            switch (currentDisplayFrameType) {
                case DisplayFrameType.Infrared:
                    using (InfraredFrame infraredFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame()) {
                        ShowInfraredFrame(infraredFrame);
                    }
                    break;
                case DisplayFrameType.Color:
                    using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame()) {
                        ShowColorFrame(colorFrame);
                    }
                    break;
                case DisplayFrameType.Depth:
                    using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame()) {
                        ShowDepthFrame(depthFrame);
                    }
                    break;
                default:
                    break;
            }
        }

        private void ShowInfraredFrame(InfraredFrame infraredFrame) {
            bool infraredFrameProcessed = false;

            if (infraredFrame != null) {
                FrameDescription infraredFrameDescription = infraredFrame.FrameDescription;

                // verify data and write the new infrared frame data to the display bitmap
                if (((infraredFrameDescription.Width * infraredFrameDescription.Height)  == this.infraredFrameData.Length) &&
                    (infraredFrameDescription.Width == this.bitmap.PixelWidth) && (infraredFrameDescription.Height == this.bitmap.PixelHeight))  {
                    // Copy the pixel data from the image to a temporary array
                    infraredFrame.CopyFrameDataToArray(this.infraredFrameData);
                    infraredFrameProcessed = true;
                }
            }

            // we got a frame, convert and render
            if (infraredFrameProcessed) {
                this.ConvertInfraredDataToPixels();
                this.RenderPixelArray(this.infraredPixels);
            }
        }

        private void ShowColorFrame(ColorFrame colorFrame) {
            bool colorFrameProcessed = false;

            if (colorFrame != null) {
                FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                // verify data and write the new color frame data to the Writeable bitmap
                if ((colorFrameDescription.Width == this.bitmap.PixelWidth) && (colorFrameDescription.Height == this.bitmap.PixelHeight)) {
                    if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra) {
                        colorFrame.CopyRawFrameDataToBuffer(this.bitmap.PixelBuffer);
                    } else {
                        colorFrame.CopyConvertedFrameDataToBuffer(this.bitmap.PixelBuffer, ColorImageFormat.Bgra);
                    }
                    colorFrameProcessed = true;
                }
            }

            if (colorFrameProcessed)  {
                this.bitmap.Invalidate();
                FrameDisplayImage.Source = this.bitmap;
            }
        }

        // notice how the entire image appears jittery and fuzzy around edges. This is because it’s not representing a visual feed, 
        // and instead representing calculations of time which has a margin of error associated when computing at such high speeds.
        private void ShowDepthFrame(DepthFrame depthFrame) {
            bool depthFrameProcessed = false;
            ushort minDepth = 0;
            ushort maxDepth = 0;

            if (depthFrame != null) {
                FrameDescription depthFrameDescription = depthFrame.FrameDescription;

                // verify data and write the new infrared frame data to the display bitmap
                if (((depthFrameDescription.Width * depthFrameDescription.Height) == this.infraredFrameData.Length) &&
                    (depthFrameDescription.Width == this.bitmap.PixelWidth) && (depthFrameDescription.Height == this.bitmap.PixelHeight)) {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyFrameDataToArray(this.depthFrameData);

                    // max = 8000; big distances are unreliable for the Kinect to calculate as real points, 
                    // the fuzziness of the image will increase at large distances.
                    minDepth = depthFrame.DepthMinReliableDistance;
                    maxDepth = depthFrame.DepthMaxReliableDistance;

                    depthFrameProcessed = true;
                }
            }

            // we got a frame, convert and render
            if (depthFrameProcessed) {
                ConvertDepthDataToPixels(minDepth, maxDepth);
                RenderPixelArray(this.depthPixels);
            }
        }


        // Convert the infrared to RGB
        private void ConvertInfraredDataToPixels() {
            int colorPixelIndex = 0;
            for (int i = 0; i < this.infraredFrameData.Length; ++i) {
                // normalize the incoming infrared data (ushort) to a float between InfraredOutputValueMinimum to InfraredOutputValueMaximum] by
                // 1. dividing the incoming value by the source maximum value
                float intensityRatio = (float)this.infraredFrameData[i] / InfraredSourceValueMaximum;
                // 2. dividing by the (average scene value * standard deviations)
                intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;
                // 3. limiting the value to InfraredOutputValueMaximum
                intensityRatio = Math.Min(InfraredOutputValueMaximum, intensityRatio);
                // 4. limiting the lower value InfraredOutputValueMinimum
                intensityRatio = Math.Max(InfraredOutputValueMinimum, intensityRatio);

                // 5. converting the normalized value to a byte and using the result as the RGB components required by the image
                byte intensity = (byte)(intensityRatio * 255.0f);
                this.infraredPixels[colorPixelIndex++] = intensity; //Blue
                this.infraredPixels[colorPixelIndex++] = intensity; //Green
                this.infraredPixels[colorPixelIndex++] = intensity; //Red
                this.infraredPixels[colorPixelIndex++] = 255;       //Alpha           
            }
        }

        // iterates through each point in the depth frame and finds a matching color for it based on the intensity
        private void ConvertDepthDataToPixels(ushort minDepth, ushort maxDepth) {
            int colorPixelIndex = 0;
            int mapDepthToByte = maxDepth / 256; // Shape the depth to the range of a byte

            for (int i = 0; i < this.depthFrameData.Length; ++i) {
                ushort depth = this.depthFrameData[i]; // Get the depth for this pixel

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                byte intensity = (byte)(depth >= minDepth &&  depth <= maxDepth ? (depth / mapDepthToByte) : 0);

                this.depthPixels[colorPixelIndex++] = intensity; //Blue
                this.depthPixels[colorPixelIndex++] = intensity; //Green
                this.depthPixels[colorPixelIndex++] = intensity; //Red
                this.depthPixels[colorPixelIndex++] = 255; //Alpha
            }
        }

        // ConvertInfraredDataToPixels() before this...
        private void RenderPixelArray(byte[] pixels) {
            pixels.CopyTo(this.bitmap.PixelBuffer);
            this.bitmap.Invalidate();
            FrameDisplayImage.Source = this.bitmap;
            //FrameDisplayImage1.Source = this.bitmap;
        }

        private void SetupCurrentDisplay(DisplayFrameType newDisplayFrameType) {
            currentDisplayFrameType = newDisplayFrameType;
            switch (currentDisplayFrameType) {
                case DisplayFrameType.Infrared:
                    // get the infraredFrameDescription from the InfraredFrameSource
                    FrameDescription infraredFrameDescription = this.kinectSensor.InfraredFrameSource.FrameDescription;
                    this.CurrentFrameDescription = infraredFrameDescription;
                    // allocate space to put the pixels being  received and converted            
                    this.infraredFrameData = new ushort[infraredFrameDescription.Width * infraredFrameDescription.Height];
                    this.infraredPixels = new byte[infraredFrameDescription.Width * infraredFrameDescription.Height * BytesPerPixel];
                    // create the bitmap to display
                    this.bitmap = new WriteableBitmap(infraredFrameDescription.Width, infraredFrameDescription.Height);
                    break;
                case DisplayFrameType.Color:
                    FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
                    this.CurrentFrameDescription = colorFrameDescription;
                    // create the bitmap to display
                    this.bitmap =  new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height);
                    break;
                case DisplayFrameType.Depth:
                    // Could be the same as infrared, but good practice to keep separate for more advanced cases
                    FrameDescription depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
                    this.CurrentFrameDescription = depthFrameDescription;
                    // allocate space to put the pixels being received and converted
                    this.depthFrameData =  new ushort[depthFrameDescription.Width * depthFrameDescription.Height];
                    this.depthPixels =  new byte[depthFrameDescription.Width * depthFrameDescription.Height * BytesPerPixel];
                    this.bitmap = new WriteableBitmap(depthFrameDescription.Width, depthFrameDescription.Height);
                    break;
                default:
                    break;
            }
        }

        // code will fire the property changed event when the status changes, and when the frame description changes. 
        // Whenever the StatusText or CurrentFrameDescription is set, the PropertyChanged event will fire and notify the xaml that it has changed, which refreshes the binding
        public string StatusText{
            get { return this.statusText; }
            set {
                if (this.statusText != value) {
                    this.statusText = value;
                    if (this.PropertyChanged != null) {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        //code will fire the property changed event when the status changes, and when the frame description changes. 
        public FrameDescription CurrentFrameDescription {
            get { return this.currentFrameDescription; }
            set {
                if (this.currentFrameDescription != value) {
                    this.currentFrameDescription = value;
                    if (this.PropertyChanged != null) {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("CurrentFrameDescription"));
                    }
                }
            }
        }

        //XAML onClick method
        private void InfraredButton_Click(object sender, RoutedEventArgs e) {
            SetupCurrentDisplay(DisplayFrameType.Infrared);
        }

        //XAML onClick method
        private void ColorButton_Click(object sender, RoutedEventArgs e) {
            SetupCurrentDisplay(DisplayFrameType.Color);
        }

        //XAML onClick method
        private void DepthButton_Click(object sender, RoutedEventArgs e) {
            SetupCurrentDisplay(DisplayFrameType.Depth);
        }

    }
}
