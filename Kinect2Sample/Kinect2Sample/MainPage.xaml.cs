using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.WindowsRuntime;
using System.ComponentModel;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.Storage.Streams;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.UI.Xaml.Media.Imaging;
using WindowsPreview.Kinect;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.UI;

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

        private MultiSourceFrameReader multiSourceFrameReader = null;
        private CoordinateMapper coordinateMapper = null;
        private BodiesManager bodiesManager = null; // responsible for organizing and drawing all the tracked bodies in a frame

        //Infrared Frame
        private ushort[] infraredFrameData = null; // Intermediate storage for receiving frame data from the sensor
        private byte[] infraredPixels = null; // Intermediate storage for frame data converted to color pixels for display

        //Depth Frame
        private ushort[] depthFrameData = null;
        private byte[] depthPixels = null;

        //BodyMask Frames
        private DepthSpacePoint[] colorMappedToDepthPoints = null;

        //Body Joints are drawn here
        private Canvas drawingCanvas;

        private const int BytesPerPixel = 4; // Size of the RGB pixel in the bitmap

        /// The highest value that can be returned in the InfraredFrame. It is cast to a float for readability in the visualization code.
        private const float InfraredSourceValueMaximum = (float)ushort.MaxValue;

        /// Used to set the lower limit, post processing, of theinfrared data that we will render.
        /// Increasing or decreasing this value sets a brightness "wall" either closer or further away.
        private const float InfraredOutputValueMinimum = 0.01f;

        /// The upper limit, post processing, of the infrared data that will render.
        private const float InfraredOutputValueMaximum = 1.0f;

        /// The InfraredSceneValueAverage value specifies the average infrared value of the scene. 
        /// This value was selected by analyzing the average pixel intensity for a given scene.
        /// This could be calculated at runtime to handle different IR conditions of a scene (outside vs inside).
        private const float InfraredSceneValueAverage = 0.08f;

        /// The InfraredSceneStandardDeviations value specifies the number of standard deviations to apply to InfraredSceneValueAverage.
        /// This value was selected by analyzing data from a given scene. This could be calculated at runtime to handle different
        /// IR conditions of a scene (outside vs inside).
        private const float InfraredSceneStandardDeviations = 3.0f;

        // The Body Mask display requires three frame types from the MultiSourceFrame (infrared, color, depth)
        // To use these and dispose of them all at once it’s more appropriate to use a try, finally pattern
        // In the finally section, all the used objects can be disposed of at once.
        public enum DisplayFrameType {
            Infrared,
            Color,
            Depth,
            BodyMask,
            BodyJoints
        }

        public MainPage() {
            this.kinectSensor = KinectSensor.GetDefault(); // one sensor is currently supported

            // currently selected state to initialize the bitmap with the right size
            SetupCurrentDisplay(DEFAULT_DISPLAYFRAMETYPE);

            // If all the feeds from the Kinect were from the same camera with the same type
            // there would be no need to map as they would already be naturally relative to one another
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            //The MultiSourceFrameReader is initialized with the FrameSourceTypes which will be used,
            // so it is important to remember that to add new frame types
            this.multiSourceFrameReader = this.kinectSensor.
                OpenMultiSourceFrameReader(FrameSourceTypes.Infrared | FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);
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

            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;
            InfraredFrame infraredFrame = null;
            BodyIndexFrame bodyIndexFrame = null;
            BodyFrame bodyFrame = null;
            //windows.storage.streams
            IBuffer depthFrameDataBuffer = null;
            IBuffer bodyIndexFrameData = null;
            IBufferByteAccess bodyIndexByteAccess = null; // Com interface for unsafe byte manipulation

            switch (currentDisplayFrameType) {
                case DisplayFrameType.Infrared:
                    using (infraredFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame()) {
                        ShowInfraredFrame(infraredFrame);
                    }
                    break;
                case DisplayFrameType.Color:
                    using (colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame()) {
                        ShowColorFrame(colorFrame);
                    }
                    break;
                case DisplayFrameType.Depth:
                    using (depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame()) {
                        ShowDepthFrame(depthFrame);
                    }
                    break;
                case DisplayFrameType.BodyMask:
                    // Put in a try catch to utilise finally() and clean up frames
                    try {
                        depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                        bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();
                        colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                        if ((depthFrame == null) || (colorFrame == null) || (bodyIndexFrame == null)) {
                            return;
                        }

                        // Access the depth frame data directly via LockImageBuffer to avoid making a copy
                        depthFrameDataBuffer = depthFrame.LockImageBuffer();
                        this.coordinateMapper.MapColorFrameToDepthSpaceUsingIBuffer(depthFrameDataBuffer, this.colorMappedToDepthPoints);
                        // Process Color
                        colorFrame.CopyConvertedFrameDataToBuffer(this.bitmap.PixelBuffer,ColorImageFormat.Bgra);
                        // Access the body index frame data directly via LockImageBuffer to avoid making a copy
                        bodyIndexFrameData = bodyIndexFrame.LockImageBuffer();
                        ShowMappedBodyFrame(depthFrame.FrameDescription.Width, depthFrame.FrameDescription.Height, bodyIndexFrameData, bodyIndexByteAccess);
                    } finally {
                        if (depthFrame != null) {
                            depthFrame.Dispose();
                        }
                        if (colorFrame != null) {
                            colorFrame.Dispose();
                        }
                        if (bodyIndexFrame != null) {
                            bodyIndexFrame.Dispose();
                        }

                        if (depthFrameDataBuffer != null) {
                            // We must force a release of the IBuffer in order toensure that we have dropped all references to it.
                            System.Runtime.InteropServices.Marshal.ReleaseComObject(depthFrameDataBuffer);
                        }
                        if (bodyIndexFrameData != null) {
                            System.Runtime.InteropServices.Marshal.ReleaseComObject(bodyIndexFrameData);
                        }
                        if (bodyIndexByteAccess != null) {
                            System.Runtime.InteropServices.Marshal.ReleaseComObject(bodyIndexByteAccess);
                        }
                    }
                    break;
                case DisplayFrameType.BodyJoints:
                    using (bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame()) {
                        ShowBodyJoints(bodyFrame);
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

                var imgBuffer = new Byte[colorFrameDescription.BytesPerPixel * colorFrameDescription.Width * colorFrameDescription.Height];

                // verify data and write the new color frame data to the Writeable bitmap
                if ((colorFrameDescription.Width == this.bitmap.PixelWidth) && (colorFrameDescription.Height == this.bitmap.PixelHeight)) {
                    if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra) {
                        colorFrame.CopyRawFrameDataToArray(imgBuffer);
                    } else {
                        colorFrame.CopyConvertedFrameDataToArray(imgBuffer, ColorImageFormat.Bgra);
                    }

                    using (var ms = new MemoryStream(imgBuffer)){
                        System.Drawing.Bitmap bmp = new System.Drawing.Bitmap(ms);
                        //canny on bmp

                        Windows.Graphics.Imaging.SoftwareBitmap sb;
                        using (Stream outStream = this.bitmap.PixelBuffer.AsStream()){
                            bmp.Save(outStream, System.Drawing.Imaging.ImageFormat.Png);
                        }
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

        // Using pointers in C# is also known as unsafe code because the memory management is done manually.
        // Unsafe code must be allowed by the project settings before it can build. 
        // Unsafe code generally performs better, but only when used correctly and optimized during the build.
        unsafe private void ShowMappedBodyFrame(int depthWidth, int depthHeight, IBuffer bodyIndexFrameData, IBufferByteAccess bodyIndexByteAccess) {
            bodyIndexByteAccess = (IBufferByteAccess)bodyIndexFrameData;
            byte* bodyIndexBytes = null;
            bodyIndexByteAccess.Buffer(out bodyIndexBytes);

            fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = this.colorMappedToDepthPoints) {
                IBufferByteAccess bitmapBackBufferByteAccess = (IBufferByteAccess)this.bitmap.PixelBuffer;

                byte* bitmapBackBufferBytes = null;
                bitmapBackBufferByteAccess.Buffer(out bitmapBackBufferBytes);

                // Treat the color data as 4-byte pixels
                uint* bitmapPixelsPointer = (uint*)bitmapBackBufferBytes;

                // Loop over each row and column of the color image. Zero out any pixels that don't correspond to a body index
                int colorMappedLength = this.colorMappedToDepthPoints.Length;
                for (int colorIndex = 0; colorIndex < colorMappedLength; ++colorIndex) {
                    float colorMappedToDepthX = colorMappedToDepthPointsPointer[colorIndex].X;
                    float colorMappedToDepthY = colorMappedToDepthPointsPointer[colorIndex].Y;

                    // The sentinel value is -inf, -inf, meaning that no depth pixel corresponds to this color pixel.
                    if (!float.IsNegativeInfinity(colorMappedToDepthX) && !float.IsNegativeInfinity(colorMappedToDepthY)) {
                        // Make sure the depth pixel maps to a valid point in color space
                        int depthX = (int)(colorMappedToDepthX + 0.5f);
                        int depthY = (int)(colorMappedToDepthY + 0.5f);

                        // If the point is not valid, there is no body index there.
                        if ((depthX >= 0) && (depthX < depthWidth) && (depthY >= 0) && (depthY < depthHeight)) {
                            int depthIndex = (depthY * depthWidth) + depthX;

                            // If we are tracking a body for the current pixel,do not zero out the pixel
                            if (bodyIndexBytes[depthIndex] != 0xff) {
                                // this bodyIndexByte is good and is a body, loop again.
                                continue;
                            }
                        }
                    }
                    // this pixel does not correspond to a body so make it black and transparent
                    bitmapPixelsPointer[colorIndex] = 0;
                }
            }

            this.bitmap.Invalidate();
            FrameDisplayImage.Source = this.bitmap;
        }

        private void ShowBodyJoints(BodyFrame bodyFrame) {
            Body[] bodies = new Body[ this.kinectSensor.BodyFrameSource.BodyCount];
            bool dataReceived = false;
            if (bodyFrame != null) {
                bodyFrame.GetAndRefreshBodyData(bodies);
                dataReceived = true;
            }

            if (dataReceived) {
                this.bodiesManager.UpdateBodiesAndEdges(bodies);
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
            // Frames used by more than one type are declared outside the switch
            FrameDescription colorFrameDescription = null;

            // reset the display methods; prevents the xaml elements from rendering on top of one another
            if (this.BodyJointsGrid != null) {
                this.BodyJointsGrid.Visibility = Visibility.Collapsed;
            }
            if (this.FrameDisplayImage != null) {
                this.FrameDisplayImage.Source = null;
            }

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
                    colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
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
                case DisplayFrameType.BodyMask:
                    colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
                    this.CurrentFrameDescription = colorFrameDescription;
                    // allocate space to put the pixels being received and converted
                    this.colorMappedToDepthPoints = new DepthSpacePoint[colorFrameDescription.Width * colorFrameDescription.Height];
                    this.bitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height);
                    break;
                case DisplayFrameType.BodyJoints:
                    // instantiate a new Canvas
                    this.drawingCanvas = new Canvas();
                    // set the clip rectangle to prevent rendering outside the canvas
                    this.drawingCanvas.Clip = new RectangleGeometry();
                    this.drawingCanvas.Clip.Rect = new Rect(0.0, 0.0, this.BodyJointsGrid.Width, this.BodyJointsGrid.Height);
                    this.drawingCanvas.Width = this.BodyJointsGrid.Width;
                    this.drawingCanvas.Height = this.BodyJointsGrid.Height;
                    // reset the body joints grid
                    this.BodyJointsGrid.Visibility = Visibility.Visible;
                    this.BodyJointsGrid.Children.Clear();
                    // add canvas to DisplayGrid
                    this.BodyJointsGrid.Children.Add(this.drawingCanvas);
                    bodiesManager = new BodiesManager(this.coordinateMapper, this.drawingCanvas, this.kinectSensor.BodyFrameSource.BodyCount);
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

        //XAML onClick method
        private void BodyMask_Click(object sender, RoutedEventArgs e) {
            SetupCurrentDisplay(DisplayFrameType.BodyMask);
        }

        private void BodyJointsButton_Click(object sender, RoutedEventArgs e){
            SetupCurrentDisplay(DisplayFrameType.BodyJoints);
        }

        // To expose and manipulate the buffer behind the destination bitmap
        // you can use a Com interface called IBufferByteAccess through interop services
        [Guid("905a0fef-bc53-11df-8c49-001e4fc686da"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        interface IBufferByteAccess {
            unsafe void Buffer(out byte* pByte);
        }

    }
}
