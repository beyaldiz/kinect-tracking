namespace KinectProject
{
    using System;
    using System.IO;
    using System.IO.Ports;
    using System.Collections.Generic;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Windows.Threading;
    using System.Threading;
    using System.Runtime.InteropServices;

    public partial class MainWindow : Window
    {
        DispatcherTimer timer = new DispatcherTimer();

        private KinectSensor sensor;
        
        private WriteableBitmap colorBitmap;

        private DepthImagePixel[] depthPixels;

        private byte[] colorPixels;

        private Point mousePosition;

        private SerialPort serialPort1;

        private double servoMidX = 125d;

        private double servoMidY = 110d;

        private int sendServoX = 125;

        private int sendServoY = 110;

        private int lastX = 0;

        private int lastY = 0;

        private double maxB = 255, minB = 0, maxG = 255, minG = 0, maxR = 64, minR = 0;

        private byte[] filterForTracking = new byte[64];

        private bool buttonStart = false;

        private bool firstStart = false;

        private bool eventGuard = true;

        private int[,] markForArea = new int[480, 640];

        private int areaObject = 0;

        private int averageX = 0, averageY;

        private int minWhichArea = -1;

        private int nForFilter = 8;

        private int kForFilter = 32;

        public MainWindow()
        {
            InitializeComponent();
            timer.Interval = TimeSpan.FromSeconds(1d / 30d);
            timer.IsEnabled = true;
            timer.Tick += timerTick;
        }

        private void timerTick(object sender, EventArgs e)
        {
            int toIndex = XYToIndex(sendServoX, sendServoY);
            if(toIndex >= 0 && toIndex < 640 * 480 && buttonStart && !firstStart && lastX != sendServoX && lastY != sendServoY)
                SendServoFromPixel(toIndex);

            lastX = sendServoX;
            lastY = sendServoY;
        }

        private void InitSerialPort()
        {
            serialPort1 = new SerialPort("COM7", 9600, Parity.None, 8, StopBits.One)
            {
                Handshake = Handshake.None,
                Encoding = System.Text.Encoding.Default
            };
            return;
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the depth stream to receive depth frames
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

                // Allocate space to put the depth pixels we'll receive
                this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];

                // Allocate space to put the color pixels we'll create
                this.colorPixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                this.Image.Source = this.colorBitmap;

                this.sensor.DepthFrameReady += this.SensorDepthFrameReady;

                this.sensor.ColorFrameReady += this.SensorColorFrameReady;
                
                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }

                catch (IOException)
                {
                    this.sensor = null;
                }
            }
            InitSerialPort();
            
        }
        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null && eventGuard)
                {
                    eventGuard = false;
                    // Copy the pixel data from the image to a temporary array
                    colorFrame.CopyPixelDataTo(this.colorPixels);

                    for (int i = 0; i < colorPixels.Length; i += 4)
                    {
                        double r = 0, g = 0, b = 0;
                        b = colorPixels[i];
                        g = colorPixels[i + 1];
                        r = colorPixels[i + 2];

                        if (b <= maxB && b >= minB && g <= maxG && g >= minG && r >= minR && r <= maxR)
                        {
                            colorPixels[i] = 255;
                            colorPixels[i + 1] = 255;
                            colorPixels[i + 2] = 255;
                        }

                        else
                        {
                            colorPixels[i] = 0;
                            colorPixels[i + 1] = 0;
                            colorPixels[i + 2] = 0;
                        }
                    }

                    if (buttonStart)
                        for (int i = 0; i < 480; i += 8)
                        {
                            for (int j = 0; j < 640; j += 8)
                            {
                                int whitesNumForFilter = 0;
                                for (int m = i; m <= i + 7; m++)
                                {
                                    for (int n = j; n <= j + 7; n++)
                                    {
                                        int index = XYToIndex(m, n);
                                        if (colorPixels[index * 4] == 255)
                                            whitesNumForFilter++;
                                    }
                                }
                                if (whitesNumForFilter >= kForFilter)
                                {
                                    for (int t = i; t <= i + 7; t++)
                                    {
                                        for (int k = j; k <= j + 7; k++)
                                        {
                                            int index2 = XYToIndex(t, k);
                                            colorPixels[4 * index2] = 255;
                                            colorPixels[4 * index2 + 1] = 255;
                                            colorPixels[4 * index2 + 2] = 255;
                                        }
                                    }
                                }
                                else
                                {
                                    for (int t = i; t <= i + 7; t++)
                                    {
                                        for (int k = j; k <= j + 7; k++)
                                        {
                                            int index2 = XYToIndex(t, k);
                                            colorPixels[4 * index2] = 0;
                                            colorPixels[4 * index2 + 1] = 0;
                                            colorPixels[4 * index2 + 2] = 0;
                                        }
                                    }
                                }
                            }
                        }
                    if (firstStart && buttonStart)
                    {
                        compareAreas();
                        for (int i = 0; i < 480; i++)
                            for (int j = 0; j < 640; j++)
                            {
                                if(markForArea[i,j] == minWhichArea)
                                {
                                    averageX += i;
                                    averageY += j;
                                }
                                markForArea[i, j] = 0;
                            }

                        firstStart = false;
                        averageX = 0;
                        averageY = 0;
                    }

                    else if (buttonStart)
                    {
                         compareAreas();
                         for (int i = 0; i < 480; i++)
                            for (int j = 0; j < 640; j++)
                            {
                                if (markForArea[i, j] == minWhichArea)
                                {
                                    averageX += i;
                                    averageY += j;
                                }
                                markForArea[i, j] = 0;
                            }
                        
                        if(areaObject != 0)
                        {
                            sendServoX = averageX / areaObject;
                            sendServoY = averageY / areaObject;
                        }
                        
                        averageX = 0;
                        averageY = 0;
                    }

                    this.colorBitmap.WritePixels(
                        new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                        this.colorPixels,
                        this.colorBitmap.PixelWidth * sizeof(int),
                        0);
                    eventGuard = true;
                }
            }
        }

        private void compareAreas()
        {
            int whichArea = 0, min = Int32.MaxValue, maxAreaForFirst = Int32.MinValue, maxWhichAreaForFirst = -1, areaObjectFind = 0;
            for (int i = 0; i < colorPixels.Length; i += 4)
            {
                int i2 = i / 4;
                if (colorPixels[i] == 255 && markForArea[i2 / 640, i2 % 640] == 0)
                {
                    int t = areaFind(i2 / 640, i2 % 640, ++whichArea);
                    if (!firstStart && min > Math.Abs(areaObject - t))
                    {
                        areaObjectFind = t;
                        min = Math.Abs(areaObject - t);
                        minWhichArea = whichArea;
                    }
                    if(firstStart && maxAreaForFirst < t)
                    {
                        maxAreaForFirst = t;
                        areaObject = t;
                        maxWhichAreaForFirst = whichArea;
                    }
                }
            }
            if (!firstStart)
                areaObject = areaObjectFind;
        }

        private class info
        {
             public int x = 0, y = 0, areaName = 0;
        }


        private int areaFind(int x, int y, int areaNum)
        {
            int AAA = 0;
            Stack<info> S;
            S = new Stack<info>();
            info start1 = new info();
            info start2 = new info();
            start1.x = x;
            start1.y = y;
            start1.areaName = areaNum;
            S.Push(start1);
            while (S.Count != 0)
            {
                start1 = S.Pop();
                markForArea[start1.x, start1.y] = areaNum;
                if (start1.x + 1 < 480 && markForArea[start1.x + 1, start1.y] == 0 && colorPixels[4 * ((start1.x + 1) * 640 + start1.y)] == 255)
                {
                    AAA++;
                    start2 = new info();
                    start2.x = start1.x + 1;
                    start2.y = start1.y;
                    start2.areaName = start1.areaName;
                    markForArea[start2.x, start2.y] = areaNum;
                    S.Push(start2);
                }
                if (start1.x - 1 >= 0 && markForArea[start1.x - 1, start1.y] == 0 && colorPixels[4 * ((start1.x - 1) * 640 + start1.y)] == 255)
                {
                    AAA++;
                    start2 = new info();
                    start2.x = start1.x - 1;
                    start2.y = start1.y;
                    start2.areaName = start1.areaName;
                    markForArea[start2.x, start2.y] = areaNum;
                    S.Push(start2);
                }
                if (start1.y + 1 < 640 && markForArea[start1.x, start1.y + 1] == 0 && colorPixels[4 * (start1.x * 640 + start1.y + 1)] == 255)
                {
                    AAA++;
                    start2 = new info();
                    start2.x = start1.x;
                    start2.y = start1.y + 1;
                    start2.areaName = start1.areaName;
                    markForArea[start2.x, start2.y] = areaNum;
                    S.Push(start2);
                }
                if (start1.y - 1 >= 0 && markForArea[start1.x, start1.y - 1] == 0 && colorPixels[4 * (start1.x * 640 + start1.y - 1)] == 255)
                {
                    AAA++;
                    start2 = new info();
                    start2.x = start1.x;
                    start2.y = start1.y - 1;
                    start2.areaName = start1.areaName;
                    markForArea[start2.x, start2.y] = areaNum;
                    S.Push(start2);
                }
            }
            return AAA;
        }


        private int XYToIndex(int x, int y)
        {
            return x * 640 + y;
        }

        private int IndexToX(int index)
        {
            return index / 640;
        }

        private int IndexToY(int index)
        {
            return index % 640;
        }

        private void sliderMinB_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            minB = sliderMinB.Value;
            if (min_B != null)
                min_B.Text = ((int)minB).ToString();
        }
        private void sliderMaxB_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            maxB = sliderMaxB.Value;
            if (max_B != null)
                max_B.Text = ((int)maxB).ToString();
        }
        private void sliderMinG_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            minG = sliderMinG.Value;
            if (min_G != null)
                min_G.Text = ((int)minG).ToString();
        }
        private void sliderMaxG_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            maxG = sliderMaxG.Value;
            if (max_G != null)
                max_G.Text = ((int)maxG).ToString();
        }
        private void sliderMinR_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            minR = sliderMinR.Value;
            if (min_R != null)
                min_R.Text = ((int)minR).ToString();
        }
        private void sliderMaxR_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            maxR = sliderMaxR.Value;
            if(max_R != null)
                max_R.Text = ((int)maxR).ToString();
        }

        private void textBoxN_TextChanged(object sender, TextChangedEventArgs e)
        {
            int t;
            Int32.TryParse(textBoxN.Text, out t);
            nForFilter = t;
        }

        private void textBoxK_TextChanged(object sender, TextChangedEventArgs e)
        {
            int t;
            Int32.TryParse(textBoxK.Text, out t);
            kForFilter = t;
        }

        private void startButton_Click(object sender, RoutedEventArgs e)
        {
            buttonStart = !buttonStart;
            firstStart = true;
        }

        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);
            }
        }

        private void Image_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            Point mousePos = e.GetPosition(Image);
            double tmp = mousePos.Y;
            mousePos.Y = mousePos.X;
            mousePos.X = tmp;
            mousePosition = mousePos;
            mouseX.Text = mousePosition.X.ToString();
            mouseY.Text = mousePosition.Y.ToString();
        }

        private void Image_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            
        }

        private void sendToArduino(double angleX, double angleY)
        {
            serialPort1.Open();
            serialPort1.Write(angleY.ToString("000") + angleX.ToString("000"));
            serialPort1.Close();
        }

        private void SendServoFromPixel(int indexPixel)
        {
            double mPX = indexPixel / 640;
            double mPY = indexPixel % 640;
            double tan28bucuk = Math.Tan(28.5d * Math.PI / 180d);
            double tan21bucuk = Math.Tan(21.5d * Math.PI / 180d);
            double ratioX = tan28bucuk / 320;
            double ratioY = tan21bucuk / 240;
            double tanAciX = 0;
            double tanAciY = 0;
            double aciX = 0;
            double aciY = 0;

            tanAciX = (320 - mPY) * ratioX;
            aciX = Math.Atan(tanAciX) * 180 / Math.PI;
            double xAci = servoMidX - aciX;

            tanAciY = (240 - mPX) * ratioY;
            aciY = Math.Atan(tanAciY) * 180 / Math.PI;
            double servoToSensor = 450; //mm
            int depthToObject = depthPixels[indexPixel].Depth;
            aciY = Math.Atan((((servoToSensor * Math.Sqrt(tanAciY * tanAciY + 1)) / depthToObject) + tanAciY)) * 180 / Math.PI;
            double yAci = servoMidY + aciY;

            sendToArduino(xAci, yAci);
        }
    }
}
