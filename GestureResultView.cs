﻿//------------------------------------------------------------------------------
// <copyright file="GestureResultView.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DiscreteGestureBasics
{
    using System;
    using System.ComponentModel;
    using System.Runtime.CompilerServices;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Media;
   
    /// <summary>
    /// Stores discrete gesture results for the GestureDetector.
    /// Properties are stored/updated for display in the UI.
    /// </summary>
    public sealed class GestureResultView : INotifyPropertyChanged
    {
        /// <summary> Image to show when the 'detected' property is true for a tracked body </summary>
        private readonly ImageSource seatedImage = new BitmapImage(new Uri(@"Images\Seated.png", UriKind.Relative));

        /// <summary> Image to show when the 'detected' property is false for a tracked body </summary>
        private readonly ImageSource LeftHandImage = new BitmapImage(new Uri(@"Images\LeftHand.png", UriKind.Relative));

        private readonly ImageSource RightHandImage = new BitmapImage(new Uri(@"Images\RightHand.png", UriKind.Relative));

        private readonly ImageSource FootImage = new BitmapImage(new Uri(@"Images\Foot.png", UriKind.Relative));

        /// <summary> Image to show when the body associated with the GestureResultView object is not being tracked </summary>
        private readonly ImageSource notTrackedImage = new BitmapImage(new Uri(@"Images\NotTracked.png", UriKind.Relative));

        /// <summary> Array of brush colors to use for a tracked body; array position corresponds to the body colors used in the KinectBodyView class </summary>
        private readonly Brush[] trackedColors = new Brush[] { Brushes.Red, Brushes.Orange, Brushes.Green, Brushes.Blue, Brushes.Indigo, Brushes.Violet };

        /// <summary> Brush color to use as background in the UI </summary>
        private Brush bodyColor = Brushes.Gray;

        /// <summary> The body index (0-5) associated with the current gesture detector </summary>
        private int bodyIndex = 0;

        /// <summary> Current confidence value reported by the discrete gesture </summary>
        private float confidence = 0.0f;

        /// <summary> True, if the discrete gesture is currently being detected </summary>
        private bool detected = false;

        /// <summary> Image to display in UI which corresponds to tracking/detection state </summary>
        private ImageSource imageSource = null;

        /// <summary> True, if the body is currently being tracked </summary>
        private bool isTracked = false;

        public static bool signLeft = false;
        public static bool signRight = false;
        public static bool signFoot = false;
        public static SoundPlayer sound;
        public static SoundPlayer soundleft;
        public static WMPLib.WindowsMediaPlayer wplayerhihat;
        public static WMPLib.WindowsMediaPlayer wplayersnare;
        public static WMPLib.WindowsMediaPlayer wplayerride;
        public static WMPLib.WindowsMediaPlayer wplayerkick;
        public static WMPLib.WindowsMediaPlayer wplayercrash;
        public static bool doublectrlLeft1 = false;
        public static bool doublectrlRight1 = false;

        /// <summary>
        /// Initializes a new instance of the GestureResultView class and sets initial property values
        /// </summary>
        /// <param name="bodyIndex">Body Index associated with the current gesture detector</param>
        /// <param name="isTracked">True, if the body is currently tracked</param>
        /// <param name="detected">True, if the gesture is currently detected for the associated body</param>
        /// <param name="confidence">Confidence value for detection of the 'Seated' gesture</param>
        public GestureResultView(int bodyIndex, bool isTracked, bool detected, float confidence)
        {
            this.BodyIndex = bodyIndex;
            this.IsTracked = isTracked;
            this.Detected = detected;
            this.Confidence = confidence;
            this.ImageSource = this.notTrackedImage;
            /*
            sound = new SoundPlayer();
            sound.SoundLocation = "7788.wav";
            sound.Load();
            soundleft = new SoundPlayer();
            soundleft.SoundLocation = "snare.wav";
            soundleft.Load();
            */
          
            wplayerhihat = new WMPLib.WindowsMediaPlayer();
            wplayerhihat.URL = @"22050-32.mp3";
            wplayersnare = new WMPLib.WindowsMediaPlayer();
            wplayersnare.URL = @"1600-40-snare.mp3";
            wplayercrash = new WMPLib.WindowsMediaPlayer();
            wplayercrash.URL = @"crash-1.mp3";
            wplayerride = new WMPLib.WindowsMediaPlayer();
            wplayerride.URL = @"ride.mp3";
            wplayerkick = new WMPLib.WindowsMediaPlayer();
            wplayerkick.URL = @"kick-1.mp3";

        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary> 
        /// Gets the body index associated with the current gesture detector result 
        /// </summary>
        public int BodyIndex
        {
            get
            {
                return this.bodyIndex;
            }

            private set
            {
                if (this.bodyIndex != value)
                {
                    this.bodyIndex = value;
                    this.NotifyPropertyChanged();
                }
            }
        }

        /// <summary> 
        /// Gets the body color corresponding to the body index for the result
        /// </summary>
        public Brush BodyColor
        {
            get
            {
                return this.bodyColor;
            }

            private set
            {
                if (this.bodyColor != value)
                {
                    this.bodyColor = value;
                    this.NotifyPropertyChanged();
                }
            }
        }

        /// <summary> 
        /// Gets a value indicating whether or not the body associated with the gesture detector is currently being tracked 
        /// </summary>
        public bool IsTracked
        {
            get
            {
                return this.isTracked;
            }

            private set
            {
                if (this.IsTracked != value)
                {
                    this.isTracked = value;
                    this.NotifyPropertyChanged();
                }
            }
        }

        /// <summary> 
        /// Gets a value indicating whether or not the discrete gesture has been detected
        /// </summary>
        public bool Detected
        {
            get
            {
                return this.detected;
            }

            private set
            {
                if (this.detected != value)
                {
                    this.detected = value;
                    this.NotifyPropertyChanged();
                }
            }
        }

        /// <summary> 
        /// Gets a float value which indicates the detector's confidence that the gesture is occurring for the associated body 
        /// </summary>
        public float Confidence
        {
            get
            {
                return this.confidence;
            }

            private set
            {
                if (this.confidence != value)
                {
                    this.confidence = value;
                    this.NotifyPropertyChanged();
                }
            }
        }

        /// <summary> 
        /// Gets an image for display in the UI which represents the current gesture result for the associated body 
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }

            private set
            {
                if (this.ImageSource != value)
                {
                    this.imageSource = value;
                    this.NotifyPropertyChanged();
                }
            }
        }

        /// <summary>
        /// Updates the values associated with the discrete gesture detection result
        /// </summary>
        /// <param name="isBodyTrackingIdValid">True, if the body associated with the GestureResultView object is still being tracked</param>
        /// <param name="isGestureDetected">True, if the discrete gesture is currently detected for the associated body</param>
        /// <param name="detectionConfidence">Confidence value for detection of the discrete gesture</param>
        public void UpdateGestureResult(bool isBodyTrackingIdValid, bool isGestureDetected, float detectionConfidence, string side)
        {
            this.IsTracked = isBodyTrackingIdValid;
            this.Confidence = 0.0f;

            if (!this.IsTracked)
            {
                this.ImageSource = this.notTrackedImage;
                this.Detected = false;
                this.BodyColor = Brushes.Gray;
            }
            else
            {
                this.Detected = isGestureDetected;
                this.BodyColor = this.trackedColors[this.BodyIndex];

                if (this.Detected)
                {
                    this.Confidence = detectionConfidence;
                    //this.ImageSource = this.seatedImage;

                    sound.Play();

                    Console.Write(side + "\n");
                    if (side.Equals("Left_hand"))
                        this.ImageSource = this.LeftHandImage;
                    if (side.Equals("Right_hand"))
                        this.ImageSource = this.RightHandImage;
                    if (side.Equals("foot"))
                        this.ImageSource = this.FootImage;

                    //sound.Play();
                }
                else
                {
                    this.ImageSource = this.seatedImage;
                }
            }
        }

        public void UpdateContinuousGestureResult(bool isBodyTrackingIdValid, float fProgress, String side)
        // public void UpdateContinuousGestureResult(bool isBodyTrackingIdValid, bool isGestureDetected, String side)
        {
            this.IsTracked = isBodyTrackingIdValid;
            //this.fProgress = 0.0f;

            if (!this.IsTracked)
            {
                this.ImageSource = this.notTrackedImage;
                //this.Detected = false;
                this.BodyColor = Brushes.Gray;
            }
            else
            {
                //this.Detected = isGestureDetected;
                this.BodyColor = this.trackedColors[this.BodyIndex];
                //this.BodyColor = Brushes.Gray;

                //if (fProgress > 0.75f)
                //if (this.Detected)
                // {
                // this.Confidence = detectionConfidence;
                //this.BodyColor = Brushes.White;


                //Console.Write(fProgress + "\n");

                if (side.Equals("footProgress"))
                {
                    this.ImageSource = this.FootImage;
                    signFoot = true;
             //       Console.Write("foot  :" + fProgress + "\n");
                }

                if (side.Equals("rideProgress_Right"))
                {
                    if (fProgress > 0.5f)
                    {
                        this.ImageSource = this.RightHandImage;
                        //  Console.Write("right  :" + fProgress + "\n");
                        signRight = true;
                        //wplayerkick.controls.play();
                        doublectrlRight1 = true;
                    }
                }
                if (side.Equals("hihatProgress_Right"))
                {
                    if (fProgress > 0.6f)
                    {
                        this.ImageSource = this.RightHandImage;
                        signRight = true;
                        doublectrlRight1 = true;
                    }
                }

                if (side.Equals("hihatProgress_Left"))
                {
                    if (fProgress > 0.6f)
                    {
                        this.ImageSource = this.LeftHandImage;
                        signLeft = true;
                        doublectrlLeft1 = true;
                    }
                }
                if (side.Equals("snareProgress_Right"))
                {
                    if (fProgress > 0.6f)
                    {
                        this.ImageSource = this.RightHandImage;
                        //  Console.Write("right  :" + fProgress + "\n");
                        signRight = true;
                        doublectrlRight1 = true;
                    }
                }
                if (side.Equals("snareProgress_Right"))
                {
                    if (fProgress > 0.6f)
                    {
                        this.ImageSource = this.RightHandImage;
                        //  Console.Write("right  :" + fProgress + "\n");
                        signRight = true;
                        doublectrlRight1 = true;
                    }
                }

                if (side.Equals("snareProgress_Left"))
                {
                    if (fProgress > 0.6f)
                    {
                        this.ImageSource = this.LeftHandImage;
                        //  Console.Write("right  :" + fProgress + "\n");
                        signLeft = true;
                        doublectrlLeft1 = true;
                    }
                }
                if (side.Equals("crashProgress_Right"))
                {
                    if (fProgress > 0.6f)
                    {
                        this.ImageSource = this.RightHandImage;
                        //  Console.Write("right  :" + fProgress + "\n");
                        signRight = true;
                        doublectrlRight1 = true;
                    }
                }

                if (side.Equals("crashProgress_Left"))
                {
                    if (fProgress > 0.6f)
                    {
                        this.ImageSource = this.LeftHandImage;
                        //  Console.Write("right  :" + fProgress + "\n");
                        signLeft = true;
                        doublectrlLeft1 = true;
                    }
                }
            //}
                else
                {
                    this.ImageSource = this.seatedImage;
                }
            }
        }


        /// <summary>
        /// Notifies UI that a property has changed
        /// </summary>
        /// <param name="propertyName">Name of property that has changed</param> 
        private void NotifyPropertyChanged([CallerMemberName] string propertyName = "")
        {
            if (this.PropertyChanged != null)
            {
                this.PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
    }
}