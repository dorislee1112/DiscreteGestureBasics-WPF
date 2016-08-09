//------------------------------------------------------------------------------
// <copyright file="KinectBodyView.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DiscreteGestureBasics
{
    using System;
    using System.Collections.Generic;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;

    /// <summary>
    /// Visualizes the Kinect Body stream for display in the UI
    /// </summary>
    public sealed class KinectBodyView
    {
        public static bool doublectrlLeft2 = true;
        public static bool doublectrlRight2 = true;

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        private readonly Brush rect1Brush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 255));
        private readonly Brush rect2Brush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 128));
        private readonly Brush rect3Brush = new SolidColorBrush(Color.FromArgb(128, 255, 129, 66));
        private readonly Brush rect4Brush = new SolidColorBrush(Color.FromArgb(128, 255, 231, 112));
        private readonly Brush rectknock = new SolidColorBrush(Color.FromArgb(128, 255, 255,255));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Initializes a new instance of the KinectBodyView class
        /// </summary>
        /// <param name="kinectSensor">Active instance of the KinectSensor</param>
        public KinectBodyView(KinectSensor kinectSensor)
        {
            if (kinectSensor == null)
            {
                throw new ArgumentNullException("kinectSensor");
            }

            // get the coordinate mapper
            this.coordinateMapper = kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            //this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            //this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            //this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            //this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);
        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        double lastY_left=0;
        double lastY_right=0;
        double lastZ_left = 0;
        double lastZ_right = 0;
        double lastY_foot = 0;

        bool snareRctrl = false;

        /// <summary>
        /// Updates the body array with new information from the sensor
        /// Should be called whenever a new BodyFrameArrivedEvent occurs
        /// </summary>
        /// <param name="bodies">Array of bodies to update</param>
        public void UpdateBodyFrame(Body[] bodies)
        {
            if (bodies != null)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    

                    int penIndex = 0;
                    foreach (Body body in bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(body,joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                            
                            double hihatX = jointPoints[JointType.SpineMid].X -160;
                            double hihatY = jointPoints[JointType.SpineMid].Y -80;
                            double snareX = jointPoints[JointType.SpineMid].X - 40;
                            double snareY = jointPoints[JointType.SpineMid].Y ;
                            double crashX = jointPoints[JointType.SpineMid].X - 110;
                            double crashY = jointPoints[JointType.SpineMid].Y - 180;
                            double rideX = jointPoints[JointType.SpineMid].X + 120;
                            double rideY = jointPoints[JointType.SpineMid].Y -35;
                            double boardsize = 80;

                            dc.DrawRectangle(this.rect1Brush, null, new Rect(snareX, snareY, boardsize, boardsize));
                            dc.DrawRectangle(this.rect2Brush, null, new Rect(hihatX, hihatY, boardsize, boardsize));
                            dc.DrawRectangle(this.rect3Brush, null, new Rect(rideX, rideY, boardsize, boardsize));
                            dc.DrawRectangle(this.rect4Brush, null, new Rect(crashX, crashY, boardsize, boardsize));

                            ///////////////////////////
                            //右手打hihat
                            ///////////////////////////
                            if (jointPoints[JointType.HandTipRight].X > hihatX && jointPoints[JointType.HandTipRight].X < hihatX + boardsize
                                && jointPoints[JointType.HandTipRight].Y > hihatY && jointPoints[JointType.HandTipRight].Y < hihatY + boardsize
                                && GestureResultView.signRight == true && jointPoints[JointType.HandTipRight].Y - lastY_right >3)
                            //if (jointPoints[JointType.HandTipLeft].X > leftX && jointPoints[JointType.HandTipLeft].X < leftX + 90 && jointPoints[JointType.HandTipLeft].Y > leftY && jointPoints[JointType.HandTipLeft].Y < leftY + 90 && GestureResultView.doublectrlLeft1==true &&doublectrlLeft2 ==true  && jointPoints[JointType.HandTipLeft].Y > lastY_left)
                            {
                                //GestureResultView.wplayerhihat.controls.stop();
                                dc.DrawRectangle(this.rectknock, null, new Rect(hihatX, hihatY, boardsize, boardsize));
                                GestureResultView.wplayerhihat.controls.play();
                                GestureResultView.signRight = false;
                                doublectrlRight2 = false;
                                
                            }

                            ///////////////////////////
                            //左手打hihat
                            ///////////////////////////
                            if (jointPoints[JointType.HandTipLeft].X > hihatX && jointPoints[JointType.HandTipLeft].X < hihatX + boardsize
                               && jointPoints[JointType.HandTipLeft].Y > hihatY && jointPoints[JointType.HandTipLeft].Y < hihatY + boardsize
                               && GestureResultView.signLeft == true && jointPoints[JointType.HandTipLeft].Y - lastY_left >3)
                            //if (jointPoints[JointType.HandTipLeft].X > leftX && jointPoints[JointType.HandTipLeft].X < leftX + 90 && jointPoints[JointType.HandTipLeft].Y > leftY && jointPoints[JointType.HandTipLeft].Y < leftY + 90 && GestureResultView.doublectrlLeft1==true &&doublectrlLeft2 ==true  && jointPoints[JointType.HandTipLeft].Y > lastY_left)
                            {
                                //MusicStop();
                                //GestureResultView.wplayerhihat.controls.stop();
                                dc.DrawRectangle(this.rectknock, null, new Rect(hihatX, hihatY, boardsize, boardsize));
                                GestureResultView.wplayerhihat.controls.play();
                                GestureResultView.signLeft = false;
                                doublectrlLeft2 = false;

                            }


                            ///////////////////////////
                            //右手打snare
                            ///////////////////////////

                            if (jointPoints[JointType.HandTipRight].X > snareX && jointPoints[JointType.HandTipRight].X < snareX + boardsize
                                && jointPoints[JointType.HandTipRight].Y > snareY && jointPoints[JointType.HandTipRight].Y < snareY + boardsize 
                                && GestureResultView.signRight == true && jointPoints[JointType.HandTipRight].Y - lastY_right > 15)
                            //if (jointPoints[JointType.HandTipRight].X > rightX && jointPoints[JointType.HandTipRight].X < rightX + 90 && jointPoints[JointType.HandTipRight].Y > rightY && jointPoints[JointType.HandTipRight].Y < rightY + 90 && GestureResultView.doublectrlRight1==true && doublectrlRight2 == true && jointPoints[JointType.HandTipRight].Y > lastY_right)
                            {
                                //MusicStop();
                                GestureResultView.wplayersnare.controls.stop();
                                dc.DrawRectangle(this.rectknock, null, new Rect(snareX, snareY, boardsize, boardsize));
                                GestureResultView.wplayersnare.controls.play();
                                GestureResultView.signRight = false;
                                doublectrlRight2 = false;
                            }

                            /////////////////////////
                            //左手打snare
                            /////////////////////////

                            if (jointPoints[JointType.HandTipLeft].X > snareX && jointPoints[JointType.HandTipLeft].X < snareX + boardsize
                                && jointPoints[JointType.HandTipLeft].Y > snareY && jointPoints[JointType.HandTipLeft].Y < snareY + boardsize
                                && GestureResultView.signLeft == true && jointPoints[JointType.HandTipLeft].Y - lastY_left > 15)
                            {
                                //MusicStop();
                                GestureResultView.wplayersnare.controls.stop();
                                dc.DrawRectangle(this.rectknock, null, new Rect(snareX, snareY, boardsize, boardsize));
                                GestureResultView.wplayersnare.controls.play();
                                GestureResultView.signLeft = false;
                                doublectrlLeft2 = false;
                            }

                            ///////////////////
                            //右手打crash
                            ///////////////////
                            if (jointPoints[JointType.HandTipRight].X > crashX && jointPoints[JointType.HandTipRight].X < crashX + boardsize
                                && jointPoints[JointType.HandTipRight].Y > crashY && jointPoints[JointType.HandTipRight].Y < crashY + boardsize
                                && GestureResultView.signRight == true && jointPoints[JointType.HandTipRight].Y - lastY_right > 5
                                && lastZ_right - body.Joints[JointType.HandTipRight].Position.Z > 0)
                            //if (jointPoints[JointType.HandTipLeft].X > leftX && jointPoints[JointType.HandTipLeft].X < leftX + 90 && jointPoints[JointType.HandTipLeft].Y > leftY && jointPoints[JointType.HandTipLeft].Y < leftY + 90 && GestureResultView.doublectrlLeft1==true &&doublectrlLeft2 ==true  && jointPoints[JointType.HandTipLeft].Y > lastY_left)
                            {
                                //MusicStop();
                                GestureResultView.wplayercrash.controls.stop();
                                dc.DrawRectangle(this.rectknock, null, new Rect(crashX, crashY, boardsize, boardsize));
                                GestureResultView.wplayercrash.controls.play();
                                GestureResultView.signRight = false;
                                doublectrlLeft2 = false;

                            }

                            ///////////////////
                            //左手打crash
                            ///////////////////
                            if (jointPoints[JointType.HandTipLeft].X > crashX && jointPoints[JointType.HandTipLeft].X < crashX + boardsize
                                && jointPoints[JointType.HandTipLeft].Y > crashY && jointPoints[JointType.HandTipLeft].Y < crashY + boardsize
                                && GestureResultView.signLeft == true && jointPoints[JointType.HandTipLeft].Y - lastY_left > 5 
                                && lastZ_left - body.Joints[JointType.HandTipLeft].Position.Z > 0)
                            //if (jointPoints[JointType.HandTipLeft].X > leftX && jointPoints[JointType.HandTipLeft].X < leftX + 90 && jointPoints[JointType.HandTipLeft].Y > leftY && jointPoints[JointType.HandTipLeft].Y < leftY + 90 && GestureResultView.doublectrlLeft1==true &&doublectrlLeft2 ==true  && jointPoints[JointType.HandTipLeft].Y > lastY_left)
                            {
                                //MusicStop();
                                GestureResultView.wplayercrash.controls.stop();
                                dc.DrawRectangle(this.rectknock, null, new Rect(crashX, crashY, boardsize, boardsize));
                                GestureResultView.wplayercrash.controls.play();
                                GestureResultView.signLeft = false;
                                doublectrlLeft2 = false;

                            }

                            ///////////////////
                            //右手打ride
                            ///////////////////
                            
                            if (jointPoints[JointType.HandTipRight].X > rideX && jointPoints[JointType.HandTipRight].X < rideX + boardsize
                                && jointPoints[JointType.HandTipRight].Y > rideY && jointPoints[JointType.HandTipRight].Y < rideY + boardsize
                                && GestureResultView.signRight == true && jointPoints[JointType.HandTipRight].Y - lastY_right > 5 /*|| body.Joints[JointType.HandTipRight].Position.Z > lastZ_right)*/)// && lastZ_right-body.Joints[JointType.HandTipRight].Position.Z > 5)
                            //if (jointPoints[JointType.HandTipRight].X > rightX && jointPoints[JointType.HandTipRight].X < rightX + 90 && jointPoints[JointType.HandTipRight].Y > rightY && jointPoints[JointType.HandTipRight].Y < rightY + 90 && GestureResultView.doublectrlRight1==true && doublectrlRight2 == true && jointPoints[JointType.HandTipRight].Y > lastY_right)
                            {
                                //MusicStop();
                                GestureResultView.wplayerride.controls.stop();
                                dc.DrawRectangle(this.rectknock, null, new Rect(rideX, rideY, boardsize, boardsize));
                                GestureResultView.wplayerride.controls.play();
                                GestureResultView.signRight = false;
                                doublectrlRight2 = false;
                            }

                            ///////////////////
                            //腳
                            ///////////////////
                            //if (GestureResultView.signFoot == true && jointPoints[JointType.FootRight].Y-lastY_foot>7)
                            if (GestureResultView.signFoot == true && jointPoints[JointType.AnkleRight].Y - lastY_foot > 5)
                            {
                                GestureResultView.wplayerkick.controls.stop();
                                dc.DrawRectangle(this.rectknock, null, new Rect(jointPoints[JointType.FootRight].X - 15, jointPoints[JointType.FootRight].Y - 15, 30, 30));
                                GestureResultView.wplayerkick.controls.play();
                                GestureResultView.signFoot = false;
                            }

                            if (GestureResultView.doublectrlLeft1 == false) {
                                doublectrlLeft2 = true;
                            }
                            if (GestureResultView.doublectrlRight1 == false) {
                                doublectrlRight2 = true;
                            }
                            lastY_left = jointPoints[JointType.HandTipLeft].Y;
                            lastY_right=jointPoints[JointType.HandTipRight].Y;
                            lastZ_left = jointPoints[JointType.HandTipLeft].Y;
                            lastZ_right = jointPoints[JointType.HandTipRight].Y;
                            //lastY_foot = jointPoints[JointType.FootRight].Y;
                            lastY_foot = jointPoints[JointType.AnkleRight].Y;
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }


        private void MusicStop()
        {
            //GestureResultView.wplayerkick.controls.stop();
            //GestureResultView.wplayercrash.controls.stop();
            //GestureResultView.wplayersnare.controls.stop();
            //GestureResultView.wplayerhihat.controls.stop();
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(Body body,IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }


            }
            //Draw Rectangle
            //drawingContext.DrawRectangle(this.rect1Brush, null, new Rect(body.Joints[JointType.SpineMid].Position.X + 50, body.Joints[JointType.SpineMid].Position.Y + 50, 30, 30));
           // Console.Write("X: " + body.Joints[JointType.SpineMid].Position.X + " Y: " + body.Joints[JointType.SpineMid].Position.Y);
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
           
           
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
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }
    }
}
