// Copyright 2020 Dustin Dobransky

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;

namespace Rotrics.DexArm
{
    public class DexArm : IDexArm, IDisposable
    {
        private string portName;
        private SerialPort serialPort;
        private List<string> commandResponses = new List<string>();

        private DateTime softRebootTimestamp = DateTime.MinValue;
        private TimeSpan powerUpTime = TimeSpan.FromMilliseconds(3500);
        private bool rebooted = false;

        private Vector3 lastEncoderPosition = Vector3.Zero;
        private DateTime lastIsMovingPollTime = DateTime.MinValue;
        private TimeSpan isMovingPollPeriod = TimeSpan.FromMilliseconds(50);

        private DateTime timeLastMoveCommandSent = DateTime.MinValue;
        private TimeSpan timeForEncoderToUpdateAfterMoveCommandSent = TimeSpan.FromMilliseconds(160);

        public bool PrintCommand { get; set; } = true;
        public bool PrintResponse { get; set; } = true;
        public bool PrintReceivedCommandResponses { get; set; } = true;
        public int ResponseRetries { get; set; } = 50;

        private uint mmPerMinute = 1000;

        public uint DefaultSpeed => mmPerMinute;


        public DexArmMoveMode DefaultMoveMode { get; set; } = DexArmMoveMode.FastMode;

        public DexArmPositioningMode PositioningMode { get; private set; } = DexArmPositioningMode.Absolute;

        public DexArmModule Module { get; private set; } = DexArmModule.PenHolder;

        public IDexArmSettings Settings => settings;

        private DexArmSettings settings = new DexArmSettings()
        {
            MinSegmentTimeMicroseconds = 20000.00f,
            MinFeedrate = 0.00f,
            MinTravelFeedrate = 0.00f,
            MaxXJerk = 10.00f,
            MaxYJerk = 10.00f,
            MaxZJerk = 10.00f,
            MaxEJerk = 5.00f
        };

        public event EventHandler<SerialPortReadLineArgs> NewSerialPortMessage;
        private CancellationTokenSource serialPortMessageCancellationTokenSource;

        private readonly Channel<string> serialPortChannel;
        private readonly ChannelWriter<string> serialPortMessageWriter;
        private readonly ChannelReader<string> serialPortMessageReader;

        private StringBuilder sb = new StringBuilder();
        private static readonly string[] newLineDelimiter = new string[] { Environment.NewLine };

        public DexArm(string portName)
        {
            this.portName = portName;
            this.serialPortChannel = Channel.CreateUnbounded<string>();
            this.serialPortMessageWriter = this.serialPortChannel.Writer;
            this.serialPortMessageReader = this.serialPortChannel.Reader;
        }

        public bool Init()
        {
            // Serial port won't be null if SoftReboot is called
            if (this.serialPort == null)
            {
                serialPort = new SerialPort(this.portName);

                serialPort.BaudRate = 115200;
                serialPort.DataBits = 8;
                serialPort.StopBits = StopBits.One;
                serialPort.Parity = Parity.None;
                serialPort.Handshake = Handshake.RequestToSend;
                serialPort.DtrEnable = true;
                serialPort.NewLine = "\n";

                Trace.WriteLine("Created new serial port");
            }

            // Handle soft reboot case
            if (rebooted)
            {
                TimeSpan timeSinceSoftReboot = DateTime.UtcNow - this.softRebootTimestamp;
                Trace.WriteLine("Time since reboot: " + timeSinceSoftReboot.TotalMilliseconds);
                if (timeSinceSoftReboot < powerUpTime)
                {
                    Trace.WriteLine("Sleeping for : " + (powerUpTime - timeSinceSoftReboot));
                    Thread.Sleep(powerUpTime - timeSinceSoftReboot);
                }
                rebooted = false;
            }

            // Initialize DexArm
            if (!this.serialPort.IsOpen)
            {
                this.serialPort.Open();
                Trace.WriteLine("Opened serial port");

                // Hacky workaround for a bug where upon first powerup or reboot,
                // DexArm responds with "??????" when the first command is sent.
                if (this.serialPort.IsOpen)
                {
                    // Only looks for "ok" response, any other garbage is discarded
                    this.Dwell(TimeSpan.FromMilliseconds(1));

                    // Get intial arm position. Necessary for IsMoving command.
                    this.lastEncoderPosition = this.GetEncoderPosition();
                }
                else
                {
                    throw new IOException($"Could not connect to DexArm on Port '{this.serialPort.PortName}'");
                }
            }

            //serialPort.DataReceived += this.OnSerialPortDataReceived;
            this.serialPortMessageCancellationTokenSource?.Cancel();
            this.serialPortMessageCancellationTokenSource = new CancellationTokenSource();
            Task.Run(() => this.PublishSerialPortReadLineMessage(serialPortMessageCancellationTokenSource.Token));

            return this.serialPort.IsOpen;
        }

        private void PublishSerialPortReadLineMessage(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                string message = this.serialPort.ReadLine();

                //this.serialPortMessages.Enqueue(message);
                //this.serialPortMessagesBlocking.Add(message);
                this.serialPortMessageWriter.TryWrite(message);

                this.NewSerialPortMessage?.Invoke(this, new SerialPortReadLineArgs(message));
            }
        }

        //private void OnSerialPortDataReceived(object sender, SerialDataReceivedEventArgs e)
        //{
        //    SerialPort port = (SerialPort)sender;
        //    string input = port.ReadExisting();

        //    // If there is a newline in the message, a complete message can be built.
        //    // Build the full message and publish to listeners.
        //    if (input.Contains(Environment.NewLine))
        //    {
        //        string[] lines = input.Split(newLineDelimiter, StringSplitOptions.RemoveEmptyEntries);
                
        //        if (lines.Length > 0)
        //        {
        //            sb.Append(lines[0]);
        //            sb.Append(Environment.NewLine);
        //        }

        //        string message = sb.ToString();

        //        sb.Clear();
        //        if (lines.Length > 1)
        //        {
        //            sb.Append(lines[1]);
        //        }

        //        // Publish message to all listeners
        //        this.NewSerialPortMessage.Invoke(this, new SerialPortReadLineArgs(message));
        //    } 
        //    else
        //    {
        //        sb.Append(input);
        //    }
        //}

        private Task<string> SerialPortReadLineAsync() 
        {
            TaskCompletionSource<string> tsc = new TaskCompletionSource<string>();
            EventHandler<SerialPortReadLineArgs> handler = null;
            handler = (sender, args) =>
            {
                this.NewSerialPortMessage -= handler;
                tsc.SetResult(args.Message);
            };

            this.NewSerialPortMessage += handler;

            return tsc.Task;
        }

        //private void OnNewSerialPortReadLineMessage(object sender, SerialPortReadLineArgs args)
        //{
        //    this.serialPortMessages.Enqueue(args.Message);
        //}

        private async Task<(bool, object)> ProcessResponseAsync(DexArmCommand command, CancellationToken token = default)
        {
            this.commandResponses.Clear();

            int responseAttempts = 0;
            bool successfulResponse = false;

            do
            {
                //string response = await this.SerialPortReadLineAsync();
                //this.serialPortMessagesBlocking.Take();
                //if (this.serialPortMessages.TryDequeue(out string response))
                //string response = this.serialPortMessagesBlocking.Take(token);
                string response = await this.serialPortMessageReader.ReadAsync(token);

                if (token.IsCancellationRequested)
                {
                    break;
                }
                    
                if (this.PrintReceivedCommandResponses)
                {
                    Trace.WriteLine("ProcessResponseAsync: " + response);
                }

                if (response.StartsWith("wait") == false)
                {
                    this.commandResponses.Add(response);
                }

                if (response.StartsWith("ok"))
                {
                    successfulResponse = true;
                    break;
                }
            }
            while (responseAttempts++ < this.ResponseRetries);

            if (this.PrintResponse)
            {
                Trace.WriteLine($"Responses [{this.commandResponses.Count}]");
                Trace.Indent();
                foreach (string log in this.commandResponses)
                {
                    Trace.WriteLine($"| {log}");
                }
                Trace.Unindent();
            }

            if (!successfulResponse || token.IsCancellationRequested)
            {
                return (false, null);
            }

            bool success = this.ProcessCommandResponses(command, this.commandResponses, out object output);
            return (success, output);
        }

        private bool ProcessResponse(DexArmCommand command, out object output)
        {
            this.commandResponses.Clear();

            int responseAttempts = 0;
            bool successfulResponse = false;
            do
            {
                string response = this.serialPort.ReadLine();
                if (response.StartsWith("wait") == false)
                {
                    this.commandResponses.Add(response);
                }
                if (response.StartsWith("ok"))
                {
                    successfulResponse = true;
                    break;
                }
            }
            while (responseAttempts++ < this.ResponseRetries);

            if (this.PrintResponse)
            {
                Trace.WriteLine($"Responses [{this.commandResponses.Count}]");
                Trace.Indent();
                foreach (string log in this.commandResponses)
                {
                    Trace.WriteLine($"| {log}");
                }
                Trace.Unindent();
            }

            if (!successfulResponse)
            {
                output = null;
                return false;
            }

            return this.ProcessCommandResponses(command, this.commandResponses, out output);
        }

        private bool ProcessCommandResponses(DexArmCommand command, List<string> responses, out object output)
        {
            output = null;
            switch (command)
            {
                case DexArmCommand.Ok:
                    return true;
                case DexArmCommand.GoHome:
                    return responses[0].StartsWith("M1112");
                case DexArmCommand.GetXyAxisSlope:
                    {
                        Match xAxisSlopeMatch = Regex.Match(responses[responses.Count - 3], @"GET X AXIS SLOPE IS (?<xSlope>-?\d+\.\d+)");
                        Match yAxisSlopeMatch = Regex.Match(responses[responses.Count - 2], @"GET Y AXIS SLOPE IS (?<ySlope>-?\d+\.\d+)");

                        if (!xAxisSlopeMatch.Success || !yAxisSlopeMatch.Success)
                        {
                            output = Vector2.Zero;
                            return false;
                        }

                        try
                        {
                            output = new Vector2(
                                float.Parse(xAxisSlopeMatch.Groups["xSlope"].Value),
                                float.Parse(yAxisSlopeMatch.Groups["ySlope"].Value));
                            return true;
                        }
                        catch
                        {
                            return false;
                        }
                    }
                case DexArmCommand.GetJointAngles:
                    {
                        Match jointAnglesMatch = Regex.Match(responses[2], @"DEXARM Theta A:(?<A>-?\d+\.\d+)  Theta B:(?<B>-?\d+\.\d+)  Theta C:(?<C>-?\d+\.\d+)");
                        if (!jointAnglesMatch.Success)
                        {
                            output = Vector3.Zero;
                            return false;
                        }

                        GroupCollection jointAngle = jointAnglesMatch.Groups;
                        try
                        {
                            output = new Vector3(
                               float.Parse(jointAngle["A"].Value),
                               float.Parse(jointAngle["B"].Value),
                               float.Parse(jointAngle["C"].Value));

                            return true;
                        }
                        catch
                        {
                            output = Vector3.Zero;
                            return false;
                        }
                    }
                case DexArmCommand.GetEncoderPosition:
                    {
                        Match encoderPositionMatch = Regex.Match(responses[0], @"M894 X(?<X>-?\d+\.?\d+) Y(?<Y>-?\d+\.?\d+) Z(?<Z>-?\d+\.?\d+)");
                        if (!encoderPositionMatch.Success)
                        {
                            output = Vector3.Zero;
                            return false;
                        }

                        GroupCollection encoderPosition = encoderPositionMatch.Groups;
                        try
                        {
                            output = new Vector3(
                                float.Parse(encoderPosition["X"].Value),
                                float.Parse(encoderPosition["Y"].Value),
                                float.Parse(encoderPosition["Z"].Value));

                            return true;
                        }
                        catch
                        {
                            output = Vector3.Zero;
                            return false;
                        }
                    }
                case DexArmCommand.IsMoving:
                    return false;
                case DexArmCommand.GetCurrentPosition:
                    {
                        Match positionMatch = Regex.Match(responses[0], @"X:(?<X>-?\d+\.\d+) Y:(?<Y>-?\d+\.\d+) Z:(?<Z>-?\d+\.\d+)");
                        if (!positionMatch.Success)
                        {
                            output = Vector3.Zero;
                            return false;
                        }

                        GroupCollection position = positionMatch.Groups;
                        try
                        {
                            output = new Vector3(
                               float.Parse(position["X"].Value),
                               float.Parse(position["Y"].Value),
                               float.Parse(position["Z"].Value));

                            return true;
                        }
                        catch
                        {
                            output = Vector3.Zero;
                            return false;
                        }
                    }
                case DexArmCommand.GetAxisAcceleration:
                    {
                        Match axisAccelMatch = Regex.Match(responses[10], @"M201 X(?<X>-?\d+\.\d+) Y(?<Y>-?\d+\.\d+) Z(?<Z>-?\d+\.\d+) E(?<E>-?\d+\.\d+)");
                        if (!axisAccelMatch.Success)
                        {
                            output = Vector4.Zero;
                            return false;
                        }

                        GroupCollection axisAccel = axisAccelMatch.Groups;
                        try
                        {
                            output = new Vector4(
                               float.Parse(axisAccel["X"].Value),
                               float.Parse(axisAccel["Y"].Value),
                               float.Parse(axisAccel["Z"].Value),
                               float.Parse(axisAccel["E"].Value));

                            return true;
                        }
                        catch
                        {
                            output = Vector4.Zero;
                            return false;
                        }
                    }
                case DexArmCommand.Get3DPrintingAcceleration:
                    {
                        Match accel3DMatch = Regex.Match(responses[12], @"M204 P(?<P>-?\d+\.\d+) R(?<R>-?\d+\.\d+) T(?<T>-?\d+\.\d+)");
                        if (!accel3DMatch.Success)
                        {
                            output = Vector3.Zero;
                            return false;
                        }

                        GroupCollection accel3D = accel3DMatch.Groups;
                        try
                        {
                            output = new Vector3(
                               float.Parse(accel3D["P"].Value),
                               float.Parse(accel3D["R"].Value),
                               float.Parse(accel3D["T"].Value));

                            return true;
                        }
                        catch
                        {
                            output = Vector4.Zero;
                            return false;
                        }
                    }
                case DexArmCommand.GetModuleOffset:
                    {
                        output = 0.0f;
                        return true;
                    }
                case DexArmCommand.GetModule:
                    {
                        string response = responses[0];
                        if (response.Contains("PEN"))
                        {
                            output = DexArmModule.PenHolder;
                            return true;
                        }
                        else if (response.Contains("LASER"))
                        {
                            output = DexArmModule.LaserEngraver;
                            return true;
                        }
                        else if (response.Contains("PUMP"))
                        {
                            output = DexArmModule.PneumaticModule;
                            return true;
                        }
                        else if (response.Contains("3D"))
                        {
                            output = DexArmModule.Printer3D;
                            return true;
                        }
                        return false;
                    }
                case DexArmCommand.ReportSettings:
                    {
                        output = string.Join(Environment.NewLine, responses);
                        return true;
                    }
                default:
                    return false;
            }
        }

        public void SendCommand(string command, [CallerMemberName] string memberName = "")
        {
            if (this.PrintCommand)
            {
                Trace.WriteLine($"Command {memberName}: {command}");
            }

            this.serialPort.WriteLine(command);
        }
        
        public bool SetPosition(Vector3 position)
        {
            return this.SetPosition(position.X, position.Y, position.Z);
        }

        public bool SetPosition(float x, float y, float z)
        {
            this.SendCommand($"G{(int)this.DefaultMoveMode} X{x} Y{y} Z{z}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.timeLastMoveCommandSent = DateTime.UtcNow;
                return true;
            }

            return false;
        }

        public async Task<bool> SetPositionAsync(float x, float y, float z)
        {
            this.SendCommand($"G{(int)this.DefaultMoveMode} X{x} Y{y} Z{z}");

            (bool success, _) = await this.ProcessResponseAsync(DexArmCommand.Ok);

            return success;                
        }

        public async Task<bool> SetPositionAsync(float x, float y, float z, DexArmMoveMode moveMode)
        {
            this.SendCommand($"G{(int)moveMode} X{x} Y{y} Z{z}");

            (bool success, _) = await this.ProcessResponseAsync(DexArmCommand.Ok);

            return success;
        }

        public async Task<bool> SetPositionAsync(Vector3 xyz, DexArmMoveMode moveMode)
        {
            return await this.SetPositionAsync(xyz.X, xyz.Y, xyz.Z, moveMode);
        }

        public bool SetPosition(Vector3 position, DexArmMoveMode moveMode)
        {
            return this.SetPosition(position.X, position.Y, position.Z, moveMode);
        }

        public bool SetPosition(float x, float y, float z, DexArmMoveMode moveMode)
        {
            this.SendCommand($"G{(int)moveMode} X{x} Y{y} Z{z}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.timeLastMoveCommandSent = DateTime.UtcNow;
                return true;
            }

            return false;
        }
        public bool SetPosition(Vector3 position, uint mmPerMinute)
        {
            return this.SetPosition(position.X, position.Y, position.Z, mmPerMinute);
        }

        public bool SetPosition(float x, float y, float z, uint mmPerMinute)
        {
            this.SendCommand($"G{(int)this.DefaultMoveMode} F{mmPerMinute} X{x} Y{y} Z{z}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.timeLastMoveCommandSent = DateTime.UtcNow;
                return true;
            }

            return false;
        }

        public bool SetPosition(Vector3 position, uint mmPerMinute, DexArmMoveMode moveMode)
        {
            return this.SetPosition(position.X, position.Y, position.Z, mmPerMinute, moveMode);
        }

        public bool SetPosition(float x, float y, float z, uint mmPerMinute, DexArmMoveMode moveMode)
        {
            this.SendCommand($"G{(int)moveMode} F{mmPerMinute} X{x} Y{y} Z{z}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.timeLastMoveCommandSent = DateTime.UtcNow;
                return true;
            }

            return false;
        }

        public Vector3 GetCurrentPosition()
        {
            this.SendCommand("M114");

            if (this.ProcessResponse(DexArmCommand.GetCurrentPosition, out object position))
            {
                return (Vector3)position;
            }

            return Vector3.Zero;
        }

        public async Task<Vector3> GetCurrentPositionAsync()
        {
            this.SendCommand("M114");

            (bool success, object output) = await this.ProcessResponseAsync(DexArmCommand.GetCurrentPosition);

            if (success)
            {
                return (Vector3)output;
            }

            return Vector3.Zero;
        }

        public Vector3 GetEncoderPosition()
        {
            this.SendCommand("M893");

            if (this.ProcessResponse(DexArmCommand.GetEncoderPosition, out object encoderPosition))
            {
                return (Vector3)encoderPosition;
            }

            return Vector3.Zero;
        }

        public Vector3 GetJointAngles()
        {
            this.SendCommand("M114");

            if (this.ProcessResponse(DexArmCommand.GetJointAngles, out object jointAngles))
            {
                return (Vector3)jointAngles;
            }

            return Vector3.Zero;
        }

        public bool Set3DPrintingAcceleration(int acceleration, int travelAcceleration, int retractAcceleration)
        {
            this.SendCommand($"M204 P{acceleration} T{travelAcceleration} R{retractAcceleration}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetAxisAcceleration(int x, int y, int z, int e)
        {
            this.SendCommand($"M201 E{e} X{x} Y{y} Z{z}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public Vector4 GetAxisAcceleration()
        {
            this.SendCommand("M503");
            
            if (this.ProcessResponse(DexArmCommand.GetAxisAcceleration, out object axisAcceleration))
            {
                return (Vector4)axisAcceleration;
            }

            return Vector4.Zero;
        }

        public Vector3 Get3DPrintingAcceleration()
        {
            this.SendCommand("M503");

            if (this.ProcessResponse(DexArmCommand.Get3DPrintingAcceleration, out object printingAcceleration))
            {
                return (Vector3)printingAcceleration;
            }

            return Vector3.Zero;
        }

        public bool GoHome()
        {
            this.SendCommand("M1112");
            return this.ProcessResponse(DexArmCommand.GoHome, out _);
        }

        public async Task<bool> GoHomeAsync()
        {
            this.SendCommand("M1112");

            (bool success, _) = await this.ProcessResponseAsync(DexArmCommand.Ok);

            return success;
        }

        public bool SetWorkOrigin()
        {
            this.SendCommand("G92 X0 Y0 Z0 E0");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetAdvancedParams(
            float minSegmentTimeMicroseconds = 20000.00f,
            float minFeedrate = 0.00f,
            float minTravelFeedrate = 0.00f,
            float maxXJerk = 10.00f,
            float maxYJerk = 10.00f,
            float maxZJerk = 10.00f,
            float maxEJerk = 5.00f)
        {
            this.SendCommand(
                $"M205" +
                $" B{minSegmentTimeMicroseconds:N2}" +
                $" S{minFeedrate:N2}" +
                $" T{minTravelFeedrate:N2}" +
                $" X{maxXJerk:N2}" +
                $" Y{maxYJerk:N2}" +
                $" Z{maxZJerk:N2}" +
                $" E{maxEJerk:N2}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.settings.MinSegmentTimeMicroseconds = minSegmentTimeMicroseconds;
                this.settings.MinFeedrate = minFeedrate;
                this.settings.MinTravelFeedrate = minTravelFeedrate;
                this.settings.MaxXJerk = maxXJerk;
                this.settings.MaxYJerk = maxYJerk;
                this.settings.MaxZJerk = maxZJerk;
                this.settings.MaxEJerk = maxEJerk;
                return true;
            }

            return false;
        }

        public bool ResetHomePosition()
        {
            this.SendCommand("G92.1");
            bool resetSuccess = this.ProcessResponse(DexArmCommand.Ok, out _);
            if (!resetSuccess) return false;
            this.SendCommand("G0 X300 Y0 Z0");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public float GetModuleOffset()
        {
            this.SendCommand($"M503 S");
            this.ProcessResponse(DexArmCommand.GetModuleOffset, out object offset);
            return (float)offset;
        }

        public bool SetModule(DexArmModule module)
        {
            this.SendCommand($"M888 P{(int)module}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.Module = module;
                return true;
            }

            return false;
        }

        /// <summary>
        /// Load pen into pen module.
        /// Send commands:
        /// G0 X75 Y300 Z-80
        /// G0 X75 Y300 Z-86 ;make a dot on paper
        /// G0 X75 Y300 Z-80
        /// 
        /// G0 X-75 Y300 Z-80
        /// G0 X-75 Y300 Z-86 ;make a dot on paper
        /// G0 X-75 Y300 Z-80
        /// 
        /// Measure actual distance between two dots.
        /// Target distance is 150.
        /// </summary>
        /// <param name="targetDistance"></param>
        /// <param name="actualDistance"></param>
        /// <returns></returns>
        public bool CalibrateCustomModule(float targetDistance, float actualDistance)
        {
            this.SendCommand($"M888 P5 T{targetDistance} A{actualDistance}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.Module = DexArmModule.Custom;
                return true;
            }

            return false;
        }

        public DexArmModule GetModule()
        {
            this.SendCommand($"M888");
            this.ProcessResponse(DexArmCommand.Ok, out object module);
            return (DexArmModule)module;
        }

        public bool ResetToOriginPosition()
        {
            this.SendCommand("M1111");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetPositioningMode(DexArmPositioningMode mode)
        {
            this.SendCommand($"G9{(int)mode}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.PositioningMode = mode;
                return true;
            }

            return false;
        }

        public async Task<bool> SetPositioningModeAsync(DexArmPositioningMode mode)
        {
            if (this.PositioningMode == mode)
            {
                Trace.WriteLine($"PositioningMode already set to '{mode}'");
                return true;
            }

            this.SendCommand($"G9{(int)mode}");

            (bool success, _) = await this.ProcessResponseAsync(DexArmCommand.Ok);

            if (success)
            {
                this.PositioningMode = mode;
                return true;
            }

            return false;
        }

        public bool SetSpeed(uint mmPerMinute = 1000)
        {
            this.SendCommand($"G0 F{mmPerMinute}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.mmPerMinute = mmPerMinute;
                return true;
            }

            return false;
        }

        public async Task<bool> SetSpeedAsync(uint mmPerMinute = 1000)
        {
            if (mmPerMinute < 1 || mmPerMinute > 8000)
            {
                throw new ArgumentOutOfRangeException("Speed must be between 1 and 8000 millimeters per minute");
            }

            this.SendCommand($"G0 F{mmPerMinute}");

            (bool success, _) = await this.ProcessResponseAsync(DexArmCommand.Ok);

            if (success)
            {
                this.mmPerMinute = mmPerMinute;
                return true;
            }

            return false;
        }

        public bool SetXySlope(float x, float y)
        {
            if (x < -1 || x > 1 || y < -1 || y > 1)
            {
                throw new ArgumentOutOfRangeException($"Slope must be within [-1,1]. X:{x}, Y:{y}");
            }

            this.SendCommand($"M891 X{x} Y{y}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public Vector2 GetXySlope()
        {
            this.SendCommand("M892");
            if (this.ProcessResponse(DexArmCommand.GetXyAxisSlope, out object xyAxisSlope))
            {
                return (Vector2) xyAxisSlope;
            }

            return Vector2.Zero;
        }

        public bool Dwell(TimeSpan delayTime)
        {
            this.SendCommand($"G4 P{delayTime.TotalMilliseconds}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool ResetWorkingHeight()
        {
            this.SendCommand("G92.1");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetEncoderPosition(float x, float y, float z)
        {
            this.SendCommand($"M894 X{x} Y{y} Z{z}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.timeLastMoveCommandSent = DateTime.UtcNow;
                return true;
            }

            return false;
        }

        public string ReportSettings(bool verbose)
        {
            this.SendCommand(verbose ? "M503" : "M503 S");
            this.ProcessResponse(DexArmCommand.ReportSettings, out object settings);
            return (string)settings;
        }

        public void SoftReboot()
        {
            this.SendCommand("M2007");
            this.softRebootTimestamp = DateTime.UtcNow;
            this.rebooted = true;
        }

        public bool IsMoving(out Vector3 encoderPosition)
        {
            TimeSpan timeSinceLastMoveCommandSent = DateTime.UtcNow - this.timeLastMoveCommandSent;
            if (timeSinceLastMoveCommandSent < this.timeForEncoderToUpdateAfterMoveCommandSent)
            {
                encoderPosition = lastEncoderPosition;
                return true;
            }

            TimeSpan timeSinceLastIsMovingPoll = DateTime.UtcNow - this.lastIsMovingPollTime;
            if (timeSinceLastIsMovingPoll < this.isMovingPollPeriod)
            {
                Thread.Sleep(this.isMovingPollPeriod - timeSinceLastIsMovingPoll);
            }

            encoderPosition = this.GetEncoderPosition();
            this.lastIsMovingPollTime = DateTime.UtcNow;

            if (encoderPosition != Vector3.Zero && encoderPosition != lastEncoderPosition)
            {
                lastEncoderPosition = encoderPosition;
                return true;
            }

            return false;
        }

        public bool WaitForFinish()
        {
            this.SendCommand("M400");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public async Task<bool> WaitForFinishAsync(CancellationToken token = default)
        {
            this.SendCommand("M400");

            (bool success, _) = await this.ProcessResponseAsync(DexArmCommand.Ok, token);

            return success;
        }

        public void Dispose()
        {
            this.serialPort?.Dispose();
        }
    }
}