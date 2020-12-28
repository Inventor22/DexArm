// Copyright 2020 Dustin Dobransky

using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Numerics;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace Rotrics.DexArm
{
    public class DexArm : IDexArm, IDisposable
    {
        private string portName;
        private SerialPort serialPort;
        private List<string> commandResponses = new List<string>();

        private uint mmPerMinute = 1000;

        private DateTime softRebootTimestamp = DateTime.MinValue;
        private TimeSpan powerUpTime = TimeSpan.FromMilliseconds(3500);
        private bool rebooted = false;

        private Vector3 lastEncoderPosition = Vector3.Zero;
        private DateTime lastIsMovingPollTime = DateTime.MinValue;
        private TimeSpan isMovingPollPeriod = TimeSpan.FromMilliseconds(50);

        private DateTime timeLastMoveCommandSent = DateTime.MinValue;
        private TimeSpan timeForEncoderToUpdateAfterMoveCommandSent = TimeSpan.FromMilliseconds(160);

        private StringBuilder positionBuilder = new StringBuilder();

        public bool PrintCommand { get; set; } = true;
        public bool PrintResponse { get; set; } = true;

        public DexArmPositioningMode PositioningMode { get; private set; } = DexArmPositioningMode.Absolute;

        public DexArmMoveMode DefaultMoveMode { get; set; } = DexArmMoveMode.FastMode;

        private uint defaultVelocity = 1000; // mm per minute
        public uint DefaultVelocity
        {
            get => defaultVelocity;
            set
            {
                if (value > 8000)
                {
                    throw new ArgumentOutOfRangeException($"Cannot set velocity greater than 8000mm/minute. Assigned Velocity: {value}");
                }

                this.defaultVelocity = value;
            }
        }

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

        public DexArm(string portName)
        {
            this.portName = portName;
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
                //serialPort.DataReceived += this.PrintDexArmOutput;
            }

            // Handle soft reboot case
            if (rebooted)
            {
                TimeSpan timeSinceSoftReboot = DateTime.UtcNow - this.softRebootTimestamp;
                Console.WriteLine("Time since reboot: " + timeSinceSoftReboot.TotalMilliseconds);
                if (timeSinceSoftReboot < powerUpTime)
                {
                    Console.WriteLine("Sleeping for : " + (powerUpTime - timeSinceSoftReboot));
                    Thread.Sleep(powerUpTime - timeSinceSoftReboot);
                }
                rebooted = false;
            }

            // Initialize DexArm
            if (!this.serialPort.IsOpen)
            {
                this.serialPort.Open();

                // Hacky workaround for a bug where upon first powerup or reboot,
                // DexArm responds with "??????" when the first command is sent.
                if (this.serialPort.IsOpen)
                {
                    bool printResponse = this.PrintResponse;
                    this.PrintResponse = false;
                    // Only looks for "ok" response, any other garbage is discarded
                    this.Dwell(TimeSpan.FromMilliseconds(1));
                    // Get intial arm position. Necessary for IsMoving command.
                    this.lastEncoderPosition = this.GetEncoderPosition();
                    this.PrintResponse = printResponse;

                }
                else
                {
                    throw new IOException($"Could not connect to DexArm on Port '{this.serialPort.PortName}'");
                }
            }

            return this.serialPort.IsOpen;
        }

        private void PrintDexArmOutput(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort port = (SerialPort)sender;
            Console.WriteLine(port.ReadLine());
        }

        private bool ProcessResponse(DexArmCommand command, out object output)
        {
            output = null;
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
            while (responseAttempts++ < 50);

            if (this.PrintResponse)
            {
                Console.WriteLine($"Responses [{this.commandResponses.Count}]");
                foreach (string log in this.commandResponses)
                {
                    Console.WriteLine($"  | {log}");
                }
            }

            if (!successfulResponse)
            {
                return false;
            }

            switch (command)
            {
                case DexArmCommand.Ok:
                    return true;
                case DexArmCommand.GoHome:
                    return this.commandResponses[0].StartsWith("M1112");
                case DexArmCommand.GetXyAxisSlope:
                    {
                        Match xAxisSlopeMatch = Regex.Match(this.commandResponses[this.commandResponses.Count - 3], @"GET X AXIS SLOPE IS (?<xSlope>-?\d+\.\d+)");
                        Match yAxisSlopeMatch = Regex.Match(this.commandResponses[this.commandResponses.Count - 2], @"GET Y AXIS SLOPE IS (?<ySlope>-?\d+\.\d+)");

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
                        Match jointAnglesMatch = Regex.Match(this.commandResponses[2], @"DEXARM Theta A:(?<A>-?\d+\.\d+)  Theta B:(?<B>-?\d+\.\d+)  Theta C:(?<C>-?\d+\.\d+)");
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
                        Match encoderPositionMatch = Regex.Match(this.commandResponses[0], @"M894 X(?<X>-?\d+\.?\d+) Y(?<Y>-?\d+\.?\d+) Z(?<Z>-?\d+\.?\d+)");
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
                        Match positionMatch = Regex.Match(this.commandResponses[0], @"X:(?<X>-?\d+\.\d+) Y:(?<Y>-?\d+\.\d+) Z:(?<Z>-?\d+\.\d+)");
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
                        Match axisAccelMatch = Regex.Match(this.commandResponses[10], @"M201 X(?<X>-?\d+\.\d+) Y(?<Y>-?\d+\.\d+) Z(?<Z>-?\d+\.\d+) E(?<E>-?\d+\.\d+)");
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
                        Match accel3DMatch = Regex.Match(this.commandResponses[12], @"M204 P(?<P>-?\d+\.\d+) R(?<R>-?\d+\.\d+) T(?<T>-?\d+\.\d+)");
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
                        string response = commandResponses[0];
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
                        output = string.Join(Environment.NewLine, this.commandResponses);
                        return true;
                    }
                default:
                    return false;
            }
        }

        public void SendCommand(string command)
        {
            if (this.PrintCommand) Console.WriteLine(command);
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
            this.serialPort.WriteLine("M114");

            if (this.ProcessResponse(DexArmCommand.GetCurrentPosition, out object position))
            {
                return (Vector3)position;
            }

            return Vector3.Zero;
        }

        public Vector3 GetEncoderPosition()
        {
            this.serialPort.WriteLine("M893");

            if (this.ProcessResponse(DexArmCommand.GetEncoderPosition, out object encoderPosition))
            {
                return (Vector3)encoderPosition;
            }

            return Vector3.Zero;
        }

        public Vector3 GetJointAngles()
        {
            this.serialPort.WriteLine("M114");

            if (this.ProcessResponse(DexArmCommand.GetJointAngles, out object jointAngles))
            {
                return (Vector3)jointAngles;
            }

            return Vector3.Zero;
        }

        public bool Set3DPrintingAcceleration(int acceleration, int travelAcceleration, int retractAcceleration)
        {
            this.serialPort.WriteLine($"M204 P{acceleration} T{travelAcceleration} R{retractAcceleration}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetAxisAcceleration(int x, int y, int z, int e)
        {
            this.serialPort.WriteLine($"M201 E{e} X{x} Y{y} Z{z}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public Vector4 GetAxisAcceleration()
        {
            this.serialPort.WriteLine("M503");
            
            if (this.ProcessResponse(DexArmCommand.GetAxisAcceleration, out object axisAcceleration))
            {
                return (Vector4)axisAcceleration;
            }

            return Vector4.Zero;
        }

        public Vector3 Get3DPrintingAcceleration()
        {
            this.serialPort.WriteLine("M503");

            if (this.ProcessResponse(DexArmCommand.Get3DPrintingAcceleration, out object printingAcceleration))
            {
                return (Vector3)printingAcceleration;
            }

            return Vector3.Zero;
        }

        public bool GoHome()
        {
            this.serialPort.WriteLine("M1112");
            return this.ProcessResponse(DexArmCommand.GoHome, out _);
        }

        public bool SetWorkOrigin()
        {
            this.serialPort.WriteLine("G92 X0 Y0 Z0 E0");
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
            this.serialPort.WriteLine(
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
            this.serialPort.WriteLine("G92.1");
            bool resetSuccess = this.ProcessResponse(DexArmCommand.Ok, out _);
            if (!resetSuccess) return false;
            this.serialPort.WriteLine("G0 X300 Y0 Z0");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public float GetModuleOffset()
        {
            this.serialPort.WriteLine($"M503 S");
            this.ProcessResponse(DexArmCommand.GetModuleOffset, out object offset);
            return (float)offset;
        }

        public bool SetModule(DexArmModule module)
        {
            this.serialPort.WriteLine($"M888 P{(int)module}");

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
            this.serialPort.WriteLine($"M888 P5 T{targetDistance} A{actualDistance}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.Module = DexArmModule.Custom;
                return true;
            }

            return false;
        }

        public DexArmModule GetModule()
        {
            this.serialPort.WriteLine($"M888");
            this.ProcessResponse(DexArmCommand.Ok, out object module);
            return (DexArmModule)module;
        }

        public bool ResetToOriginPosition()
        {
            this.serialPort.WriteLine("M1111");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetPositioningMode(DexArmPositioningMode mode)
        {
            this.serialPort.WriteLine($"G9{(int)mode}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
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
        
        public bool SetXySlope(float x, float y)
        {
            if (x < -1 || x > 1 || y < -1 || y > 1)
            {
                throw new ArgumentOutOfRangeException($"Slope must be within [-1,1]. X:{x}, Y:{y}");
            }

            this.serialPort.WriteLine($"M891 X{x} Y{y}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public Vector2 GetXySlope()
        {
            this.serialPort.WriteLine("M892");
            if (this.ProcessResponse(DexArmCommand.GetXyAxisSlope, out object xyAxisSlope))
            {
                return (Vector2) xyAxisSlope;
            }

            return Vector2.Zero;
        }

        public bool Dwell(TimeSpan delayTime)
        {
            this.serialPort.WriteLine($"G4 P{delayTime.TotalMilliseconds}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool ResetWorkingHeight()
        {
            this.serialPort.WriteLine("G92.1");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetEncoderPosition(float x, float y, float z)
        {
            this.serialPort.WriteLine($"M894 X{x} Y{y} Z{z}");

            if (this.ProcessResponse(DexArmCommand.Ok, out _))
            {
                this.timeLastMoveCommandSent = DateTime.UtcNow;
                return true;
            }

            return false;
        }

        public string ReportSettings(bool verbose)
        {
            this.serialPort.WriteLine(verbose ? "M503" : "M503 S");
            this.ProcessResponse(DexArmCommand.ReportSettings, out object settings);
            return (string)settings;
        }

        public void SoftReboot()
        {
            this.serialPort.WriteLine("M2007");
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
            this.serialPort.WriteLine("M400");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public Task<bool> WaitForFinishAsync(CancellationToken token = default)
        {
            this.serialPort.WriteLine("M400");

            return Task.Run(() =>
            {
                while (true)
                {
                    if (token.IsCancellationRequested)
                    {
                        return false;
                    }

                    string response = this.serialPort.ReadLine();
                    Console.WriteLine($"{nameof(DexArm.WaitForFinishAsync)}: {response}");

                    if (response.StartsWith("ok"))
                    {
                        return true;
                    }
                }
            }, 
            token);
        }

        public void Dispose()
        {
            this.serialPort?.Dispose();
        }
    }
}