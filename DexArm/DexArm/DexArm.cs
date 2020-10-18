// Copyright 2020 Dustin Dobransky

using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO.Ports;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace Rotrics.DexArm
{
    public class DexArm : IDexArm
    {
        private SerialPort serialPort;
        private List<string> commandResponses = new List<string>();

        private uint mmPerMinute = 1000;

        public DexArm(string portName)
        {
            serialPort = new SerialPort(portName);

            serialPort.BaudRate = 115200;
            serialPort.DataBits = 8;
            serialPort.StopBits = StopBits.One;
            serialPort.Parity = Parity.None;
            serialPort.Handshake = Handshake.RequestToSend;
            serialPort.DtrEnable = true;
            serialPort.NewLine = "\n";

            //serialPort.DataReceived += this.PrintDexArmOutput;
        }

        private void PrintDexArmOutput(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort port = (SerialPort)sender;
            Console.WriteLine(port.ReadLine());
        }

        public bool PrintResponse { get; set; } = true;

        public DexArmPositioningMode PositioningMode { get; private set; } = DexArmPositioningMode.Absolute;

        public bool Init()
        {
            this.serialPort.Open();
            return this.serialPort.IsOpen;
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
                case DexArmCommand.ReportSettings:
                    {
                        output = string.Join(Environment.NewLine, this.commandResponses);
                        return true;
                    }
                default:
                    return false;
            }
        }

        public bool SetPosition(int x, int y, int z, uint mmPerMinute = 1000, DexArmMoveMode moveMode = DexArmMoveMode.FastMode)
        {
            if (this.mmPerMinute == mmPerMinute)
            {
                this.serialPort.WriteLine($"G{(int)moveMode} X{x} Y{y} Z{z}");
            }
            else
            {
                this.mmPerMinute = mmPerMinute;
                this.serialPort.WriteLine($"G{(int)moveMode} F{mmPerMinute} X{x} Y{y} Z{z}");
            }

            return this.ProcessResponse(DexArmCommand.Ok, out _);
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

        public bool Set3DPrintingAcceleration(int acceleration, int travelAcceleration, int retractAcceleration = 60)
        {
            this.serialPort.WriteLine($"M204 P{acceleration}T{travelAcceleration}T{retractAcceleration}");
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

        public void ResetHomePosition()
        {
            this.serialPort.WriteLine("G92.1");
            this.ProcessResponse(DexArmCommand.Ok, out _);
            this.serialPort.WriteLine("G0 X300 Y0 Z0");
            this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetOffset(DexArmModuleOffset offset)
        {
            this.serialPort.WriteLine($"M888 P{(int)offset}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool ResetToOriginPosition()
        {
            this.serialPort.WriteLine("M1111");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetPositioningMode(DexArmPositioningMode mode)
        {
            this.PositioningMode = mode;
            this.serialPort.WriteLine($"G9{(int)mode}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public bool SetSpeed(uint mmPerMinute = 1000)
        {
            this.serialPort.WriteLine($"G0 F{mmPerMinute}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public Vector2 GetCurrentXyAxisSlope()
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
            return this.ProcessResponse(DexArmCommand.Ok, out _);
        }

        public string ReportSettings(bool verbose)
        {
            this.serialPort.WriteLine(verbose ? "M503" : "M503 S");
            this.ProcessResponse(DexArmCommand.Ok, out object settings);
            return (string)settings;
        }

        public async Task SoftReboot()
        {
            this.serialPort.WriteLine("M2007");
            await Task.Delay(1000);
            this.Init();
        }
    }
}