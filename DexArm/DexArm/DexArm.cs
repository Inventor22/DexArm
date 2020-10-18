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

        private string msgReceived = string.Empty;
        private DexArmPositioningMode positioningMode = DexArmPositioningMode.Absolute;
        private DateTime lastWaitReceieved;
        private uint mmPerMinute = 1000;

        private ConcurrentQueue<string> responses = new ConcurrentQueue<string>();

        private ConcurrentQueue<TaskCompletionSource<object>> requests = new ConcurrentQueue<TaskCompletionSource<object>>();
        private ConcurrentQueue<Func<string, object>> processFunc = new ConcurrentQueue<Func<string, object>>();

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

            //serialPort.DataReceived += this.ProcessDexArmResponse;
            //serialPort.DataReceived += this.PrintDexArmOutput;
        }

        public bool PrintResponse { get; set; } = true;

        private void PrintDexArmOutput(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort port = (SerialPort)sender;
            Console.WriteLine(port.ReadLine());
        }

        private void ProcessDexArmResponse(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort port = (SerialPort)sender;

            string response = port.ReadLine();

            if (requests.Count > 0)
            {
                processFunc.TryDequeue(out Func<string, object> func);
                object conversion = func(response);
                requests.TryDequeue(out TaskCompletionSource<object> task);
                task.SetResult(conversion);
            }

            if (response == "wait")
            {
                lastWaitReceieved = DateTime.Now;
               // Console.WriteLine(response);
            }
            else
            {
                this.responses.Enqueue(response);

                if (this.PrintResponse)
                {
                    Console.WriteLine(response);
                }
            }
        }

        private bool CommandSuccessful()
        {
            string response;
            int pollCount = 0;
            do
            {
                response = this.serialPort.ReadLine();
                Console.WriteLine($"CommandSuccessful poll {pollCount}: {response}");
            }
            while (pollCount++ < 5 && response.StartsWith("wait"));

            return response.StartsWith("ok");
        }

        private bool ProcessResponse(DexArmCommand command, out object output)
        {
            output = null;

            //bool cmdExecuted = this.CommandSuccessful(command);

            //if (cmdExecuted == false)
            //{
            //    return false;
            //}

            //string response = this.serialPort.ReadLine();
            //Console.WriteLine($"Response: {response}");

            List<string> responses = new List<string>();

            string response;
            int responseAttempts = 0;
            bool successfulResponse = false;
            do
            {
                response = this.serialPort.ReadLine();
                if (response.StartsWith("wait") == false)
                {
                    responses.Add(response);
                }
                if (response.StartsWith("ok"))
                {
                    successfulResponse = true;
                    break;
                }
            }
            while (responseAttempts++ < 5);

            Console.WriteLine($"Responses [{responses.Count}]");
            foreach (string log in responses)
            {
                Console.WriteLine($"  | {log}");
            }

            if (!successfulResponse)
            {
                return false;
            }

            int lastIndex = responses.Count - 1;

            switch (command)
            {
                case DexArmCommand.Ok:
                    return true;
                case DexArmCommand.GoHome:
                    return responses[0].StartsWith("M1112");
                case DexArmCommand.GetXyAxisSlope:
                    return false;
                case DexArmCommand.GetJointAngles:
                    return false;
                case DexArmCommand.IsMoving:
                    return false;
                case DexArmCommand.GetCurrentPosition:
                    Match match = Regex.Match(responses[0], @"X:(?<X>-?\d+\.\d+) Y:(?<Y>-?\d+\.\d+) Z:(?<Z>-?\d+\.\d+)");
                    if (match.Success == false)
                    {
                        return false;
                    }

                    GroupCollection groups = match.Groups;
                    try
                    {
                        output = new Vector3(
                           (int)Convert.ToDouble(groups["X"].Value),
                           (int)Convert.ToDouble(groups["Y"].Value),
                           (int)Convert.ToDouble(groups["Z"].Value));

                        return true;
                    } 
                    catch
                    {
                        return false;
                    }
                default:
                    return false;
            }
        }

        public bool Init()
        {
            this.serialPort.Open();
            return this.serialPort.IsOpen;
        }

        public bool IsWaiting()
        {
            return (DateTime.Now - lastWaitReceieved) < TimeSpan.FromSeconds(2);
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
            // return this.CommandSuccessful();
        }

        public Vector3 GetCurrentPosition()
        {
            this.serialPort.WriteLine("M114");

            if (this.ProcessResponse(DexArmCommand.GetCurrentPosition, out object position))
            {
                return (Vector3)position;
            }

            return Vector3.Zero;

            //TaskCompletionSource<object> a = new TaskCompletionSource<object>();
            //this.requests.Enqueue(a);
            //this.processFunc.Enqueue((rawResponse) =>
            //{
            //    var groups = Regex.Match(rawResponse, @"X:(?<X>-?\d+\.\d+) Y:(?<Y>-?\d+\.\d+) Z:(?<Z>-?\d+\.\d+)").Groups;
            //    return new Vector3(
            //        (int)Convert.ToDouble(groups["X"].Value),
            //        (int)Convert.ToDouble(groups["Y"].Value),
            //        (int)Convert.ToDouble(groups["Z"].Value));
            //});

            //return (Vector3) a.Task.Result;
        }

        public void SetAcceleration(int acceleration, int travelAcceleration, int retractAcceleration = 60)
        {
            this.serialPort.WriteLine($"M204 P{acceleration}T{travelAcceleration}T{retractAcceleration}");
        }

        public bool GoHome()
        {
            this.serialPort.WriteLine("M1112");
            return this.ProcessResponse(DexArmCommand.GoHome, out _);
        }

        public void SetHomeToCurrentPosition()
        {
            this.serialPort.WriteLine("G92 X0 Y0 Z0 E0");
        }

        public bool SetWorkOrigin()
        {
            this.SetHomeToCurrentPosition();
            return true;
        }

        public void ResetHomePosition()
        {
            this.serialPort.WriteLine("G92.1");
            this.serialPort.WriteLine("G0 X300 Y0 Z0");
        }

        public void SetOffset(DexArmModuleOffset offset)
        {
            this.serialPort.WriteLine($"M888 P{(int)offset}");
        }

        public void ResetToOriginPosition()
        {
            this.serialPort.WriteLine("M1111");
        }

        public bool SetPositioningMode(DexArmPositioningMode mode)
        {
            this.positioningMode = mode;
            this.serialPort.WriteLine($"G9{(int)mode}");
            return this.ProcessResponse(DexArmCommand.Ok, out _);
            //return this.CommandSuccessful();
        }

        public bool SetSpeed(uint mmPerMinute = 1000)
        {
            this.serialPort.WriteLine($"G0 F{mmPerMinute}");
            return true;
        }

        public bool SetWaitingTime(uint milliseconds)
        {
            this.serialPort.WriteLine($"P{milliseconds}");
            return true;
        }

        public void AquireCurrentXyAxisSlope()
        {
            this.serialPort.WriteLine("M892");
        }

        public async Task SoftReboot()
        {
            this.serialPort.WriteLine("M2007");

            await Task.Delay(500);

            this.Init();
        }
    }
}