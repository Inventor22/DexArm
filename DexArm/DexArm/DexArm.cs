﻿// Copyright 2020 Dustin Dobransky

using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO.Ports;
using System.Numerics;
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

            serialPort.DataReceived += this.ProcessDexArmResponse;
        }

        public bool PrintResponse { get; set; } = true;

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

        public bool Init()
        {
            this.serialPort.Open();
            bool isOpen = this.serialPort.IsOpen;

            Console.WriteLine($"DexArm Initialized: {isOpen}");

            return isOpen;
        }

        public bool IsWaiting()
        {
            return (DateTime.Now - lastWaitReceieved) < TimeSpan.FromSeconds(2);
        }

        public void SetPosition(int x, int y, int z, uint mmPerMinute = 1000, DexArmMoveMode moveMode = DexArmMoveMode.FastMode)
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
        }

        public Vector3 GetCurrentPosition()
        {
            this.serialPort.WriteLine("M114");

            TaskCompletionSource<object> a = new TaskCompletionSource<object>();
            this.requests.Enqueue(a);
            this.processFunc.Enqueue((rawResponse) =>
            {
                var groups = Regex.Match(rawResponse, @"X:(?<X>-?\d+\.\d+) Y:(?<Y>-?\d+\.\d+) Z:(?<Z>-?\d+\.\d+)").Groups;
                return new Vector3(
                    (int)Convert.ToDouble(groups["X"].Value),
                    (int)Convert.ToDouble(groups["Y"].Value),
                    (int)Convert.ToDouble(groups["Z"].Value));
            });

            return (Vector3) a.Task.Result;
        }

        public void SetAcceleration(int acceleration, int travelAcceleration, int retractAcceleration = 60)
        {
            this.serialPort.WriteLine($"M204 P{acceleration}T{travelAcceleration}T{retractAcceleration}");
        }

        public void GoHome()
        {
            this.serialPort.WriteLine("M1112");
        }

        public void SetHomeToCurrentPosition()
        {
            this.serialPort.WriteLine("G92 X0 Y0 Z0 E0");
        }

        public void SetWorkOrigin()
        {
            this.SetHomeToCurrentPosition();
        }

        public void ResetHomePosition()
        {
            this.serialPort.WriteLine("G92.1");
            this.serialPort.WriteLine("G0 X300 Y0 Z0");
        }

        public void SetOffset(ModuleOffset offset)
        {
            this.serialPort.WriteLine($"M888 P{(int)offset}");
        }

        public void ResetToOriginPosition()
        {
            this.serialPort.WriteLine("M1111");
        }

        public void SetPositioningMode(DexArmPositioningMode mode)
        {
            this.positioningMode = mode;
            this.serialPort.WriteLine($"G9{(int)mode}");
        }

        public void SetSpeed(uint mmPerMinute = 1000)
        {
            this.serialPort.WriteLine($"G0 F{mmPerMinute}");
        }

        public void SetWaitingTime(uint milliseconds)
        {
            this.serialPort.WriteLine($"P{milliseconds}");
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
