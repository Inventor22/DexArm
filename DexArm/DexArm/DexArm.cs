// Copyright 2020 Dustin Dobransky

using System;
using System.IO.Ports;
using System.Threading.Tasks;

namespace Rotrics.DexArm
{
    public class DexArm
    {
        private SerialPort serialPort;

        private string msgReceived = string.Empty;
        private DexArmPositioningMode positioningMode = DexArmPositioningMode.Absolute;
        private DateTime lastWaitReceieved;
        private uint mmPerMinute = 1000;

        public DexArm(string portName)
        {
            serialPort = new SerialPort(portName);

            serialPort.BaudRate = 115200;
            serialPort.DataBits = 8;
            serialPort.StopBits = StopBits.One;
            serialPort.Parity = Parity.None;
            serialPort.Handshake = Handshake.XOnXOff;
            serialPort.DtrEnable = true;
            serialPort.NewLine = "\n";

            serialPort.DataReceived += this.ProcessDexArmResponse;
        }

        private void ProcessDexArmResponse(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort port = (SerialPort)sender;

            msgReceived = port.ReadLine();

            if (msgReceived == "wait")
            {
                lastWaitReceieved = DateTime.Now;
            }
            else
            {
                Console.WriteLine(msgReceived);
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

        public void SetPositionOld(int x, int y, int z, DexArmMoveMode moveMode = DexArmMoveMode.FastMode)
        {
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

        public void GetCurrentPosition()
        {
            this.serialPort.WriteLine("M114");
        }

        public void SetAcceleration(int acceleration, int travelAcceleration, int retractAcceleration=60)
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
