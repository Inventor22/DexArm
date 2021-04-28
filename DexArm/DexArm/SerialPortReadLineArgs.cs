using System;

namespace Rotrics.DexArm
{
    public class SerialPortReadLineArgs : EventArgs
    {
        public string Message { get; private set; }

        public SerialPortReadLineArgs(string message)
        {
            this.Message = message;
        }
    }
}
