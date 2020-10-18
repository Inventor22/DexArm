// Copyright 2020 Dustin Dobransky

using System.Numerics;
using System.Threading.Tasks;

namespace Rotrics.DexArm
{
    public interface IDexArm
    {
        void AquireCurrentXyAxisSlope();
        Vector3 GetCurrentPosition();
        bool GoHome();
        bool Init();
        bool IsWaiting();
        void ResetHomePosition();
        void ResetToOriginPosition();
        void SetAcceleration(int acceleration, int travelAcceleration, int retractAcceleration = 60);
        void SetHomeToCurrentPosition();
        void SetOffset(DexArmModuleOffset offset);
        bool SetPosition(int x, int y, int z, uint mmPerMinute = 1000, DexArmMoveMode moveMode = DexArmMoveMode.FastMode);
        bool SetPositioningMode(DexArmPositioningMode mode);
        bool SetSpeed(uint mmPerMinute = 1000);
        bool SetWaitingTime(uint milliseconds);
        bool SetWorkOrigin();
        Task SoftReboot();
    }
}