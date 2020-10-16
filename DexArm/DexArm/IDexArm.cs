// Copyright 2020 Dustin Dobransky

using System.Numerics;
using System.Threading.Tasks;

namespace Rotrics.DexArm
{
    public interface IDexArm
    {
        void AquireCurrentXyAxisSlope();
        Vector3 GetCurrentPosition();
        void GoHome();
        bool Init();
        bool IsWaiting();
        void ResetHomePosition();
        void ResetToOriginPosition();
        void SetAcceleration(int acceleration, int travelAcceleration, int retractAcceleration = 60);
        void SetHomeToCurrentPosition();
        void SetOffset(ModuleOffset offset);
        void SetPosition(int x, int y, int z, uint mmPerMinute = 1000, DexArmMoveMode moveMode = DexArmMoveMode.FastMode);
        void SetPositioningMode(DexArmPositioningMode mode);
        void SetSpeed(uint mmPerMinute = 1000);
        void SetWaitingTime(uint milliseconds);
        void SetWorkOrigin();
        Task SoftReboot();
    }
}