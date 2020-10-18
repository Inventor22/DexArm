// Copyright 2020 Dustin Dobransky

using System;
using System.Numerics;
using System.Threading.Tasks;

namespace Rotrics.DexArm
{
    public interface IDexArm
    {
        bool Init();
        bool GoHome();
        Vector3 GetCurrentPosition();
        Vector4 GetAxisAcceleration();
        Vector3 Get3DPrintingAcceleration();
        Vector3 GetEncoderPosition();
        Vector2 GetCurrentXyAxisSlope();
        void ResetHomePosition();
        bool ResetToOriginPosition();
        bool ResetWorkingHeight();
        bool SetPositioningMode(DexArmPositioningMode mode);
        bool SetPosition(int x, int y, int z, uint mmPerMinute = 1000, DexArmMoveMode moveMode = DexArmMoveMode.FastMode);
        bool Set3DPrintingAcceleration(int printAcceleration = 60, int travelAcceleration = 40, int retractAcceleration = 60);
        bool SetAxisAcceleration(int x, int y, int z, int e);
        bool SetOffset(DexArmModuleOffset offset);
        bool SetSpeed(uint mmPerMinute = 1000);
        bool SetEncoderPosition(float x, float y, float z);
        bool SetWorkOrigin();
        bool Dwell(TimeSpan delayTime);
        string ReportSettings(bool verbose);
        Task SoftReboot();
    }
}