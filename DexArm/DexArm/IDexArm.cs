// Copyright 2020 Dustin Dobransky

using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
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
        Vector2 GetXySlope();
        bool SetXySlope(float xSlope, float ySlope);
        bool SetPositioningMode(DexArmPositioningMode mode);

        bool SetPosition(Vector3 position);
        bool SetPosition(float x, float y, float z); // Uses DefaultVelocity and DefaultMoveMode
        bool SetPosition(Vector3 position, DexArmMoveMode moveMode);
        bool SetPosition(float x, float y, float z, DexArmMoveMode moveMode);
        bool SetPosition(Vector3 position, uint mmPerMinute);
        bool SetPosition(float x, float y, float z, uint mmPerMinute);
        bool SetPosition(float x, float y, float z, uint mmPerMinute, DexArmMoveMode moveMode);
        bool SetPosition(Vector3 position, uint mmPerMinute, DexArmMoveMode moveMode);

        bool Set3DPrintingAcceleration(int printAcceleration = 60, int travelAcceleration = 40, int retractAcceleration = 60);
        bool SetAxisAcceleration(int x = 3000, int y = 3000, int z = 3000, int e = 10000);
        bool SetModule(DexArmModule offset);
        bool CalibrateCustomModule(float targetDistance, float actualDistance);
        DexArmModule GetModule();
        float GetModuleOffset();
        bool SetSpeed(uint mmPerMinute = 1000);
        bool SetEncoderPosition(float x, float y, float z);
        bool SetAdvancedParams(
            float minSegmentTimeMicroseconds = 20000.00f,
            float minFeedrate = 0.00f,
            float minTravelFeedrate = 0.00f, 
            float maxXJerk = 10.00f, 
            float maxYJerk = 10.00f, 
            float maxZJerk = 10.00f,
            float maxEJerk = 5.00f);
        bool SetWorkOrigin();
        bool Dwell(TimeSpan delayTime);
        string ReportSettings(bool verbose);
        bool ResetHomePosition();
        bool ResetToOriginPosition();
        bool ResetWorkingHeight();
        void SoftReboot();
        bool WaitForFinish();
        Task<bool> WaitForFinishAsync(CancellationToken token = default);
        bool IsMoving(out Vector3 encoderPosition);
        void SendCommand(string command, [CallerMemberName] string memberName = "");
    }
}