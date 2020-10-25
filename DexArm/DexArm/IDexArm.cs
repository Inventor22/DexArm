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
        Vector2 GetXySlope();
        bool SetXySlope(float xSlope, float ySlope);
        bool SetPositioningMode(DexArmPositioningMode mode);
        bool SetPosition(int x, int y, int z, uint mmPerMinute = 1000, DexArmMoveMode moveMode = DexArmMoveMode.FastMode);
        bool Set3DPrintingAcceleration(int printAcceleration = 60, int travelAcceleration = 40, int retractAcceleration = 60);
        bool SetAxisAcceleration(int x = 3000, int y = 3000, int z = 3000, int e = 10000);
        bool SetModule(DexArmModule offset);
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
        bool SoftReboot();
        bool IsMoving(out Vector3 encoderPosition);
    }
}