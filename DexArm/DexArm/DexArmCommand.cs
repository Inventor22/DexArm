
namespace Rotrics.DexArm
{
    public enum DexArmCommand
    {
        Ok = 0,
        GoHome,
        GetCurrentPosition,
        GetXyAxisSlope,
        GetJointAngles,
        GetEncoderPosition,
        GetAxisAcceleration,
        Get3DPrintingAcceleration,
        IsMoving,
        ReportSettings
    }
}
