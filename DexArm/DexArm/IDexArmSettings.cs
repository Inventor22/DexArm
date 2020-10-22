// Copyright 2020 Dustin Dobransky

namespace Rotrics.DexArm
{
    public interface IDexArmSettings
    {
        float MinSegmentTimeMicroseconds { get; }
        float MinFeedrate { get; }
        float MinTravelFeedrate { get; }
        float MaxXJerk { get; }
        float MaxYJerk { get; }
        float MaxZJerk { get; }
        float MaxEJerk { get; }
    }
}
