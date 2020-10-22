// Copyright 2020 Dustin Dobransky

namespace Rotrics.DexArm
{
    public class DexArmSettings : IDexArmSettings
    {
        public float MinSegmentTimeMicroseconds { get; set; }
        public float MinFeedrate { get; set; }
        public float MinTravelFeedrate { get; set; }
        public float MaxXJerk { get; set; }
        public float MaxYJerk { get; set; }
        public float MaxZJerk { get; set; }
        public float MaxEJerk { get; set; }
    }
}
