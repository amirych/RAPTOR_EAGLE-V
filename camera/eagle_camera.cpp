#include "eagle_camera.h"

EagleCamera::EagleCamera():
    currentPreAmpGain(EagleCamera::HighGain), currentReadoutRate(EagleCamera::SlowRate),
    currentShutterState(EagleCamera::ShutterExp), currentReadoutMode(EagleCamera::NormalReadoutMode)
{

}
