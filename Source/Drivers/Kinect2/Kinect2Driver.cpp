#include "Kinect2Driver.h"
#include "Kinect2Device.h"
#include <Kinect.h>

using namespace oni::driver;
using namespace kinect2_device;

static const char VENDOR_VAL[] = "Microsoft";

// adding more characters after "kinect" does not work with nite: please refer to here
// https://github.com/OpenKinect/libfreenect2/issues/243#issuecomment-107170614
static const char NAME_VAL[] = "Kinect"; 

Kinect2Driver::Kinect2Driver(OniDriverServices* pDriverServices)
  : DriverBase(pDriverServices)
{}
	
Kinect2Driver::~Kinect2Driver()
{}

OniStatus Kinect2Driver::initialize(DeviceConnectedCallback connectedCallback,
                                    DeviceDisconnectedCallback disconnectedCallback,
                                    DeviceStateChangedCallback deviceStateChangedCallback,
                                    void* pCookie)
{
	HRESULT hr;
	DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie);

  // Get sensor instance
  IKinectSensor* pKinectSensor = NULL;
  hr = ::GetDefaultKinectSensor(&pKinectSensor);
  if (FAILED(hr)) {
    if (pKinectSensor) {
      pKinectSensor->Release();
    }
    return ONI_STATUS_NO_DEVICE;
  }

  hr = pKinectSensor->Open();
  if (FAILED(hr)) {
    pKinectSensor->Release();
    return ONI_STATUS_ERROR;
  }

  // Wait some time to let the sensor initialize
  BOOLEAN available = FALSE;
  for (size_t i = 0; i < (60000 / 500); ++i)
  {
	  hr = pKinectSensor->get_IsAvailable(&available);
	  if (SUCCEEDED(hr) && available)
		  break;
	  Sleep(500);
  }

  if (!available)
  {
	  pKinectSensor->Close();
	  pKinectSensor->Release();
	  return ONI_STATUS_NO_DEVICE;
  }

  // Get sensor info
  OniDeviceInfo* pInfo = XN_NEW(OniDeviceInfo);
  WCHAR sensorId[ONI_MAX_STR];
  pKinectSensor->get_UniqueKinectId(ONI_MAX_STR, sensorId);
	size_t convertedChars = 0;
	const size_t newsize = ONI_MAX_STR;
	size_t origsize = wcslen(sensorId) + 1;
	wcstombs_s(&convertedChars, pInfo->uri, origsize, sensorId, _TRUNCATE);
	xnOSStrCopy(pInfo->vendor, VENDOR_VAL, ONI_MAX_STR);
	xnOSStrCopy(pInfo->name, NAME_VAL, ONI_MAX_STR);
	m_devices[pInfo] = NULL;
	deviceConnected(pInfo);
	deviceStateChanged(pInfo, S_OK); // Sensor is ready
  
  // Close sensor instance
  pKinectSensor->Close();
  pKinectSensor->Release();

  return ONI_STATUS_OK;
}

DeviceBase* Kinect2Driver::deviceOpen(const char* uri, const char* /*mode*/)
{
	for (xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*>::Iterator iter = m_devices.Begin(); iter != m_devices.End(); ++iter)
	{
		if (xnOSStrCmp(iter->Key()->uri, uri) == 0)
		{
			// Found
			if (iter->Value() != NULL)
			{
				// already using
				return iter->Value();
			}
			else
			{
        // Get sensor instance
        IKinectSensor* pKinectSensor = NULL;
				HRESULT hr;
        hr = ::GetDefaultKinectSensor(&pKinectSensor);
        if (FAILED(hr)) {
          if (pKinectSensor) {
            pKinectSensor->Release();
          }
          return NULL;
        }

        // Compare sensor id. TODO: To be removed when multi-device support is added
        char sensorUri[ONI_MAX_STR];
        WCHAR sensorId[ONI_MAX_STR];
        pKinectSensor->get_UniqueKinectId(ONI_MAX_STR, sensorId);
	      size_t convertedChars = 0;
	      const size_t newsize = ONI_MAX_STR;
	      size_t origsize = wcslen(sensorId) + 1;
	      wcstombs_s(&convertedChars, sensorUri, origsize, sensorId, _TRUNCATE);
        if (xnOSStrCmp(iter->Key()->uri, sensorUri) != 0) {
          pKinectSensor->Release();
          return NULL;
        }

        // Initialize the Kinect2
        hr = pKinectSensor->Open();
        Kinect2Device* pDevice = XN_NEW(Kinect2Device, pKinectSensor);
        if (!pDevice) {
          pKinectSensor->Close();
          pKinectSensor->Release();
          return NULL;
        }
				iter->Value() = pDevice;
				return pDevice;
			}
		}
	}
	return NULL;	
}

void kinect2_device::Kinect2Driver::deviceClose(oni::driver::DeviceBase* pDevice)
{
	for (xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*>::Iterator iter = m_devices.Begin(); iter != m_devices.End(); ++iter)
	{
		if (iter->Value() == pDevice)
		{
			iter->Value() = NULL;
			XN_DELETE(pDevice);
			return;
		}
	}

	// not our device?!
	XN_ASSERT(FALSE);
}

void Kinect2Driver::shutdown()
{}

OniStatus Kinect2Driver::tryDevice(const char* uri)
{
	return ONI_STATUS_OK;	
}

void* Kinect2Driver::enableFrameSync(StreamBase** pStreams, int streamCount)
{
	return NULL;
}

void Kinect2Driver::disableFrameSync(void* frameSyncGroup)
{}

ONI_EXPORT_DRIVER(kinect2_device::Kinect2Driver)
