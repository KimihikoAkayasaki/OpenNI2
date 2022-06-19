#include "Kinect2Driver.h"

#include <iostream>

#include "Kinect2Device.h"
#include <Kinect.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

constexpr const char* Kinect2StatusSHMAddress = "KinectONEStatusSHM";
boost::interprocess::mapped_region Kinect2StatusSHMRegion;

using namespace oni::driver;
using namespace kinect2_device;

// adding more characters after "kinect" does not work with nite: please refer to here
// https://github.com/OpenKinect/libfreenect2/issues/243#issuecomment-107170614
static const char NAME_VAL[] = "Kinect";
static const char VENDOR_VAL[] = "Microsoft";

bool Kinect2Driver::getIsInitializationAllowed()
{
	__try
	{
		return [&, this]
		{
			// Open already created shared memory object.
			boost::interprocess::shared_memory_object shm(
				boost::interprocess::open_only,
				"KinectAllowedVersionSHM",
				boost::interprocess::read_only);

			// Map the whole shared memory in this process
			boost::interprocess::mapped_region region(
				shm, boost::interprocess::read_only);

			// Check that memory was initialized to 1
			auto mem = static_cast<char*>(region.get_address());
			return static_cast<int>(*mem) == 2; // V2
		}();
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		std::cerr << "Couldn't read from boost/shared memory at name '" <<
			"KinectAllowedVersionSHM" << "', an exception occurred\n";
		return true; // Just in case
	}
}

void Kinect2Driver::updateKinect2StatusSHM(HRESULT _status)
{
	__try
	{
		[&, this]
		{
			// Write all the memory
			std::memset(Kinect2StatusSHMRegion.get_address(),
			            static_cast<char>(_status), Kinect2StatusSHMRegion.get_size());
		}();
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		std::cerr << "Couldn't write to boost/shared memory at name '" <<
			Kinect2StatusSHMAddress << "', an exception occurred\n";
	}
}

Kinect2Driver::Kinect2Driver(OniDriverServices* pDriverServices)
	: DriverBase(pDriverServices)
{
	[&, this]
	{
		__try
		{
			[&, this]
			{
				// Remove shared memory on construction and destruction
				static struct shm_remove
				{
					shm_remove() { boost::interprocess::shared_memory_object::remove(Kinect2StatusSHMAddress); }
					~shm_remove() { boost::interprocess::shared_memory_object::remove(Kinect2StatusSHMAddress); }
				} remover;

				// Create a shared memory object.
				boost::interprocess::shared_memory_object shm(
					boost::interprocess::create_only,
					Kinect2StatusSHMAddress,
					boost::interprocess::read_write);

				// Set size
				shm.truncate(1000);

				// Map the whole shared memory in this process
				Kinect2StatusSHMRegion =
					boost::interprocess::mapped_region(shm, boost::interprocess::read_write);
			}();
		}
		__except (EXCEPTION_EXECUTE_HANDLER)
		{
			std::cerr << "Couldn't create boost/shared memory at name '" <<
				Kinect2StatusSHMAddress << "', an exception occurred\n";
		}
	}();
}

Kinect2Driver::~Kinect2Driver()
{
}

OniStatus Kinect2Driver::initialize(DeviceConnectedCallback connectedCallback,
                                    DeviceDisconnectedCallback disconnectedCallback,
                                    DeviceStateChangedCallback deviceStateChangedCallback,
                                    void* pCookie)
{
	HRESULT hr;
	DriverBase::initialize(
		connectedCallback,
		disconnectedCallback,
		deviceStateChangedCallback,
		pCookie);

	// Get sensor instance
	IKinectSensor* pKinectSensor = nullptr;
	hr = GetDefaultKinectSensor(&pKinectSensor);
	if (!getIsInitializationAllowed() || FAILED(hr))
	{
		if (pKinectSensor)
		{
			pKinectSensor->Release();
		}
		updateKinect2StatusSHM(S_FALSE);
		return ONI_STATUS_NO_DEVICE;
	}

	hr = pKinectSensor->Open();
	if (FAILED(hr))
	{
		pKinectSensor->Release();
		updateKinect2StatusSHM(S_FALSE);
		return ONI_STATUS_ERROR;
	}

	// Wait some time to let the sensor initialize
	BOOLEAN available = FALSE;
	for (size_t i = 0; i < 100; ++i) // 10 Seconds
	{
		hr = pKinectSensor->get_IsAvailable(&available);
		if (SUCCEEDED(hr) && available)
			break;
		Sleep(100);
	}

	if (!available)
	{
		pKinectSensor->Close();
		pKinectSensor->Release();
		updateKinect2StatusSHM(S_FALSE);
		return ONI_STATUS_NO_DEVICE;
	}

	// Get sensor info
	auto pInfo = XN_NEW(OniDeviceInfo);
	WCHAR sensorId[ONI_MAX_STR];
	pKinectSensor->get_UniqueKinectId(ONI_MAX_STR, sensorId);
	size_t convertedChars = 0;
	const size_t newsize = ONI_MAX_STR;
	size_t origsize = wcslen(sensorId) + 1;
	wcstombs_s(&convertedChars, pInfo->uri, origsize, sensorId, _TRUNCATE);
	xnOSStrCopy(pInfo->vendor, VENDOR_VAL, ONI_MAX_STR);
	xnOSStrCopy(pInfo->name, NAME_VAL, ONI_MAX_STR);
	m_devices[pInfo] = nullptr;
	deviceConnected(pInfo);
	deviceStateChanged(pInfo, S_OK); // Sensor is ready

	// Close sensor instance
	pKinectSensor->Close();
	pKinectSensor->Release();

	updateKinect2StatusSHM(S_OK);
	return ONI_STATUS_OK;
}

DeviceBase* Kinect2Driver::deviceOpen(const char* uri, const char* /*mode*/)
{
	for (xnl::Hash<OniDeviceInfo*, DeviceBase*>::Iterator iter = m_devices.Begin(); iter != m_devices.End()
	     ; ++iter)
	{
		if (xnOSStrCmp(iter->Key()->uri, uri) == 0)
		{
			// Found
			if (iter->Value() != nullptr)
			{
				// already using
				updateKinect2StatusSHM(S_OK);
				return iter->Value();
			}
			// Get sensor instance
			IKinectSensor* pKinectSensor = nullptr;
			HRESULT hr;
			hr = GetDefaultKinectSensor(&pKinectSensor);
			if (FAILED(hr))
			{
				if (pKinectSensor)
				{
					pKinectSensor->Release();
				}
				updateKinect2StatusSHM(S_FALSE);
				return nullptr;
			}

			// Compare sensor id. TODO: To be removed when multi-device support is added
			char sensorUri[ONI_MAX_STR];
			WCHAR sensorId[ONI_MAX_STR];
			pKinectSensor->get_UniqueKinectId(ONI_MAX_STR, sensorId);
			size_t convertedChars = 0;
			const size_t newsize = ONI_MAX_STR;
			size_t origsize = wcslen(sensorId) + 1;
			wcstombs_s(&convertedChars, sensorUri, origsize, sensorId, _TRUNCATE);
			if (xnOSStrCmp(iter->Key()->uri, sensorUri) != 0)
			{
				pKinectSensor->Release();
				updateKinect2StatusSHM(S_FALSE);
				return nullptr;
			}

			// Initialize the Kinect2
			hr = pKinectSensor->Open();
			auto pDevice = XN_NEW(Kinect2Device, pKinectSensor);
			if (!pDevice)
			{
				pKinectSensor->Close();
				pKinectSensor->Release();
				updateKinect2StatusSHM(S_FALSE);
				return nullptr;
			}
			iter->Value() = pDevice;
			updateKinect2StatusSHM(S_OK);
			return pDevice;
		}
	}
	return nullptr;
}

void Kinect2Driver::deviceClose(DeviceBase* pDevice)
{
	for (xnl::Hash<OniDeviceInfo*, DeviceBase*>::Iterator iter = m_devices.Begin(); iter != m_devices.End()
	     ; ++iter)
	{
		if (iter->Value() == pDevice)
		{
			iter->Value() = nullptr;
			XN_DELETE(pDevice);
			return;
		}
	}

	// not our device?!
	XN_ASSERT(FALSE);
}

void Kinect2Driver::shutdown()
{
}

OniStatus Kinect2Driver::tryDevice(const char* uri)
{
	return ONI_STATUS_OK;
}

void* Kinect2Driver::enableFrameSync(StreamBase** pStreams, int streamCount)
{
	return nullptr;
}

void Kinect2Driver::disableFrameSync(void* frameSyncGroup)
{
}

ONI_EXPORT_DRIVER(kinect2_device::Kinect2Driver)
