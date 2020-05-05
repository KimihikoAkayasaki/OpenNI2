#include <cmath>
#include <cassert>
#include <algorithm>
#include "DepthKinect2Stream.h"
#include "PS1080.h"
#include "Kinect2StreamImpl.h"

#undef min
#undef max

using namespace oni::driver;
using namespace kinect2_device;

namespace
{;
constexpr const int DEFAULT_FPS = 30;
constexpr const XnInt MAX_SHIFT_VAL = 2047;
constexpr const XnInt PARAM_COEFF_VAL = 4;
constexpr const XnInt SHIFT_SCALE_VAL = 10;
constexpr const XnInt GAIN_VAL = 42;
constexpr const XnInt ZPD_VAL = 120;
constexpr const XnInt CONST_SHIFT_VAL = 200;
constexpr const XnInt DEVICE_MAX_DEPTH_VAL = 10000;
constexpr const XnDouble ZPPS_VAL = 0.10520000010728836;
constexpr const XnDouble LDDIS_VAL = 7.5;

template < typename DepthType >
constexpr inline DepthType filter_reliable_depth_value(const DepthType& depth_value)
{
    return depth_value < DEVICE_MAX_DEPTH_VAL ? depth_value : 0;
};

template < typename PixelType >
struct CopyData
{
	PixelType* data;
	std::size_t xOffset;
	std::size_t yOffset;
	std::size_t stride;
};

template < const bool mirroring, typename PixelType, typename PixelKernelFn  >
constexpr inline void copy_frame
(
	const CopyData<const PixelType>& src,
	const CopyData<PixelType>& dst,
	const std::size_t width, const std::size_t height,
	PixelKernelFn&& kernelFn
)
{
	constexpr const auto first = [](auto& cd)
	{
		return cd.data + cd.xOffset + cd.yOffset * cd.stride;
	};
	constexpr const auto colPtr = [](auto& cd, auto pixPtr, const std::size_t y)
	{
		return pixPtr + y * cd.stride;
	};

	const auto srcPix = first(src);
	const auto dstPix = first(dst);

	for (std::size_t y = 0; y < height; ++y)
	{
		const PixelType* srcPtr = colPtr(src, srcPix, y);
		PixelType* dstPtr = colPtr(dst, dstPix, y);
		if constexpr (mirroring)
		{
			dstPtr += width;
			for (std::size_t x = 0; x < width; ++x)
				*dstPtr-- = kernelFn(*srcPtr++, x, y);
		}
		else
		{
			for (std::size_t x = 0; x < width; ++x)
				*dstPtr++ = kernelFn(*srcPtr++, x, y);
		}
	}
}

template < const bool mirroring, typename PixelType >
constexpr inline void copy_frame
(
	const CopyData<const PixelType>& src,
	const CopyData<PixelType>& dst,
	const std::size_t width, const std::size_t height
)
{
	copy_frame<mirroring, PixelType>(src, dst, width, height, [](const PixelType p, auto, auto) { return filter_reliable_depth_value(p); });
}
}

DepthKinect2Stream::DepthKinect2Stream(Kinect2StreamImpl* pStreamImpl)
  : BaseKinect2Stream(pStreamImpl)
{
  m_videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
  m_videoMode.fps = DEFAULT_FPS;
  m_videoMode.resolutionX = 640; // 512;
  m_videoMode.resolutionY = 480; // 424;
  m_colorSpaceCoords = new ColorSpacePoint[512*424];
  m_registeredDepthMap = new UINT16[512*424];
}

DepthKinect2Stream::~DepthKinect2Stream()
{
  delete[] m_colorSpaceCoords;
  delete[] m_registeredDepthMap;
}

void kinect2_device::DepthKinect2Stream::frameReady(void* data, std::size_t width, std::size_t height, double timestamp)
{
    assert(width >= 0 && height >= 0);
  OniFrame* pFrame = getServices().acquireFrame();
  pFrame->videoMode.resolutionY = m_videoMode.resolutionY;
  pFrame->videoMode.resolutionX = m_videoMode.resolutionX;
  pFrame->croppingEnabled = m_cropping.enabled;
  if (m_cropping.enabled)
  {
    pFrame->width = m_cropping.width;
    pFrame->height = m_cropping.height;
    pFrame->cropOriginX = m_cropping.originX;
    pFrame->cropOriginY = m_cropping.originY;
  }
  else {
    pFrame->cropOriginX = 0;
    pFrame->cropOriginY = 0;
    pFrame->width = m_videoMode.resolutionX;
    pFrame->height = m_videoMode.resolutionY;
  }
  pFrame->dataSize = pFrame->height * pFrame->width * 2;
  pFrame->stride = pFrame->width * 2;
  pFrame->videoMode.pixelFormat = m_videoMode.pixelFormat;
  pFrame->videoMode.fps = m_videoMode.fps;
  pFrame->sensorType = ONI_SENSOR_DEPTH;
  pFrame->frameIndex = m_frameIdx++;
  pFrame->timestamp = static_cast<int>(timestamp);

  UINT16* data_in = reinterpret_cast<UINT16*>(data);
  if (m_pStreamImpl->getImageRegistrationMode() == ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR) {
    copyDepthPixelsWithImageRegistration(data_in, width, height, pFrame);
  } else {
    copyDepthPixelsStraight(data_in, width, height, pFrame);
  }

  raiseNewFrame(pFrame);
  getServices().releaseFrame(pFrame);
}

OniStatus DepthKinect2Stream::getProperty(int propertyId, void* data, int* pDataSize)
{
  OniStatus status = ONI_STATUS_NOT_SUPPORTED;
  switch (propertyId)
  {
  case ONI_STREAM_PROPERTY_MAX_VALUE:
    {
      XnInt * val = (XnInt *)data;
      *val = DEVICE_MAX_DEPTH_VAL;
      status = ONI_STATUS_OK;
      break;
    }
  //case ONI_STREAM_PROPERTY_MIRRORING:
  //  {
  //    XnBool * val = (XnBool *)data;
  //    *val = FALSE;
  //    status = ONI_STATUS_OK;
  //    break;
  //  }
  case XN_STREAM_PROPERTY_GAIN:
  {
	  XnInt* val = (XnInt*)data;
	  *val = GAIN_VAL;
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_CONST_SHIFT:
  {
	  XnInt* val = (XnInt*)data;
	  *val = CONST_SHIFT_VAL;
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_MAX_SHIFT:
  {
	  XnInt* val = (XnInt*)data;
	  *val = MAX_SHIFT_VAL;
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_PARAM_COEFF:
  {
	  XnInt* val = (XnInt*)data;
	  *val = PARAM_COEFF_VAL;
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_SHIFT_SCALE:
  {
	  XnInt* val = (XnInt*)data;
	  *val = SHIFT_SCALE_VAL;
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE:
  {
	  XnInt* val = (XnInt*)data;
	  *val = ZPD_VAL;
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE:
  {
	  XnDouble* val = (XnDouble*)data;
	  *val = ZPPS_VAL;
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE:
  {
	  XnDouble* val = (XnDouble*)data;
	  *val = LDDIS_VAL;
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_S2D_TABLE:              // OniDepthPixel[]
  {
	  uint16_t* s2d = (uint16_t*)data;
	  *pDataSize = sizeof(*s2d) * 2048;
	  memset(data, 0, *pDataSize);
	  for (std::size_t i = 1; i <= 1052; ++i)
		  s2d[i] = static_cast<uint16_t>(342205.0 / (1086.671 - i));
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_D2S_TABLE:              // unsigned short[]
  {
	  uint16_t* d2s = (uint16_t*)data;
	  *pDataSize = sizeof(*d2s) * 10001;
	  memset(data, 0, *pDataSize);
	  for (std::size_t i = 315; i <= 10000; ++i)
		  d2s[i] = static_cast<uint16_t>(1086.671 - 342205.0 / (i + 1));
	  status = ONI_STATUS_OK;
	  break;
  }
  default:
    status = BaseKinect2Stream::getProperty(propertyId, data, pDataSize);
    break;
  }

  return status;
}

OniBool DepthKinect2Stream::isPropertySupported(int propertyId)
{
  OniBool status = FALSE;
  switch (propertyId)
  {
  case ONI_STREAM_PROPERTY_MAX_VALUE:
  //case ONI_STREAM_PROPERTY_MIRRORING:
  case XN_STREAM_PROPERTY_GAIN:
  case XN_STREAM_PROPERTY_CONST_SHIFT:
  case XN_STREAM_PROPERTY_MAX_SHIFT:
  case XN_STREAM_PROPERTY_PARAM_COEFF:
  case XN_STREAM_PROPERTY_SHIFT_SCALE:
  case XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE:
  case XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE:
  case XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE:
  case XN_STREAM_PROPERTY_S2D_TABLE:
  case XN_STREAM_PROPERTY_D2S_TABLE:
    status = TRUE;
  default:
    status = BaseKinect2Stream::isPropertySupported(propertyId);
    break;
  }
  return status;
}

void DepthKinect2Stream::notifyAllProperties()
{
	XnInt nInt;
    int size = sizeof(nInt);
	getProperty(ONI_STREAM_PROPERTY_MAX_VALUE, &nInt, &size);
	raisePropertyChanged(ONI_STREAM_PROPERTY_MAX_VALUE, &nInt, size);

 //   XnBool nBool;
	//size = sizeof(nBool);
	//getProperty(ONI_STREAM_PROPERTY_MIRRORING, &nBool, &size);
	//raisePropertyChanged(ONI_STREAM_PROPERTY_MIRRORING, &nBool, size);

	XnDouble nDouble;
	size = sizeof(nDouble);
	getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE, &nDouble, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE, &nDouble, size);

	getProperty(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE, &nDouble, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE, &nDouble, size);

	getProperty(XN_STREAM_PROPERTY_GAIN, &nInt, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_GAIN, &nInt, size);

	getProperty(XN_STREAM_PROPERTY_CONST_SHIFT, &nInt, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_CONST_SHIFT, &nInt, size);

	getProperty(XN_STREAM_PROPERTY_MAX_SHIFT, &nInt, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_MAX_SHIFT, &nInt, size);

	getProperty(XN_STREAM_PROPERTY_SHIFT_SCALE, &nInt, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_SHIFT_SCALE, &nInt, size);

	getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE, &nInt, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE, &nInt, size);

    static thread_local XnUInt16 nBuff[10001];
    size = sizeof(XnUInt16) * 2048;
	getProperty(XN_STREAM_PROPERTY_S2D_TABLE, nBuff, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_S2D_TABLE, nBuff, size);

    size = sizeof(XnUInt16) * 10001;
	getProperty(XN_STREAM_PROPERTY_D2S_TABLE, nBuff, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_D2S_TABLE, nBuff, size);

	size = sizeof(PARAM_COEFF_VAL);
	getProperty(XN_STREAM_PROPERTY_PARAM_COEFF, nBuff, &size);
	raisePropertyChanged(XN_STREAM_PROPERTY_PARAM_COEFF, nBuff, size);

    BaseKinect2Stream::notifyAllProperties();
}

void DepthKinect2Stream::copyDepthPixelsStraight(const UINT16* data_in, std::size_t width, std::size_t height, OniFrame* pFrame)
{
	if (width < pFrame->width || height < pFrame->height)
		memset(pFrame->data, 0x00, static_cast<std::size_t>(pFrame->width) * static_cast<std::size_t>(pFrame->height) * sizeof(UINT16));

	const auto targetWidth = std::min(width, (std::size_t)pFrame->width);
	const auto targetHeight = std::min(height, (std::size_t)pFrame->height);

	copy_frame<false,UINT16>
    (
        { data_in, (std::size_t)pFrame->cropOriginX, (std::size_t)pFrame->cropOriginY, (std::size_t)width },
        { static_cast<UINT16*>(pFrame->data), 0u, 0u, (std::size_t)pFrame->width },
        targetWidth, targetHeight
    );
}

void DepthKinect2Stream::copyDepthPixelsWithImageRegistration(const UINT16* data_in, std::size_t width, std::size_t height, OniFrame* pFrame)
{
    // Copy the depth pixels to OniDriverFrame
    // with applying cropping and depth-to-image registration.

    ICoordinateMapper* coordinateMapper = m_pStreamImpl->getCoordinateMapper();
    if (coordinateMapper == NULL)
        return;
    
    const UINT pointCount = static_cast<UINT>(width * height);
    const HRESULT hr = coordinateMapper->MapDepthFrameToColorSpace(pointCount, data_in, pointCount, m_colorSpaceCoords);
    if (FAILED(hr))
        return;

    unsigned short* data_out = (unsigned short*) m_registeredDepthMap;
    xnOSMemSet(data_out, 0, width*height*2);

	const float xFactor = static_cast<float>(width) / 1920.0f;
	const float yFactor = static_cast<float>(height) / 1080.0f;
    const ColorSpacePoint* mappedCoordsIter = m_colorSpaceCoords;
    for (std::size_t y = 0; y < height; ++y)
    {
        for (std::size_t x = 0; x < width; ++x)
        {
            const float fX = mappedCoordsIter->X * xFactor;
            const float fY = mappedCoordsIter->Y * yFactor;
            const std::size_t cx = static_cast<std::size_t>(fX + 0.5);
            const std::size_t cy = static_cast<std::size_t>(fY + 0.5);
            if (cx >= 0 && cy >= 0 && cx < width && cy < height)
            {
                unsigned short* iter = const_cast<unsigned short*>(data_in + (y * width + x));
                const unsigned short d = filter_reliable_depth_value(*iter);
                unsigned short* const p = data_out + cx + cy * width;
                if (*p == 0 || *p > d) *p = d;
            }
            mappedCoordsIter++;
        }
    }

    if (width < pFrame->width || height < pFrame->height)
	    memset(pFrame->data, 0x00, static_cast<std::size_t>(pFrame->width) * static_cast<std::size_t>(pFrame->height) * sizeof(UINT16));

    const auto targetWidth = std::min(width, (std::size_t)pFrame->width);
    const auto targetHeight = std::min(height, (std::size_t)pFrame->height);

	// Fill vertical gaps caused by the difference in the aspect ratio between depth and color resolutions
	const auto kernelFn = [&](const UINT16 depth_value, const std::size_t x, const std::size_t y) -> UINT16
	{
		if (depth_value != 0)
			return filter_reliable_depth_value(depth_value);

		UINT16 davg = 0u;
		std::size_t dw = 0;
		for (int ky = std::max(static_cast<int>(y) - 1, 0); ky <= y + 1 && ky < height; ++ky)
		{
			const UINT16 kiter = *(m_registeredDepthMap + ky * width + x);
			if (kiter != 0)
			{
				davg += kiter;
				dw += std::abs(ky - static_cast<int>(y));
			}
		}
		if (dw > 0)
		{
			assert(dw <= ((std::size_t)std::numeric_limits<UINT16>::max()));
			davg /= static_cast<UINT16>(dw);
		}
		return davg;
	};
    copy_frame<false, UINT16>
    (
        { m_registeredDepthMap, (std::size_t)pFrame->cropOriginX, (std::size_t)pFrame->cropOriginY, (std::size_t)width },
	    { static_cast<UINT16*>(pFrame->data), 0u, 0u, (std::size_t)pFrame->width },
	    targetWidth, targetHeight,
        kernelFn
    );

}
