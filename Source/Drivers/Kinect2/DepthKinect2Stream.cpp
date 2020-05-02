#include "DepthKinect2Stream.h"
#include "PS1080.h"
#include "Kinect2StreamImpl.h"

using namespace oni::driver;
using namespace kinect2_device;

#define FILTER_RELIABLE_DEPTH_VALUE(VALUE) (((VALUE) < DEVICE_MAX_DEPTH_VAL) ? (VALUE) : 0)

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
}

DepthKinect2Stream::DepthKinect2Stream(Kinect2StreamImpl* pStreamImpl)
  : BaseKinect2Stream(pStreamImpl)
{
  m_videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
  m_videoMode.fps = DEFAULT_FPS;
  m_videoMode.resolutionX = 512;
  m_videoMode.resolutionY = 424;
  m_colorSpaceCoords = new ColorSpacePoint[512*424];
  m_registeredDepthMap = new UINT16[512*424];
}

DepthKinect2Stream::~DepthKinect2Stream()
{
  delete[] m_colorSpaceCoords;
  delete[] m_registeredDepthMap;
}

void DepthKinect2Stream::frameReady(void* data, int width, int height, double timestamp)
{
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
  case ONI_STREAM_PROPERTY_MIRRORING:
    {
      XnBool * val = (XnBool *)data;
      *val = TRUE;
      status = ONI_STATUS_OK;
      break;
    }
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
	  for (int i = 1; i <= 1052; i++)
		  s2d[i] = static_cast<uint16_t>(342205.0 / (1086.671 - i));
	  status = ONI_STATUS_OK;
	  break;
  }
  case XN_STREAM_PROPERTY_D2S_TABLE:              // unsigned short[]
  {
	  uint16_t* d2s = (uint16_t*)data;
	  *pDataSize = sizeof(*d2s) * 10001;
	  memset(data, 0, *pDataSize);
	  for (int i = 315; i <= 10000; i++)
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
  case ONI_STREAM_PROPERTY_MIRRORING:
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

    XnBool nBool;
	size = sizeof(nBool);
	getProperty(ONI_STREAM_PROPERTY_MIRRORING, &nBool, &size);
	raisePropertyChanged(ONI_STREAM_PROPERTY_MIRRORING, &nBool, size);

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

	unsigned short nBuff[10001];
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

void DepthKinect2Stream::copyDepthPixelsStraight(const UINT16* data_in, int width, int height, OniFrame* pFrame)
{
  // Copy the depth pixels to OniDriverFrame
  // with applying cropping but NO depth-to-image registration.

  const int xStride = width/m_videoMode.resolutionX;
  const int yStride = height/m_videoMode.resolutionY;
  const int frameX = pFrame->cropOriginX * xStride;
  const int frameY = pFrame->cropOriginY * yStride;
  const int frameWidth = pFrame->width * xStride;
  const int frameHeight = pFrame->height * yStride;

  unsigned short* data_out = (unsigned short*) pFrame->data;
  for (int y = frameY; y < frameY + frameHeight; y += yStride) {
    for (int x = frameX; x < frameX + frameWidth; x += xStride) {
      unsigned short* iter = const_cast<unsigned short*>(data_in + (y*width + x));
      *data_out = FILTER_RELIABLE_DEPTH_VALUE(*iter);
      data_out++;
    }
  }
}

void DepthKinect2Stream::copyDepthPixelsWithImageRegistration(const UINT16* data_in, int width, int height, OniFrame* pFrame)
{
  // Copy the depth pixels to OniDriverFrame
  // with applying cropping and depth-to-image registration.

  const int xStride = width/m_videoMode.resolutionX;
  const int yStride = height/m_videoMode.resolutionY;
  const int frameX = pFrame->cropOriginX * xStride;
  const int frameY = pFrame->cropOriginY * yStride;
  const int frameWidth = pFrame->width * xStride;
  const int frameHeight = pFrame->height * yStride;
  const float xFactor = static_cast<float>(width)/1920.0f;
  const float yFactor = static_cast<float>(height)/1080.0f;

  ICoordinateMapper* coordinateMapper = m_pStreamImpl->getCoordinateMapper();
  if (coordinateMapper == NULL) {
    return;
  }

  HRESULT hr = coordinateMapper->MapDepthFrameToColorSpace(width*height, data_in, width*height, m_colorSpaceCoords);
  if (FAILED(hr)) {
    return;
  }

  unsigned short* data_out = (unsigned short*) m_registeredDepthMap;
  xnOSMemSet(data_out, 0, width*height*2);

  const ColorSpacePoint* mappedCoordsIter = m_colorSpaceCoords;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      const float fX = mappedCoordsIter->X*xFactor;
      const float fY = mappedCoordsIter->Y*yFactor;
      const int cx = static_cast<int>(fX + 0.5f);
      const int cy = static_cast<int>(fY + 0.5f);
      if (cx >= 0 && cy >= 0 && cx < width && cy < height) {
        unsigned short* iter = const_cast<unsigned short*>(data_in + (y*width + x));
        const unsigned short d = FILTER_RELIABLE_DEPTH_VALUE(*iter);
        unsigned short* const p = data_out + cx + cy * width;
        if (*p == 0 || *p > d) *p = d;
      }
      mappedCoordsIter++;
    }
  }

  // Fill vertical gaps caused by the difference in the aspect ratio between depth and color resolutions
  data_out = (unsigned short*) pFrame->data;
  for (int y = frameY; y < frameY + frameHeight; y += yStride) {
    for (int x = frameX; x < frameX + frameWidth; x += xStride) {
      unsigned short* iter = const_cast<unsigned short*>(m_registeredDepthMap + (y*width + x));
      if (*iter == 0) {
        unsigned short davg = 0;
        int dw = 0;
        for (int ky = max(y - 1, 0); ky <= y + 1 && ky < height; ky++) {
          unsigned short* kiter = const_cast<unsigned short*>(m_registeredDepthMap + (ky*width + x));
          if (*kiter != 0) {
            davg += *kiter;
            dw += abs(ky - y);
          }
        }
        *data_out = davg;
        if (dw) {
          *data_out /= dw;
        }
      }
      else {
        *data_out = FILTER_RELIABLE_DEPTH_VALUE(*iter);
      }
      data_out++;
    }
  }
}
