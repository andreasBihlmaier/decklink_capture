#include "DeckLinkCaptureStreamer.h"

// system includes

// library includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// custom includes
#include "DeckLinkCapture.h"


/*---------------------------------- public: -----------------------------{{{-*/
DeckLinkCaptureStreamer::DeckLinkCaptureStreamer()
  :m_cameraName("decklink"),
   m_deckLinkCapture(NULL),
   m_frameWidth(0),
   m_frameHeight(0),
   m_frameBytesPerPixel(0),
   m_frameByteSize(0)
{
}

bool
DeckLinkCaptureStreamer::init()
{
  m_imageTransport = new image_transport::ImageTransport(m_nodeHandle);
  m_cameraInfoManager = new camera_info_manager::CameraInfoManager(m_nodeHandle);
  m_cameraInfoManager->setCameraName(m_cameraName);
  m_imagePublisher = m_imageTransport->advertiseCamera("image_raw", 1);
  m_publishThread = new boost::thread(boost::bind(&DeckLinkCaptureStreamer::publishImages, this));
  ros::NodeHandle privateNodeHandle("~");
  if (privateNodeHandle.hasParam("frame_id")) {
    privateNodeHandle.getParam("frame_id", m_frameId);
  }

  m_deckLinkCapture = new DeckLinkCapture();
  if (!m_deckLinkCapture->init()) {
    ROS_ERROR("Failed to init DeckLinkCapture");
    return false;
  }

  m_deckLinkCapture->setCallback(boost::bind(&DeckLinkCaptureStreamer::newFrame, this, _1, _2, _3, _4, _5));

  return true;
}

void
DeckLinkCaptureStreamer::newFrame(long unsigned frameCount, void* rawFrame, unsigned rawFrameWidth, unsigned rawFrameHeight, unsigned bytesPerPixel)
{
  //printf("Received frame: count=%lu %ux%u bytesPerPixel=%u\n", frameCount, rawFrameWidth, rawFrameHeight, bytesPerPixel);
  if (m_frameWidth == 0) {
    m_frameWidth = rawFrameWidth;
    m_frameHeight = rawFrameHeight;
    m_frameBytesPerPixel = bytesPerPixel;
    m_frameByteSize = m_frameWidth * m_frameHeight * m_frameBytesPerPixel;
  }

  void* rawFrameCopy = malloc(m_frameByteSize);
  if (!rawFrameCopy) {
    ROS_ERROR("frame %lu: could not malloc()\n", frameCount);
    return;
  }
  memcpy(rawFrameCopy, rawFrame, m_frameByteSize);

  boost::lock_guard<boost::mutex> imageQueueLock(m_imageQueueMutex);
  m_imageQueue.push(rawFrameCopy);
  m_frameCount = frameCount;
  if (m_imageQueue.size() > 10) {
    ROS_WARN("frame %lu: m_imageQueue.size()=%zd\n", frameCount, m_imageQueue.size());
  }
  m_imageQueueCondition.notify_one();
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
DeckLinkCaptureStreamer::publishImages()
{
  for (;;) {
    void* frameRaw;
    {
      boost::unique_lock<boost::mutex> imageQueueLock(m_imageQueueMutex);
      while (m_imageQueue.empty()) {
        m_imageQueueCondition.wait(imageQueueLock);
      }

      frameRaw = m_imageQueue.front();
      m_imageQueue.pop();
    }

    sensor_msgs::Image image;
    image.header.seq = m_frameCount;
    image.header.stamp = ros::Time::now();
    image.header.frame_id = m_frameId;
    image.height = m_frameHeight;
    image.width = m_frameWidth;
    image.encoding = sensor_msgs::image_encodings::YUV422;
    //image.is_bigendian
    image.step = m_frameWidth * m_frameBytesPerPixel;
    image.data = std::vector<uint8_t>((uint8_t*)frameRaw, ((uint8_t*)frameRaw) + m_frameByteSize);
    sensor_msgs::CameraInfo cameraInfo = m_cameraInfoManager->getCameraInfo();
    cameraInfo.header.stamp = image.header.stamp;
    cameraInfo.header.frame_id = image.header.frame_id;
    m_imagePublisher.publish(image, cameraInfo);

    free(frameRaw);
  }
}
/*------------------------------------------------------------------------}}}-*/
