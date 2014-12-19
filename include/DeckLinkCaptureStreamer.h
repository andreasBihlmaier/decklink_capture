#ifndef _DECK_LINK_CAPTURE_STREAMER_H_
#define _DECK_LINK_CAPTURE_STREAMER_H_

// system includes
#include <queue>

// library includes
#include "ros/ros.h"
#include <boost/thread/thread.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

// custom includes


// forward declarations
class DeckLinkCapture;



class DeckLinkCaptureStreamer
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors
    DeckLinkCaptureStreamer();

    // overwritten methods

    // methods
    bool init();
    void newFrame(long unsigned frameCount, void* rawFrame, unsigned rawFrameWidth, unsigned rawFrameHeight, unsigned bytesPerPixel);

    // variables


  private:
    // methods
    void publishImages();

    // variables
    std::string m_cameraName;

    DeckLinkCapture* m_deckLinkCapture;
    unsigned m_frameWidth;
    unsigned m_frameHeight;
    unsigned m_frameBytesPerPixel;
    unsigned m_frameByteSize;
    unsigned m_frameCount; // protected by m_imageQueueMutex
    std::string m_frameId;

    boost::mutex m_imageQueueMutex;
    boost::condition_variable m_imageQueueCondition;
    std::queue<void*> m_imageQueue;
    boost::thread* m_publishThread;

    ros::NodeHandle m_nodeHandle;
    image_transport::CameraPublisher m_imagePublisher;
    image_transport::ImageTransport* m_imageTransport;
    camera_info_manager::CameraInfoManager* m_cameraInfoManager;

};

#endif // _DECK_LINK_CAPTURE_STREAMER_H_
