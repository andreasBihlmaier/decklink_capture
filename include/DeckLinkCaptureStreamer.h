#ifndef _DECK_LINK_CAPTURE_STREAMER_H_
#define _DECK_LINK_CAPTURE_STREAMER_H_

// system includes
#include <queue>

// library includes
#include "ros/ros.h"
#include <boost/thread/thread.hpp>

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
    DeckLinkCaptureStreamer(const std::string& p_imagePublishTopic);

    // overwritten methods

    // methods
    bool init();
    void newFrame(long unsigned frameCount, void* rawFrame, unsigned rawFrameWidth, unsigned rawFrameHeight, unsigned bytesPerPixel);

    // variables


  private:
    // methods
    void publishImages();

    // variables
    std::string m_imagePublishTopic;

    DeckLinkCapture* m_deckLinkCapture;
    unsigned m_frameWidth;
    unsigned m_frameHeight;
    unsigned m_frameBytesPerPixel;
    unsigned m_frameByteSize;
    unsigned m_frameCount; // protected by m_imageQueueMutex

    boost::mutex m_imageQueueMutex;
    boost::condition_variable m_imageQueueCondition;
    std::queue<void*> m_imageQueue;
    boost::thread* m_publishThread;

    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_imagePublisher;

};

#endif // _DECK_LINK_CAPTURE_STREAMER_H_
