#include "DeckLinkCaptureStreamer.h"

// system includes

// library includes
#include "ros/ros.h"

// custom includes
#include "DeckLinkCapture.h"


/*---------------------------------- public: -----------------------------{{{-*/
DeckLinkCaptureStreamer::DeckLinkCaptureStreamer()
  : m_deckLinkCapture(NULL)
{
}

bool
DeckLinkCaptureStreamer::init()
{
  m_deckLinkCapture = new DeckLinkCapture();
  if (!m_deckLinkCapture->init()) {
    ROS_ERROR("Failed to init DeckLinkCapture");
    return false;
  }

  m_deckLinkCapture->setCallback(boost::bind(&DeckLinkCaptureStreamer::newFrame, this, _1, _2, _3, _4));
}

void
DeckLinkCaptureStreamer::newFrame(long unsigned frameCount, void* rawFrame, unsigned rawFrameRowbytes, unsigned rawFrameHeight)
{
  printf("Received frame: count=%lu rawFrameRowbytes=%u rawFrameHeight=%u\n", frameCount, rawFrameRowbytes, rawFrameHeight);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
