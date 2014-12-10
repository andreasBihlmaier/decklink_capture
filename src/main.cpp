#include <unistd.h>

#include "DeckLinkCaptureStreamer.h"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "decklinkcapture_streamer");

  DeckLinkCaptureStreamer streamer;

  streamer.init();

  ros::spin();

  return 0;
}
