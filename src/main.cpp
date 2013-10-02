#include <unistd.h>

#include "DeckLinkCaptureStreamer.h"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "decklinkcapture_streamer");

  // TODO make setable via launch file
  std::string imageTopic = "/decklink/rawimage";

  DeckLinkCaptureStreamer streamer(imageTopic);

  streamer.init();

  ros::spin();

  return 0;
}
