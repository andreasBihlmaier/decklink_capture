#ifndef _DECK_LINK_CAPTURE_STREAMER_H_
#define _DECK_LINK_CAPTURE_STREAMER_H_

// system includes

// library includes

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
    void newFrame(long unsigned frameCount, void* rawFrame, unsigned rawFrameRowbytes, unsigned rawFrameHeight);

    // variables


  private:
    // methods

    // variables
    DeckLinkCapture* m_deckLinkCapture;


};

#endif // _DECK_LINK_CAPTURE_STREAMER_H_
