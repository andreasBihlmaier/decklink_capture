#include "DeckLinkCapture.h"

// system includes
#include <stdio.h>

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
DeckLinkCapture::DeckLinkCapture()
  :m_refCount(0),
   m_frameCount(0),
   m_callback(NULL)
{
}

HRESULT
DeckLinkCapture::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* audioFrame)
{
  if (!videoFrame) {
    return S_OK;
  }

  m_frameCount++;

  if (videoFrame->GetFlags() & bmdFrameHasNoInputSource) {
			fprintf(stderr, "Frame %lu: No input signal\n", m_frameCount);
  }

  if (!m_callback) {
			fprintf(stderr, "Frame %lu: No callback registered, dropping frame\n", m_frameCount);
      return S_OK;
  } else {
    void* rawFrame;
    videoFrame->GetBytes(&rawFrame);
    m_callback(m_frameCount, rawFrame, videoFrame->GetWidth(), videoFrame->GetHeight(), videoFrame->GetRowBytes() / videoFrame->GetWidth());
  }

  return S_OK;
}

bool
DeckLinkCapture::init()
{
	IDeckLinkIterator* deckLinkIterator = CreateDeckLinkIteratorInstance();
  IDeckLink* deckLink;
	if (deckLinkIterator->Next(&deckLink) != S_OK) {
    fprintf(stderr, "init(): deckLinkIterator: No DeckLink device found\n");
  }

  IDeckLinkInput* deckLinkInput;
	if (deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&deckLinkInput) != S_OK) {
    fprintf(stderr, "init(): QueryInterface() failed\n");
    return false;
  }
	deckLinkInput->SetCallback(this);

	BMDDisplayMode displayMode = bmdModeHD1080i6000;
	BMDPixelFormat pixelFormat = bmdFormat8BitYUV;
  if (deckLinkInput->EnableVideoInput(displayMode, pixelFormat, 0) != S_OK) {
    fprintf(stderr, "init(): EnableVideoInput() failed\n");
    return false;
  }

	if (deckLinkInput->StartStreams() != S_OK) {
    fprintf(stderr, "init(): StartStreams() failed\n");
    return false;
  }

  return true;
}

void
DeckLinkCapture::setCallback(DeckLinkCaptureCallbackType p_callback)
{
  m_callback = p_callback;
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
