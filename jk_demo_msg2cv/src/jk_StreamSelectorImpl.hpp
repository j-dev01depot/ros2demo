
#ifndef JK_STREAMSELECTOR_IMPL_HPP
#define JK_STREAMSELECTOR_IMPL_HPP

#include "jk_StreamSelector.hpp"

namespace JK {


class StreamSelectorImpl : public StreamSelector {
  
  // the default -1 value states that the StreamSelector hasn't been configured 
  // and the stream last index hasn't been set
  std::atomic<int> m_lastStreamIndex {-1};
  // the 0 index number points the first stream and, in case, the only avaiable
  std::atomic<int> m_selectedStream {0};

public:

  StreamSelectorImpl() = default;
  void switchToNextStream() override;
  void setStreamsTotalNumber(int howMany) override;
  int getSelectedStream() const override;
 
};

}

#endif

