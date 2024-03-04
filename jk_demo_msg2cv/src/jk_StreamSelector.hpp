
#ifndef JK_STREAMSELECTOR_HPP
#define JK_STREAMSELECTOR_HPP

namespace JK {


class StreamSelector {

public:
  virtual ~StreamSelector() = default;
  virtual void switchToNextStream() = 0;
  virtual void setStreamsTotalNumber(int) = 0;
  virtual int getSelectedStream() const = 0;
 
};

}

#endif

