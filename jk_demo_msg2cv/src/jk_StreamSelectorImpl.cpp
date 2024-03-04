
#include <rclcpp/rclcpp.hpp>

#include "jk_StreamSelectorImpl.hpp"


void JK::StreamSelectorImpl::setStreamsTotalNumber(int howMany){

  if(howMany < 1)
    return;
  m_lastStreamIndex = --howMany;
  
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void JK::StreamSelectorImpl::switchToNextStream(){
  
  if(m_lastStreamIndex!= -1){
    if( m_selectedStream < m_lastStreamIndex)
      ++m_selectedStream;
    else 
      m_selectedStream = 0;
  } else
      RCLCPP_WARN(rclcpp::get_logger("StreamSelectorImpl"), "Stream selector not initialized.");
    
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

int JK::StreamSelectorImpl::getSelectedStream() const {

  return m_selectedStream;
  
}

