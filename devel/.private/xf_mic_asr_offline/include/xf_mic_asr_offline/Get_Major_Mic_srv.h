// Generated by gencpp from file xf_mic_asr_offline/Get_Major_Mic_srv.msg
// DO NOT EDIT!


#ifndef XF_MIC_ASR_OFFLINE_MESSAGE_GET_MAJOR_MIC_SRV_H
#define XF_MIC_ASR_OFFLINE_MESSAGE_GET_MAJOR_MIC_SRV_H

#include <ros/service_traits.h>


#include <xf_mic_asr_offline/Get_Major_Mic_srvRequest.h>
#include <xf_mic_asr_offline/Get_Major_Mic_srvResponse.h>


namespace xf_mic_asr_offline
{

struct Get_Major_Mic_srv
{

typedef Get_Major_Mic_srvRequest Request;
typedef Get_Major_Mic_srvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Get_Major_Mic_srv
} // namespace xf_mic_asr_offline


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srv > {
  static const char* value()
  {
    return "68ec108b7ba6ff9867de0e21598a80d5";
  }

  static const char* value(const ::xf_mic_asr_offline::Get_Major_Mic_srv&) { return value(); }
};

template<>
struct DataType< ::xf_mic_asr_offline::Get_Major_Mic_srv > {
  static const char* value()
  {
    return "xf_mic_asr_offline/Get_Major_Mic_srv";
  }

  static const char* value(const ::xf_mic_asr_offline::Get_Major_Mic_srv&) { return value(); }
};


// service_traits::MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srvRequest> should match
// service_traits::MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srv >
template<>
struct MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srv >::value();
  }
  static const char* value(const ::xf_mic_asr_offline::Get_Major_Mic_srvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::xf_mic_asr_offline::Get_Major_Mic_srvRequest> should match
// service_traits::DataType< ::xf_mic_asr_offline::Get_Major_Mic_srv >
template<>
struct DataType< ::xf_mic_asr_offline::Get_Major_Mic_srvRequest>
{
  static const char* value()
  {
    return DataType< ::xf_mic_asr_offline::Get_Major_Mic_srv >::value();
  }
  static const char* value(const ::xf_mic_asr_offline::Get_Major_Mic_srvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srvResponse> should match
// service_traits::MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srv >
template<>
struct MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::xf_mic_asr_offline::Get_Major_Mic_srv >::value();
  }
  static const char* value(const ::xf_mic_asr_offline::Get_Major_Mic_srvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::xf_mic_asr_offline::Get_Major_Mic_srvResponse> should match
// service_traits::DataType< ::xf_mic_asr_offline::Get_Major_Mic_srv >
template<>
struct DataType< ::xf_mic_asr_offline::Get_Major_Mic_srvResponse>
{
  static const char* value()
  {
    return DataType< ::xf_mic_asr_offline::Get_Major_Mic_srv >::value();
  }
  static const char* value(const ::xf_mic_asr_offline::Get_Major_Mic_srvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // XF_MIC_ASR_OFFLINE_MESSAGE_GET_MAJOR_MIC_SRV_H
