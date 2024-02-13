#ifndef INSTA360VIDEOTOFRAMES_VIDEODECODER_H
#define INSTA360VIDEOTOFRAMES_VIDEODECODER_H

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


class videoDecoder {
public:
    const AVCodec* codec;
    AVCodecParserContext* parser;
    AVCodecContext* c;
    AVFrame* frame;
    AVPacket* pkt;
    struct SwsContext* sws_ctx = NULL;
    AVFrame *pRGBFrame;

    videoDecoder();
    ~videoDecoder();
    int parse(const uint8_t* data, size_t data_size);
    cv::Mat decode(const uint8_t* data, size_t data_size);

};


#endif //INSTA360VIDEOTOFRAMES_VIDEODECODER_H
