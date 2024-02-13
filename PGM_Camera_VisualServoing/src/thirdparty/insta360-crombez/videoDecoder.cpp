#include "videoDecoder.h"

videoDecoder::videoDecoder(){

    av_log_set_level(0);

    pkt = av_packet_alloc();
    if (!pkt){
        exit(1);
    }

    avcodec_register_all();

    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec){
        fprintf(stderr, "Codec not found\n");
        exit(1);
    }

    parser = av_parser_init(codec->id);
    if (!parser){
        fprintf(stderr, "parser not found\n");
        exit(1);
    }

    c = avcodec_alloc_context3(codec);
    if (!c){
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }

    if (avcodec_open2(c, codec, NULL) < 0){
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }

    frame = av_frame_alloc();
    if (!frame){
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }

}


videoDecoder::~videoDecoder(){
    sws_freeContext(sws_ctx);
    av_parser_close(parser);
    avcodec_free_context(&c);
    av_frame_free(&frame);
    av_packet_free(&pkt);
}


int videoDecoder::parse(const uint8_t* data, size_t data_size){
    int ret;

    ret = av_parser_parse2(parser, c, &pkt->data, &pkt->size, data, (int)data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    if (ret < 0){
        fprintf(stderr, "Error while parsing\n");
        exit(1);
    }

    return ret;
}

cv::Mat videoDecoder::decode(const uint8_t* data, size_t data_size){
    cv::Mat I(0,0,0);
    int ret;
    int sts;

    ret = avcodec_send_packet(c, pkt);
    if (ret < 0){
        return I ; //Error!
    }else {
        //Create SWS Context for converting from decode pixel format (like YUV420) to RGB
        ////////////////////////////////////////////////////////////////////////////
        sws_ctx = sws_getContext(c->width,
                                 c->height,
                                 AV_PIX_FMT_YUV420P,
                                 c->width,
                                 c->height,
                                 AV_PIX_FMT_BGR24,
                                 SWS_FAST_BILINEAR ,
                                 NULL,
                                 NULL,
                                 NULL);

        if (sws_ctx == nullptr) {
            return I ; //Error!
        }
        ////////////////////////////////////////////////////////////////////////////

        //Allocate frame for storing image converted to RGB.
        ////////////////////////////////////////////////////////////////////////////
        pRGBFrame = av_frame_alloc();
        pRGBFrame->format = AV_PIX_FMT_BGR24;
        pRGBFrame->width = c->width;
        pRGBFrame->height = c->height;

        sts = av_frame_get_buffer(pRGBFrame, 0);

        if (sts < 0) {
            return I ; //Error!
        }
        ////////////////////////////////////////////////////////////////////////////

        while (ret >= 0) {
            ret = avcodec_receive_frame(c, frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return I ; //Error!
            } else if (ret < 0) {
                fprintf(stderr, "Error during decoding\n");
                exit(1);
            }

            ////////////////////////////////////////////////////////////////////////////
            sts = sws_scale(sws_ctx,                //struct SwsContext* c,
                            frame->data,            //const uint8_t* const srcSlice[],
                            frame->linesize,        //const int srcStride[],
                            0,              //int srcSliceY,
                            frame->height,          //int srcSliceH,
                            pRGBFrame->data,        //uint8_t* const dst[],
                            pRGBFrame->linesize);   //const int dstStride[]);

            if (sts != frame->height) {
                return I ; //Error!
            }

            I.create(frame->height,frame->width, CV_8UC3);
            memcpy(I.data, pRGBFrame->data[0], I.total() * I.elemSize());
            av_frame_free(&pRGBFrame);

            return I ;
            ////////////////////////////////////////////////////////////////////////////
        }
    }
    return I ; //Error!
}