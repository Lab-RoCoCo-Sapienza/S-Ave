#ifndef _ROS_NCNN_YOLOV5_HEADER_
#define _ROS_NCNN_YOLOV5_HEADER_

#include "platform.h"
#include "net.h"
#include "cpu.h"

#include "semantic_maps/ncnn_utils.h"

typedef struct {
    std::string name;
    int stride;
    std::vector<cv::Size> anchors;
}
YoloLayerData;


struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

static const char* class_names[] = {"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
"fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
"elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
"skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
"tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
"sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
"potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
"microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
"hair drier", "toothbrush"};



class ncnnYoloV5 {
public:

  ncnn::Net neuralnet;


  int detect(const cv::Mat& bgr, std::vector<Object>& objects, uint8_t n_threads);
  void draw(const cv::Mat& bgr, const std::vector<Object>& objects, double dT);

private:
    static std::vector<Object> decode_infer(ncnn::Mat &data, int stride,const cv::Size& frame_size, int net_size,int num_classes,const std::vector<cv::Size>& anchors,float threshold);
    static void nms(std::vector<Object>& result,float nms_threshold);

    int num_class = 80;

    std::vector<YoloLayerData> layers{
        {"691",32,{{116,90},{156,198},{373,326}}}, //394
        {"671",16,{{30,61},{62,45},{59,119}}},  //375
        {"output",8,{{10,13},{16,30},{33,23}}},
    };

};

#endif
