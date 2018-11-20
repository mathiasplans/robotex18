typedef enum{
    DEF,
    CONTOURS,
    MASK
}output_t;

typedef enum{
    BLUE,
    PINK
}basket_t;

uint16_t avg(cv::Mat& frame, cv::Rect);
double polsby_doppler(std::vector<cv::Point>& contour);
void init_thresholds();