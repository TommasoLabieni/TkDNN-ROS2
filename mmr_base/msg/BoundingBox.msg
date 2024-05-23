uint8 id
float32 probability

# Bounding Boxes are Compatible with Opencv format
# Top left Corner in img is referred as (0,0)
# Each value is in pixel
#
#   0        X
#  0+-------->
#   |
#   |
#   |
# Y v

uint32 top_left_x
uint32 top_left_y
uint32 width
uint32 height

# Drawing a red Bounding Box to test is simple:
# cv::Rect actualRectangle = cv::Rect( top_left_x, top_left_y, width, height);
# cv::rectangle( frameToModify , actualRectangle, cv::Scalar(0,0,255),3,8,0);
