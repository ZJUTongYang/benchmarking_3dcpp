// Some common type definitions used by both CPU and GPU

enum ToolType
{
    CIRCULAR_TOOL = 0, 
    LINE_LIDAR_TOOL = 1
};

struct ToolParameters{
    float param1;
    float param2;
    float param3;
};
