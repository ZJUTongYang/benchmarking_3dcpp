#pragma once

// Some common type definitions used by both CPU and GPU

enum ToolType
{
    CIRCULAR_TOOL = 0, 
    LINE_LIDAR_TOOL = 1
};

struct CudaToolParameters
{
    float param1;
    float param2;
    float param3;

    int size_triple_array_param1;
    float* float_triple_array_param1;
};

