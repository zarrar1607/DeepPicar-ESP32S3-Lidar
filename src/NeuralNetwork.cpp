#include <ESP_TF.h>

// #if !defined(CONFIG_NN_OPTIMIZED)
// #error "CONFIG_NN_OPTIMIZED"
// #endif
// #if !defined(CONFIG_IDF_TARGET_ESP32S3)
// #error "CONFIG_IDF_TARGET_ESP32S3"
// #endif

#include "NeuralNetwork.h"

//#include "f1_tenth_model.cc"
//#include "opt-160x66x3.cc"
#include <esp_attr.h>
#include <Arduino.h>

// int kTensorArenaSize = 1024 * 1024;
//  Define the tensor arena and tensor arena size
constexpr int kTensorArenaSize = 40000;//1081 * 1081; // Adjust the size accordingly

// Assume num_lidar_range_values is 1081
const int num_lidar_range_values = 1081;                       // Batch size 1, 2 output values

// Define the input and output tensors
TfLiteTensor *input_tensor;
TfLiteTensor *output_tensor;

NeuralNetwork::NeuralNetwork()
{
    printf("Free heap: %d\n", ESP.getFreeHeap());
    printf("largest size (8bit): %d\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    printf("largest size (default): %d\n", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    printf("largest size (spiram): %d\n", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    printf("largest size (internal): %d\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    
    // get model (.tflite) from flash
    model = tflite::GetModel(f1_tenth_model_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        MicroPrintf("Model provided is schema version %d not equal to supported "
                    "version %d.",
                    model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }
    tflite::AllOpsResolver micro_op_resolver;

    //tensor_arena = (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    tensor_arena = (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
    
    if (tensor_arena == NULL)
    {
        printf("Couldn't allocate memory of %d bytes\n", kTensorArenaSize);
        return;
    }
    printf("Free heap: %d\n", ESP.getFreeHeap());

    // Build an interpreter to run the model with.
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    MicroPrintf("interpreter initialization");
    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        MicroPrintf("AllocateTensors() failed");
        return;
    }
    
    size_t used_bytes = interpreter->arena_used_bytes();
    MicroPrintf("Used bytes %d\n", used_bytes);

    // Obtain pointers to the model's input and output tensors.
    input = interpreter->input(0);
    output = interpreter->output(0);

    printf("tensor_arena: %p, input: %p\n", tensor_arena, input->data.uint8);
}


float *NeuralNetwork::getInputBuffer()
{
    return input->data.f;
}

TfLiteStatus NeuralNetwork::predict()
{
    return interpreter->Invoke();
}

float *NeuralNetwork::getOutput()  // Modified to return a pointer to the output tensor's data
{
    return output->data.f;
}