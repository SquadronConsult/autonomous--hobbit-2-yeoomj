/**
 * @file objectDetection.cpp
 * @brief DeepStream object detection processor for real-time agricultural monitoring
 * @version 1.0
 */

#include <nvds_meta.h>      // DeepStream SDK 6.2
#include <gstnvdsmeta.h>    // DeepStream SDK 6.2
#include <NvInfer.h>        // TensorRT 8.5
#include <cuda_runtime.h>   // CUDA 11.4
#include "../utils/tensorrt.cpp"

#include <memory>
#include <vector>
#include <stdexcept>

// Global constants for detection configuration
constexpr float CONFIDENCE_THRESHOLD = 0.95f;  // 95% detection accuracy requirement
constexpr int MAX_BATCH_SIZE = 8;             // Support for 8+ simultaneous drone feeds
constexpr int MAX_OBJECTS_PER_FRAME = 100;    // Maximum detectable objects per frame
constexpr int MODEL_INPUT_WIDTH = 1920;       // Input frame width
constexpr int MODEL_INPUT_HEIGHT = 1080;      // Input frame height

/**
 * @class ObjectDetector
 * @brief Real-time object detection processor for agricultural monitoring
 */
class ObjectDetector {
private:
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    void* device_buffers[2];  // Input and output buffers
    TensorRTLogger* logger;
    NvDsInferContext* infer_context;
    cudaStream_t cuda_stream;

    /**
     * @brief Preprocesses video frame for inference
     * @param input_buffer Input frame buffer
     * @param cuda_buffer CUDA memory buffer
     * @return Preprocessing success status
     */
    bool preprocessFrame(NvBufSurface* input_buffer, void* cuda_buffer) {
        // Convert frame to CUDA memory
        NvBufSurfaceParams* surface_params = &input_buffer->surfaceList[0];
        
        // Resize to model input dimensions
        NvBufSurfTransformParams transform_params = {0};
        transform_params.transform_flag = NVBUFSURF_TRANSFORM_FILTER |
                                       NVBUFSURF_TRANSFORM_CROP_SRC;
        transform_params.transform_flip = NvBufSurfTransform_None;
        transform_params.transform_filter = NvBufSurfTransformInter_Bilinear;
        
        transform_params.src_rect = {0, 0, 
                                   surface_params->width,
                                   surface_params->height};
        transform_params.dst_rect = {0, 0, 
                                   MODEL_INPUT_WIDTH,
                                   MODEL_INPUT_HEIGHT};

        // Apply transformation
        NvBufSurfTransform(input_buffer, input_buffer, &transform_params);

        // Normalize pixel values and convert to float32
        cudaError_t cuda_status = cudaMemcpyAsync(
            cuda_buffer,
            surface_params->dataPtr,
            MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * 3 * sizeof(float),
            cudaMemcpyDeviceToDevice,
            cuda_stream
        );

        return cuda_status == cudaSuccess;
    }

    /**
     * @brief Processes raw model outputs into structured detection results
     * @param detection_output Raw detection output
     * @param meta_list Metadata list for results
     * @return Postprocessing success status
     */
    bool postprocessDetections(float* detection_output, NvDsMetaList* meta_list) {
        NvDsObjectMeta* obj_meta = nullptr;
        
        // Process each detection
        for (int i = 0; i < MAX_OBJECTS_PER_FRAME; i++) {
            float confidence = detection_output[i * 6 + 4];
            if (confidence < CONFIDENCE_THRESHOLD) continue;

            // Create object metadata
            obj_meta = nvds_acquire_obj_meta_from_pool(meta_list);
            if (!obj_meta) return false;

            // Set detection coordinates
            obj_meta->rect_params.left = detection_output[i * 6];
            obj_meta->rect_params.top = detection_output[i * 6 + 1];
            obj_meta->rect_params.width = detection_output[i * 6 + 2];
            obj_meta->rect_params.height = detection_output[i * 6 + 3];
            
            // Set detection class and confidence
            obj_meta->class_id = static_cast<int>(detection_output[i * 6 + 5]);
            obj_meta->confidence = confidence;
            
            // Set tracking info
            obj_meta->object_id = nvds_acquire_obj_id();
            obj_meta->tracker_confidence = confidence;

            // Attach metadata to frame
            nvds_add_obj_meta_to_frame(meta_list, obj_meta);
        }

        return true;
    }

public:
    /**
     * @brief Initializes object detector with model and configuration
     * @param model_path Path to TensorRT model
     * @param init_params Inference initialization parameters
     */
    ObjectDetector(const char* model_path, 
                  const NvDsInferContextInitParams* init_params) {
        // Initialize TensorRT logger
        logger = new TensorRTLogger(nvinfer1::ILogger::Severity::kWARNING,
                                  "object_detector.log");

        // Configure optimization settings
        TensorRTOptimizationConfig config;
        config.maxBatchSize = MAX_BATCH_SIZE;
        config.fp16Enabled = true;  // Enable FP16 for performance
        
        // Create inference engine
        engine = createInferenceEngine(model_path, config, true);
        if (!engine) {
            throw std::runtime_error("Failed to create inference engine");
        }

        // Create execution context
        context = engine->createExecutionContext();
        if (!context) {
            throw std::runtime_error("Failed to create execution context");
        }

        // Allocate CUDA memory
        cudaError_t cuda_status = cudaStreamCreate(&cuda_stream);
        if (cuda_status != cudaSuccess) {
            throw std::runtime_error("Failed to create CUDA stream");
        }

        // Allocate device buffers
        for (int i = 0; i < engine->getNbBindings(); i++) {
            nvinfer1::Dims dims = engine->getBindingDimensions(i);
            size_t size = std::accumulate(dims.d, dims.d + dims.nbDims, 1,
                                        std::multiplies<int>()) * sizeof(float);
            cudaMalloc(&device_buffers[i], size);
        }

        // Initialize DeepStream context
        if (NvDsInferContextInit(init_params, &infer_context) != NVDSINFER_SUCCESS) {
            throw std::runtime_error("Failed to initialize inference context");
        }
    }

    /**
     * @brief Performs object detection on a batch of frames
     * @param input_batch Input frame batch
     * @param meta_list Metadata list for results
     * @return Detection success status
     */
    bool detect(NvBufSurface* input_batch, NvDsMetaList* meta_list) {
        if (!input_batch || !meta_list) return false;

        // Preprocess input frames
        if (!preprocessFrame(input_batch, device_buffers[0])) {
            logger->log(nvinfer1::ILogger::Severity::kERROR,
                       "Frame preprocessing failed");
            return false;
        }

        // Execute inference
        if (!context->executeV2(device_buffers)) {
            logger->log(nvinfer1::ILogger::Severity::kERROR,
                       "Inference execution failed");
            return false;
        }

        // Synchronize CUDA stream
        cudaStreamSynchronize(cuda_stream);

        // Process detections
        float* detection_output;
        cudaMemcpyAsync(&detection_output, device_buffers[1],
                       MAX_OBJECTS_PER_FRAME * 6 * sizeof(float),
                       cudaMemcpyDeviceToHost, cuda_stream);

        return postprocessDetections(detection_output, meta_list);
    }

    /**
     * @brief Releases allocated resources
     */
    void release() {
        // Free CUDA memory
        for (void* buffer : device_buffers) {
            cudaFree(buffer);
        }
        cudaStreamDestroy(cuda_stream);

        // Release TensorRT resources
        if (context) {
            context->destroy();
        }
        if (engine) {
            engine->destroy();
        }

        // Cleanup DeepStream context
        if (infer_context) {
            NvDsInferContextDestroy(infer_context);
        }

        // Delete logger
        delete logger;
    }

    /**
     * @brief Destructor
     */
    ~ObjectDetector() {
        release();
    }
};

// Export ObjectDetector class
extern "C" {
    ObjectDetector* createObjectDetector(const char* model_path,
                                       const NvDsInferContextInitParams* params) {
        return new ObjectDetector(model_path, params);
    }

    bool detectObjects(ObjectDetector* detector,
                      NvBufSurface* input_batch,
                      NvDsMetaList* meta_list) {
        return detector->detect(input_batch, meta_list);
    }

    void destroyObjectDetector(ObjectDetector* detector) {
        delete detector;
    }
}