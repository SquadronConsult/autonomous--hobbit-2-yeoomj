/**
 * @file pipeline.cpp
 * @brief Core DeepStream pipeline implementation for agricultural drone surveillance
 * @version 1.0
 */

#include <nvds_meta.h>      // DeepStream SDK 6.2
#include <gstnvdsmeta.h>    // DeepStream SDK 6.2
#include <gst_buffer.h>     // GStreamer 1.0
#include <cuda_runtime_api.h> // CUDA 11.4
#include "processors/objectDetection.cpp"
#include "processors/videoAnalytics.cpp"
#include "utils/tensorrt.cpp"

#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <stdexcept>

// Global constants for pipeline configuration
constexpr int MAX_STREAMS = 8;
constexpr int BATCH_TIMEOUT_USEC = 40000;
constexpr int DEFAULT_STREAM_WIDTH = 1920;
constexpr int DEFAULT_STREAM_HEIGHT = 1080;
constexpr const char* PIPELINE_NAME = "agricultural_analytics";
constexpr int MAX_BATCH_SIZE = 4;
constexpr int MIN_BATCH_SIZE = 1;
constexpr int MEMORY_POOL_SIZE = 32;
constexpr int MAX_LATENCY_MS = 100;

/**
 * @class DeepStreamPipeline
 * @brief High-performance video analytics pipeline for agricultural monitoring
 */
class DeepStreamPipeline {
private:
    GstElement* pipeline;
    std::unique_ptr<VideoAnalytics> videoAnalytics;
    std::unique_ptr<ObjectDetector> objectDetector;
    std::unique_ptr<TensorRTLogger> logger;
    NvDsPipelineConfig* config;
    std::atomic<bool> is_running;
    cudaStream_t cuda_stream;

    // Performance monitoring
    struct PipelineMetrics {
        std::atomic<uint64_t> frames_processed{0};
        std::atomic<uint64_t> total_latency_ms{0};
        std::atomic<uint64_t> batch_count{0};
        std::atomic<uint64_t> dropped_frames{0};
        std::chrono::steady_clock::time_point last_report_time;
    } metrics;

    /**
     * @brief Initializes GStreamer pipeline elements
     * @return Success status of pipeline initialization
     */
    bool initializePipelineElements() {
        // Create pipeline
        pipeline = gst_pipeline_new(PIPELINE_NAME);
        if (!pipeline) {
            logger->log(nvinfer1::ILogger::Severity::kERROR, 
                       "Failed to create pipeline");
            return false;
        }

        // Create and configure source elements for each stream
        for (int i = 0; i < config->num_streams; i++) {
            GstElement* source = gst_element_factory_make("nvarguscamerasrc", nullptr);
            if (!source) {
                logger->log(nvinfer1::ILogger::Severity::kERROR, 
                          "Failed to create source element");
                return false;
            }

            // Configure source properties
            g_object_set(G_OBJECT(source),
                        "sensor-id", i,
                        "bufapi-version", 1,
                        nullptr);

            // Add source to pipeline
            gst_bin_add(GST_BIN(pipeline), source);
        }

        // Create and configure video converter
        GstElement* converter = gst_element_factory_make("nvvideoconvert", nullptr);
        if (!converter) {
            logger->log(nvinfer1::ILogger::Severity::kERROR, 
                       "Failed to create converter element");
            return false;
        }
        gst_bin_add(GST_BIN(pipeline), converter);

        // Create and configure streammux
        GstElement* streammux = gst_element_factory_make("nvstreammux", nullptr);
        if (!streammux) {
            logger->log(nvinfer1::ILogger::Severity::kERROR, 
                       "Failed to create streammux element");
            return false;
        }

        g_object_set(G_OBJECT(streammux),
                    "batch-size", MAX_BATCH_SIZE,
                    "width", DEFAULT_STREAM_WIDTH,
                    "height", DEFAULT_STREAM_HEIGHT,
                    "batched-push-timeout", BATCH_TIMEOUT_USEC,
                    nullptr);
        gst_bin_add(GST_BIN(pipeline), streammux);

        return true;
    }

    /**
     * @brief Sets up pipeline memory pools and CUDA resources
     * @return Success status of resource initialization
     */
    bool initializeResources() {
        // Create CUDA stream
        cudaError_t cuda_status = cudaStreamCreateWithPriority(
            &cuda_stream, 
            cudaStreamNonBlocking,
            0
        );
        if (cuda_status != cudaSuccess) {
            logger->log(nvinfer1::ILogger::Severity::kERROR,
                       "Failed to create CUDA stream");
            return false;
        }

        // Initialize memory pools
        NvBufSurfaceCreateParams create_params = {0};
        create_params.width = DEFAULT_STREAM_WIDTH;
        create_params.height = DEFAULT_STREAM_HEIGHT;
        create_params.size = DEFAULT_STREAM_WIDTH * DEFAULT_STREAM_HEIGHT * 3;
        create_params.memType = NVBUF_MEM_CUDA_DEVICE;

        for (int i = 0; i < MEMORY_POOL_SIZE; i++) {
            NvBufSurface* surface = nullptr;
            if (NvBufSurfaceCreate(&surface, 1, &create_params) != 0) {
                logger->log(nvinfer1::ILogger::Severity::kERROR,
                          "Failed to create surface buffer");
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Handles pipeline state changes
     * @param old_state Previous pipeline state
     * @param new_state New pipeline state
     */
    void handleStateChange(GstState old_state, GstState new_state) {
        switch (new_state) {
            case GST_STATE_PLAYING:
                logger->log(nvinfer1::ILogger::Severity::kINFO,
                          "Pipeline started successfully");
                metrics.last_report_time = std::chrono::steady_clock::now();
                break;

            case GST_STATE_PAUSED:
                logger->log(nvinfer1::ILogger::Severity::kINFO,
                          "Pipeline paused");
                break;

            case GST_STATE_READY:
                logger->log(nvinfer1::ILogger::Severity::kINFO,
                          "Pipeline ready");
                break;

            case GST_STATE_NULL:
                logger->log(nvinfer1::ILogger::Severity::kINFO,
                          "Pipeline stopped");
                break;

            default:
                break;
        }
    }

public:
    /**
     * @brief Initializes DeepStream pipeline with configuration
     * @param pipelineConfig Pipeline configuration parameters
     */
    DeepStreamPipeline(const NvDsPipelineConfig* pipelineConfig) 
        : config(const_cast<NvDsPipelineConfig*>(pipelineConfig))
        , is_running(false) {
        
        // Initialize GStreamer
        gst_init(nullptr, nullptr);

        // Create logger
        logger = std::make_unique<TensorRTLogger>(
            nvinfer1::ILogger::Severity::kWARNING,
            "pipeline.log"
        );

        // Initialize video analytics
        NvDsVideoConfig video_config = {0};
        video_config.width = DEFAULT_STREAM_WIDTH;
        video_config.height = DEFAULT_STREAM_HEIGHT;
        video_config.fps = 30;

        ProcessorOptions proc_options = {0};
        proc_options.model_path = config->model_path;
        
        videoAnalytics = std::make_unique<VideoAnalytics>(
            &video_config,
            &proc_options
        );

        // Initialize object detector
        NvDsInferContextInitParams infer_params = {0};
        infer_params.gpuID = 0;
        infer_params.maxBatchSize = MAX_BATCH_SIZE;
        
        objectDetector = std::make_unique<ObjectDetector>(
            config->model_path,
            &infer_params
        );

        // Initialize pipeline elements and resources
        if (!initializePipelineElements() || !initializeResources()) {
            throw std::runtime_error("Failed to initialize pipeline");
        }
    }

    /**
     * @brief Starts the DeepStream pipeline
     * @return Success status of pipeline start
     */
    bool start() {
        if (is_running) return false;

        // Start video analytics
        if (!videoAnalytics->start()) {
            logger->log(nvinfer1::ILogger::Severity::kERROR,
                       "Failed to start video analytics");
            return false;
        }

        // Set pipeline state to playing
        GstStateChangeReturn ret = gst_element_set_state(
            pipeline,
            GST_STATE_PLAYING
        );

        if (ret == GST_STATE_CHANGE_FAILURE) {
            logger->log(nvinfer1::ILogger::Severity::kERROR,
                       "Failed to start pipeline");
            return false;
        }

        is_running = true;
        return true;
    }

    /**
     * @brief Stops the pipeline and releases resources
     */
    void stop() {
        if (!is_running) return;

        // Stop pipeline
        gst_element_set_state(pipeline, GST_STATE_NULL);
        
        // Stop video analytics
        videoAnalytics->stop();

        // Release CUDA resources
        cudaStreamDestroy(cuda_stream);

        is_running = false;
    }

    /**
     * @brief Retrieves current pipeline metrics
     * @return Pipeline performance metrics
     */
    PipelineMetrics getMetrics() const {
        return metrics;
    }

    /**
     * @brief Destructor
     */
    ~DeepStreamPipeline() {
        if (is_running) {
            stop();
        }

        if (pipeline) {
            gst_object_unref(pipeline);
        }
    }
};

// Export DeepStreamPipeline interface
extern "C" {
    DeepStreamPipeline* createPipeline(const NvDsPipelineConfig* config) {
        return new DeepStreamPipeline(config);
    }

    bool startPipeline(DeepStreamPipeline* pipeline) {
        return pipeline->start();
    }

    void stopPipeline(DeepStreamPipeline* pipeline) {
        pipeline->stop();
    }

    void destroyPipeline(DeepStreamPipeline* pipeline) {
        delete pipeline;
    }
}