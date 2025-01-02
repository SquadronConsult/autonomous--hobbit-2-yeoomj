/**
 * @file videoAnalytics.cpp
 * @brief Core video analytics processor for DeepStream pipeline with real-time processing
 * @version 1.0
 */

#include <nvds_meta.h>      // DeepStream SDK 6.2
#include <gstnvdsmeta.h>    // DeepStream SDK 6.2
#include <gst_buffer.h>     // GStreamer 1.0
#include <cuda_runtime_api.h> // CUDA 11.4
#include "objectDetection.cpp"
#include "../utils/tensorrt.cpp"

#include <memory>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>

// Global constants for video processing configuration
constexpr int MAX_BATCH_SIZE = 8;
constexpr int FRAME_MEMORY_POOL_SIZE = 32;
constexpr int DEFAULT_STREAM_WIDTH = 1920;
constexpr int DEFAULT_STREAM_HEIGHT = 1080;
constexpr int TARGET_FPS = 30;
constexpr int MAX_PROCESSING_LATENCY_MS = 100;
constexpr float MIN_DETECTION_CONFIDENCE = 0.75f;
constexpr float MEMORY_POOL_GROWTH_FACTOR = 1.5f;

/**
 * @class VideoAnalytics
 * @brief Advanced video analytics processor with performance optimization
 */
class VideoAnalytics final {
private:
    std::unique_ptr<ObjectDetector> detector;
    std::unique_ptr<TensorRTLogger> logger;
    NvDsFrameBatch* frame_batch;
    NvDsMetaPool* meta_pool;
    cudaStream_t processing_stream;
    std::atomic<bool> is_running;
    
    // Performance monitoring
    struct PerformanceMetrics {
        std::atomic<uint64_t> frames_processed{0};
        std::atomic<uint64_t> total_latency_ms{0};
        std::atomic<uint64_t> detection_count{0};
        std::chrono::steady_clock::time_point last_report_time;
    } metrics;

    // Frame processing queue
    struct FrameQueue {
        std::queue<NvBufSurface*> queue;
        std::mutex mutex;
        std::condition_variable cv;
    } frame_queue;

    // Memory management
    struct MemoryPool {
        std::vector<NvBufSurface*> available_buffers;
        std::mutex mutex;
        size_t total_size;
    } memory_pool;

    /**
     * @brief Initializes CUDA memory pool for frame processing
     * @return Success status of memory pool initialization
     */
    bool initializeMemoryPool() {
        std::lock_guard<std::mutex> lock(memory_pool.mutex);
        memory_pool.total_size = FRAME_MEMORY_POOL_SIZE;

        for (size_t i = 0; i < FRAME_MEMORY_POOL_SIZE; i++) {
            NvBufSurface* surface = nullptr;
            NvBufSurfaceCreateParams create_params = {0};
            create_params.width = DEFAULT_STREAM_WIDTH;
            create_params.height = DEFAULT_STREAM_HEIGHT;
            create_params.size = DEFAULT_STREAM_WIDTH * DEFAULT_STREAM_HEIGHT * 3;
            create_params.memType = NVBUF_MEM_CUDA_DEVICE;

            if (NvBufSurfaceCreate(&surface, 1, &create_params) != 0) {
                logger->log(nvinfer1::ILogger::Severity::kERROR, 
                          "Failed to create surface buffer");
                return false;
            }
            memory_pool.available_buffers.push_back(surface);
        }
        return true;
    }

    /**
     * @brief Processes a batch of frames with optimized memory handling
     * @param batch Input frame batch
     * @param batch_meta Batch metadata
     * @return Processing success status
     */
    bool processBatchInternal(NvBufSurface* batch, NvDsMetaList* batch_meta) {
        auto start_time = std::chrono::steady_clock::now();

        // Perform object detection
        if (!detector->detect(batch, batch_meta)) {
            logger->log(nvinfer1::ILogger::Severity::kERROR, 
                       "Object detection failed");
            return false;
        }

        // Update performance metrics
        auto end_time = std::chrono::steady_clock::now();
        auto latency = std::chrono::duration_cast<std::chrono::milliseconds>
                      (end_time - start_time).count();
        
        metrics.frames_processed += batch->numFilled;
        metrics.total_latency_ms += latency;

        // Check latency requirements
        if (latency > MAX_PROCESSING_LATENCY_MS) {
            logger->log(nvinfer1::ILogger::Severity::kWARNING,
                       "Processing latency exceeded threshold");
        }

        return true;
    }

public:
    /**
     * @brief Initializes video analytics with configuration
     * @param config Video configuration parameters
     * @param options Processor options
     */
    VideoAnalytics(const NvDsVideoConfig* config, const ProcessorOptions* options) {
        // Initialize CUDA stream
        cudaStreamCreateWithPriority(&processing_stream, cudaStreamNonBlocking, 0);

        // Initialize TensorRT logger
        logger = std::make_unique<TensorRTLogger>(
            nvinfer1::ILogger::Severity::kWARNING,
            "video_analytics.log"
        );

        // Initialize object detector
        NvDsInferContextInitParams infer_params = {0};
        infer_params.gpuID = 0;
        infer_params.maxBatchSize = MAX_BATCH_SIZE;
        detector = std::make_unique<ObjectDetector>(
            options->model_path,
            &infer_params
        );

        // Initialize frame batch handling
        frame_batch = nvds_frame_batch_create(MAX_BATCH_SIZE);
        meta_pool = nvds_meta_pool_create(MAX_BATCH_SIZE);

        // Initialize memory pool
        if (!initializeMemoryPool()) {
            throw std::runtime_error("Failed to initialize memory pool");
        }

        metrics.last_report_time = std::chrono::steady_clock::now();
        is_running = false;
    }

    /**
     * @brief Starts video analytics pipeline
     * @return Success status of pipeline startup
     */
    bool start() {
        if (is_running) return false;
        is_running = true;
        
        logger->log(nvinfer1::ILogger::Severity::kINFO,
                   "Starting video analytics pipeline");
        return true;
    }

    /**
     * @brief Processes video frame batch with optimization
     * @param input_batch Input frame batch
     * @param batch_meta Batch metadata
     * @param options Processing options
     * @return Success status with processing metrics
     */
    bool processBatch(NvBufSurface* input_batch, 
                     NvDsMetaList* batch_meta,
                     const ProcessingOptions* options) {
        if (!is_running || !input_batch || !batch_meta) return false;

        std::lock_guard<std::mutex> lock(frame_queue.mutex);
        
        // Validate batch size
        if (input_batch->numFilled > MAX_BATCH_SIZE) {
            logger->log(nvinfer1::ILogger::Severity::kERROR,
                       "Batch size exceeds maximum limit");
            return false;
        }

        return processBatchInternal(input_batch, batch_meta);
    }

    /**
     * @brief Stops pipeline with cleanup
     */
    void stop() {
        if (!is_running) return;
        is_running = false;

        // Clean up frame queue
        std::lock_guard<std::mutex> lock(frame_queue.mutex);
        while (!frame_queue.queue.empty()) {
            frame_queue.queue.pop();
        }

        // Clean up memory pool
        std::lock_guard<std::mutex> mem_lock(memory_pool.mutex);
        for (auto surface : memory_pool.available_buffers) {
            NvBufSurfaceDestroy(surface);
        }
        memory_pool.available_buffers.clear();

        // Clean up CUDA resources
        cudaStreamDestroy(processing_stream);

        logger->log(nvinfer1::ILogger::Severity::kINFO,
                   "Video analytics pipeline stopped");
    }

    /**
     * @brief Retrieves current performance metrics
     * @return Performance metrics structure
     */
    PerformanceMetrics getMetrics() const {
        return metrics;
    }

    /**
     * @brief Destructor
     */
    ~VideoAnalytics() {
        if (is_running) {
            stop();
        }
    }
};

// Export VideoAnalytics interface
extern "C" {
    VideoAnalytics* createVideoAnalytics(
        const NvDsVideoConfig* config,
        const ProcessorOptions* options
    ) {
        return new VideoAnalytics(config, options);
    }

    bool processVideoBatch(
        VideoAnalytics* processor,
        NvBufSurface* input_batch,
        NvDsMetaList* batch_meta,
        const ProcessingOptions* options
    ) {
        return processor->processBatch(input_batch, batch_meta, options);
    }

    void destroyVideoAnalytics(VideoAnalytics* processor) {
        delete processor;
    }
}